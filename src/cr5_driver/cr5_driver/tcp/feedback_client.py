"""
DobotFeedbackClient -- TCP client for CR5 real-time feedback port.

Connects to port 30004 and continuously receives 1440-byte state
packets pushed by the robot every 8ms. Stores the latest parsed
packet as a property for consumers to read at their own rate.

Usage:
    # First call -- initialise with robot IP
    get_feedback_client('192.168.0.141').start()

    # Anywhere else -- get cached instance and read latest data
    fb = get_feedback_client('192.168.0.141').message
"""

import contextlib
import struct
from functools import lru_cache
from socket import AF_INET, SHUT_RDWR, SOCK_STREAM, socket, timeout
import time
from typing import Callable, Optional

from cr5_driver.robot.feedback_model import DobotFeedbackModel

from rclpy.impl.rcutils_logger import RcutilsLogger


class DobotFeedbackClient:
    """
    TCP client for CR5 real-time feedback port (30004).

    Receives 1440-byte packets continuously from the robot and
    stores the latest parsed state as the `message` property.
    Consumers read `message` at their own rate without coupling
    to the feedback thread.

    Extend by overriding the hook methods:
        _on_connect, _on_disconnect, _on_error, _on_message
    """

    __slots__ = (
        '_host', '_port', '_buffer_size',
        '_socket', '_is_running', '_message',
        '_callbacks', '_log', '_last_valid_packet_time',
        '_watchdog_threshold'
    )

    def __init__(
            self,
            host: str,
            port: int,
            log: RcutilsLogger,
            buffer_size: int = 1440) -> None:
        """
        Initialise the feedback client.

        Parameters
        ----------
        host : str
            IP address of the CR5 controller.
        port : int
            Feedback port number.
        buffer_size : int
            Expected packet size in bytes. Default 1440.

        """
        self._host: str = host
        self._port: int = port
        self._log: RcutilsLogger = log
        self._buffer_size: int = buffer_size
        self._socket: Optional[socket] = None
        self._is_running: bool = False
        self._message: Optional[DobotFeedbackModel] = None
        self._callbacks: list[Callable[[DobotFeedbackModel], None]] = []
        self._last_valid_packet_time: float = time.monotonic()
        self._watchdog_threshold = 0.1

    @property
    def message(self) -> Optional[DobotFeedbackModel]:
        """
        Return the most recently received feedback packet.

        Returns None if no packet has been received yet.

        Returns
        -------
        Optional[DobotFeedbackModel]
            Latest parsed robot state, or None.

        """
        return self._message

    @property
    def is_healthy(self) -> bool:
        """Check socket is healthy."""
        return (
            time.monotonic() - self._last_valid_packet_time
        ) < self._watchdog_threshold

    def register_callback(
            self, callbacks: Callable[[DobotFeedbackModel], None]) -> None:
        """
        Register a callback to be called on every new message.

        Parameters
        ----------
        callback : Callable[[DobotFeedbackModel], None]
            Function called with parsed feedback on each packet.
            Duplicate registrations are ignored.

        """
        if callbacks not in self._callbacks:
            self._log.debug(f'New call back registered {callbacks}')
            self._callbacks.append(callbacks)
            return

        self._log.warning(f'{callbacks} already registered. Ignoring for now.')

    def start(self) -> None:
        """
        Connect to the robot and start receiving feedback.

        Blocks until the connection is closed or an error occurs.
        Call from a background thread to avoid blocking the caller.
        """
        try:
            self._socket_connect()

        except Exception as e:
            self._on_error(f'Connection Failure: {e}')

        finally:
            self.stop()

    def stop(self) -> None:
        """
        Stop receiving and close the connection.

        Safe to call multiple times. Sets is_running to False
        which causes the receive loop to exit cleanly.
        """
        self._log.info('Stopping dobot feedback socket')

        self._is_running = False
        if self._socket:
            with contextlib.suppress(Exception):
                self._socket.shutdown(SHUT_RDWR)
                self._socket.close()
            self._socket = None

        self._log.info('Client disconnected.')

    def _socket_connect(self) -> None:
        """
        Create socket, connect to robot, then start receive loop.

        Sets a 5 second timeout for the initial connection and
        a 1 second timeout for ongoing receives so the loop can
        check is_running periodically.

        Raises
        ------
        ConnectionRefusedError
            If the robot is not reachable at host:port.
        OSError
            If the socket cannot be created or connected.

        """
        self._log.info('Starting dobot feedback socket')

        self._log.info(
            f'Attempting to connect to Dobot at '
            f'{self._host}:{self._port}...'
        )

        self._socket = socket(AF_INET, SOCK_STREAM)
        self._socket.settimeout(5.0)
        self._socket.connect((self._host, self._port))
        self._is_running = True

        self._log.info(f'Connected to {self._host}')

        # Reduce timeout for receive loop so is_running is checked
        # at least once per second
        self._socket.settimeout(1.0)
        self._receive_loop()

    def _receive_loop(self) -> None:
        """
        Continuously receive 1440-byte packets from the robot.

        Accumulates chunks until a full packet is received, then
        calls _on_message. On recv timeout, checks is_running and
        retries. Exits when is_running is False or connection drops.
        """
        assert self._socket is not None
        self._on_connect((self._host, self._port))

        stream_buffer = bytearray()
        self._last_valid_packet_time = time.monotonic()

        try:
            while self._is_running:
                time_since_last: float = (
                    time.monotonic() - self._last_valid_packet_time)

                if time_since_last > self._watchdog_threshold:
                    self._log.error(
                        'Watchdog Triggered: No valid data for ' +
                        f'{time_since_last:.3f}s')

                    self._last_valid_packet_time = time.monotonic()
                try:
                    chunk: bytes = self._socket.recv(4096)
                    if not chunk:
                        self._on_error('Robot closed the connection.')
                        return
                    stream_buffer.extend(chunk)
                except timeout:
                    continue

                while len(stream_buffer) >= self._buffer_size:
                    msg_size = struct.unpack_from('<H', stream_buffer, 0)[0]

                    if msg_size == self._buffer_size:
                        self._last_valid_packet_time = time.monotonic()
                        if len(stream_buffer) >= (self._buffer_size * 2):
                            del stream_buffer[:self._buffer_size]
                            continue

                        packet = bytes(stream_buffer[:self._buffer_size])
                        self._last_valid_packet_time = time.monotonic()

                        self._on_message(packet, (self._host, self._port))
                        del stream_buffer[:self._buffer_size]
                    else:
                        next_header: int = stream_buffer.find(b'\xa0\x05', 1)
                        if next_header == -1:
                            del stream_buffer[:-1]
                        else:
                            del stream_buffer[:next_header]

        except OSError as e:
            if self._is_running:
                self._on_error(f'Read Error: {e}')
        finally:
            self._on_disconnect((self._host, self._port))

    def _on_message(self, raw_data: bytes, addr: tuple) -> None:
        """
        Parse raw packet and update latest message.

        Called on every successfully received 1440-byte packet.
        Override to add custom behavior on message receipt.

        Parameters
        ----------
        raw_data : bytes
            Raw 1440-byte packet from the robot.
        addr : tuple
            (host, port) of the sender.

        """
        try:
            self._message = DobotFeedbackModel.from_bytes(raw_data)

            if self._callbacks:
                for publish in self._callbacks:
                    publish(self._message)

            if self._message.error_code != 0:
                self._log.debug('Robot reported an error status!')

        except (ValueError, struct.error) as e:
            self._on_error(f'Parsing error: {e}')

    def _on_error(self, error_msg: str) -> None:
        """
        Handle an error condition.

        Override to route errors to a logger or ROS2 node.

        Parameters
        ----------
        error_msg : str
            Human-readable error description.

        """
        self._log.error(f'[ERROR] {error_msg}')

    def _on_connect(self, addr: tuple) -> None:
        """
        Handle connection establishment with the robot.

        Called after a successful TCP connection to the feedback port.
        Override to add custom connect behavior.

        Parameters
        ----------
        addr : tuple
            (host, port) of the connected robot.

        """
        self._log.info(f'[STATE] Session started with {addr}')

    def _on_disconnect(self, addr: tuple) -> None:
        """
        Handle an server disconnection.

        Override to add custom disconnect behavior such as
        triggering a reconnect attempt.

        Parameters
        ----------
        addr : tuple
            (host, port) of the disconnected robot.

        """
        self._log.info(f'[STATE] Session ended with {addr}')


@lru_cache(maxsize=1)
def get_feedback_client(
        host: Optional[str] = None,
        log: Optional[RcutilsLogger] = None,
        port: int = 30004) -> DobotFeedbackClient:
    """
    Get or create the shared feedback client instance.

    Uses lru_cache to ensure only one instance exists per
    (host, port) combination. The instance is created on the
    first call and returned from cache on all subsequent calls.

    To reset the instance (e.g. in tests), call:
        get_feedback_client.cache_clear()

    Parameters
    ----------
    host : str
        IP address of the CR5 controller.
    port : int
        Feedback port number. Default 30004.

    Returns
    -------
    DobotFeedbackClient
        The single shared instance for this host and port.

    """
    return DobotFeedbackClient(host, port, log)
