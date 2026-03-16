"""
FeedbackClient -- TCP connection to Dobot CR5 port 30004.

Reads 1440-byte state packets pushed by the robot every 8ms.
Handles TCP stream fragmentation and packet boundary sync.
"""


import contextlib
import logging
import socket
import struct
import threading
import time
from typing import Callable, cast, Optional


PACKET_SIZE = 1440
MAGIC: bytes = struct.pack('<I', PACKET_SIZE)  # b'\xa0\x05\x00\x00'


class FeedbackClient:
    """
    TCP client for CR5 real-time feedback port (30004).

    Continuously reads 1440-byte packets at 125Hz in a background
    thread and calls a user-supplied callback with each raw packet.

    Handles:
    - TCP stream fragmentation (recv may return partial packets)
    - Packet boundary sync using magic header bytes
    - Automatic reconnection on connection loss
    """

    PORT = 30004
    TIMEOUT = 5.0
    RECONNECT_DELAY = 2.0

    def __init__(
            self, robot_ip: str, callback: Callable[[bytes], None]) -> None:
        """
        Initialise feedback client.

        Parameters
        ----------
        robot_ip : str
            IP address of the CR5 controller.
        callback : Callable[[bytes], None]
            Function called with each clean 1440-byte packet.

        """
        self.robot_ip: str = robot_ip
        self.callback: Callable[[bytes], None] = callback
        self.sock: Optional[socket.socket] = None
        self.logger: logging.Logger = logging.getLogger(__name__)
        self._buffer: bytearray = bytearray()
        self._running: bool = False
        self._thread: Optional[threading.Thread] = None

    def start(self) -> None:
        """Start the feedback reader thread."""
        self._running = True
        self._thread = threading.Thread(
            target=self._run_loop, name='cr5_feedback', daemon=True
        )
        self._thread.start()
        self.logger.info('Feedback client started')

    def stop(self) -> None:
        """Stop the feedback reader thread."""
        self._running = False
        self._disconnect()
        if self._thread:
            self._thread.join(timeout=3.0)
        self.logger.info('Feedback client stopped')

    def _run_loop(self) -> None:
        """
        Run main feedback loop run in background thread.

        Connects, syncs to packet boundary, then continuously
        reads packets and calls the callback. Reconnects on error.
        """
        while self._running:
            try:
                self._connect()
                self._sync_to_packet_boundary()
                self.logger.info('Synced to packet boundary -- reading')

                while self._running:
                    raw: bytes = self._read_exact(PACKET_SIZE)
                    self.callback(raw)

            except (ConnectionError, OSError, socket.timeout) as e:
                if self._running:
                    self.logger.warning(
                        f'Feedback connection lost: {e}. '
                        f'Reconnecting in {self.RECONNECT_DELAY}s...'
                    )
                    self._disconnect()
                    self._buffer.clear()

                    time.sleep(self.RECONNECT_DELAY)

    def _connect(self) -> None:
        """
        Open TCP connection to feedback port.

        Raises
        ------
        ConnectionError
            If connection cannot be established.

        """
        self.logger.info(f'Connecting to {self.robot_ip}:{self.PORT}')
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.settimeout(self.TIMEOUT)
        self.sock.connect((self.robot_ip, self.PORT))
        self.logger.info('Connected to feedback port')

    def _disconnect(self) -> None:
        """Close the feedback connection."""
        if self.sock:
            with contextlib.suppress(OSError):
                self.sock.close()
            self.sock = None

    def _read_exact(self, n: int) -> bytes:
        """
        Block until exactly n bytes are in the buffer.

        Accumulates recv() calls until enough bytes arrive.
        Never assumes one recv() equals one packet.

        Parameters
        ----------
        n : int
            Exact number of bytes to read.

        Returns
        -------
        bytes
            Exactly n bytes consumed from the buffer.

        Raises
        ------
        ConnectionError
            If the robot closes the connection.

        """
        sock: socket.socket = cast(socket.socket, self.sock)
        while len(self._buffer) < n:
            if chunk := sock.recv(4096):
                self._buffer.extend(chunk)

            else:
                raise ConnectionError('CR5 closed the feedback connection')
        data = bytes(self._buffer[:n])
        del self._buffer[:n]
        return data

    def _sync_to_packet_boundary(self) -> None:
        """
        Scan buffer until magic header is found at position 0.

        Uses the 4-byte packet length header (0xa0050000) as a
        sync word to align to packet boundaries after connect
        or reconnect. Discards any bytes before the header.

        """
        self.logger.debug('Syncing to packet boundary...')
        discarded = 0
        sock: socket.socket = cast(socket.socket, self.sock)

        while True:
            # Fill buffer with at least one full packet worth of data
            while len(self._buffer) < PACKET_SIZE:
                if chunk := sock.recv(4096):
                    self._buffer.extend(chunk)

                else:
                    raise ConnectionError('CR5 closed connection during sync')
            # Search for magic header
            idx: int = self._buffer.find(MAGIC)

            if idx == 0:
                # Already aligned
                if discarded > 0:
                    self.logger.warning(
                        f'Sync: discarded {discarded} bytes to realign')
                return

            elif idx > 0:
                # Discard bytes before the header
                discarded += idx
                del self._buffer[:idx]

            else:
                # Magic not found -- discard all but last 3 bytes
                # (header might be split across next recv)
                discarded += len(self._buffer) - 3
                del self._buffer[:-3]
