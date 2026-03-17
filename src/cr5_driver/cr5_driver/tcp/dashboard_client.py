"""
DobotDashboardClient -- TCP client for CR5 command/control port.

Connects to port 29999 for sending dashboard commands. Commands are
string-based and responses follow the format:
ErrorID,{return_values},CommandName(param1,param2,...);

Usage:
    client = get_dashboard_client('192.168.0.141', log)
    client.start()
    resp = client.send_command('EnableRobot()')
    print(f"Error ID: {resp.error_id}")
"""
from __future__ import annotations

import contextlib
import threading
from functools import lru_cache
from socket import AF_INET, SHUT_RDWR, SOCK_STREAM, socket
from threading import Lock
from typing import Optional, Self

from cr5_driver.robot.dashboard_model import DobotDashboardModel

from rclpy.impl.rcutils_logger import RcutilsLogger


class DobotDashboardClient:
    """
    TCP client for CR5 control port (29999).

    Provides a thread-safe send_command method that blocks until
    a full response (terminated by ';') is received.
    """

    _instance: Optional[Self] = None

    __slots__ = (
        '_host', '_port', '_socket',
        '_log', '_lock', '_timeout'
    )

    def __new__(cls, host: Optional[str] = None,
                log: Optional[RcutilsLogger] = None,
                port: int = 29999) -> Self:

        if cls._instance is None:
            if host is None or log is None:
                raise RuntimeError(
                    "Missing 2 required positional arguments: 'host' and 'log'"
                )

            cls._instance = super().__new__(cls)

        return cls._instance

    def __init__(
            self,
            host: Optional[str] = None,
            log: Optional[RcutilsLogger] = None,
            port: int = 29999,
            timeout: float = 5.0
    ) -> None:
        """
        Initialise the dashboard client.

        Parameters
        ----------
        host : str
            IP address of the CR5 controller.
        log : RcutilsLogger
            ROS2 node logger for info and error messages.
        port : int
            Dashboard port number. Default 29999.
        timeout : float
            Socket timeout in seconds. Default 5.0.

        """
        if hasattr(self, '_host') and hasattr(self, '_log'):
            return

        assert host is not None
        assert log is not None

        self._host: str = host
        self._port: int = port
        self._log: RcutilsLogger = log
        self._timeout: float = timeout
        self._socket: Optional[socket] = None
        self._lock: Lock = threading.Lock()

    def start(self) -> None:
        """
        Establish connection to the dashboard port.

        Connects the socket and immediately sends RequestControl()
        to activate TCP mode. Must be called before send_command.

        Raises
        ------
        ConnectionError
            If the robot is not reachable at host:port.

        """
        self._log.info(
            f'Connecting to dashboard port at {self._host}:{self._port}'
        )
        self._socket = socket(AF_INET, SOCK_STREAM)
        self._socket.settimeout(self._timeout)
        self._socket.connect((self._host, self._port))
        self._log.info('Connected to dashboard port')
        self.send_command('RequestControl()')
        self._log.info('TCP mode activated')

    def stop(self) -> None:
        """
        Close the dashboard connection cleanly.

        Safe to call multiple times.
        """
        if self._socket:
            with contextlib.suppress(Exception):
                self._socket.shutdown(SHUT_RDWR)
                self._socket.close()
            self._socket = None
        self._log.info('Dashboard connection closed')

    def send_command(self, cmd: str) -> DobotDashboardModel:
        """
        Send a dashboard command with automatic reconnection on failure.
        """
        if not cmd.endswith('\n'):
            cmd += '\n'

        with self._lock:
            try:
                # First Attempt
                return self._execute_transaction(cmd)
            except (OSError, ConnectionError) as e:
                self._log.warning(
                    f'Dashboard connection lost, attempting reconnect... ({e})')

                try:
                    # Clean up old socket and try to re-establish
                    self.stop()
                    self.start()

                    # Second Attempt
                    return self._execute_transaction(cmd)
                except Exception as retry_error:
                    self._log.error(
                        f'Auto-reconnect failed for [{cmd.strip()}]: {retry_error}')
                    raise ConnectionError(
                        f"Robot unreachable after reconnect: {retry_error}") from retry_error

    def _execute_transaction(self, cmd: str) -> DobotDashboardModel:
        """Internal helper to handle the raw send/recv logic."""
        if not self._socket:
            raise ConnectionError('Dashboard socket is not connected.')

        self._socket.sendall(cmd.encode('utf-8'))
        response_buffer: str = ''

        while ';' not in response_buffer:
            if chunk := self._socket.recv(4096).decode('utf-8'):
                response_buffer += chunk

            else:
                raise ConnectionError('Robot closed the dashboard connection.')
        # Dobot sometimes sends multiple responses if commands were queued;
        # ensure we only parse the relevant one.
        first_response: str = response_buffer.strip().split('\n')[0]
        return self._parse_response(first_response)

    def _parse_response(self, response: str) -> DobotDashboardModel:
        """
        Convert raw response string to model instance.

        Parameters
        ----------
        response : str
            Raw response string from the robot.

        Returns
        -------
        DobotDashboardModel
            Parsed response model.

        """
        return DobotDashboardModel.from_str(response)
