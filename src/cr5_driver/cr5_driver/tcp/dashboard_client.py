"""DashboardClient -- TCP connection to Dobot CR5 port 29999.

Handles all control commands: RequestControl, PowerOn, EnableRobot,
DisableRobot, ClearError, SpeedFactor, RobotMode.

CRITICAL: RequestControl() must be called first on every connection.
Without it all commands return 'Control Mode Is Not Tcp' error.
"""


import contextlib
import logging
import socket
import time
from typing import Any, cast, Optional


class DashboardError(Exception):
    """Raised when the CR5 returns a non-zero ErrorID."""

    pass


class DashboardClient:
    """TCP client for CR5 Dashboard port (29999).

    Sends ASCII commands and parses responses in the format:
    ErrorID,{return_values},CommandName(params);
    """

    PORT = 29999
    TIMEOUT = 5.0
    BUFFER_SIZE = 1024

    def __init__(self, robot_ip: str) -> None:
        """Initialise client with robot IP address."""
        self.robot_ip: str = robot_ip
        self.sock: Optional[socket.socket] = None
        self.logger: logging.Logger = logging.getLogger(__name__)

    def connect(self) -> None:
        """Connect to dashboard port and switch to TCP mode.

        MUST be called before any other method.
        RequestControl() is sent immediately after connection.
        """
        self.logger.info(f'Connecting to {self.robot_ip}:{self.PORT}')
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.settimeout(self.TIMEOUT)
        self.sock.connect((self.robot_ip, self.PORT))
        self.logger.info('Connected to dashboard port')

        # MANDATORY FIRST COMMAND -- switch robot to TCP mode
        self._send_and_check('RequestControl()')
        self.logger.info('TCP mode activated')

    def disconnect(self) -> None:
        """Close the dashboard connection."""
        if self.sock:
            with contextlib.suppress(OSError):
                self.sock.close()

            self.sock = None
            self.logger.info('Dashboard connection closed')

    def power_on(self) -> None:
        """Power on the robot arm.

        Takes approximately 10 seconds to complete.
        Always wait after calling this before sending EnableRobot.
        """
        self._send_and_check('PowerOn()')
        self.logger.info('PowerOn sent -- waiting 10 seconds...')
        time.sleep(10.0)
        self.logger.info('Power-on wait complete')

    def enable_robot(self) -> None:
        """Enable the robot arm."""
        self._send_and_check('EnableRobot()')
        self.logger.info('Robot enabled')

    def disable_robot(self) -> None:
        """Disable the robot arm."""
        self._send_and_check('DisableRobot()')
        self.logger.info('Robot disabled')

    def clear_error(self) -> None:
        """Clear robot error state."""
        self._send_and_check('ClearError()')
        self.logger.info('Errors cleared')

    def set_speed(self, speed_percent: float) -> None:
        """Set global speed factor.

        Args:
            speed_percent: Speed as percentage 1-100.
        """
        speed: int = int(max(1, min(100, speed_percent)))
        self._send_and_check(f'SpeedFactor({speed})')
        self.logger.info(f'Speed set to {speed}%')

    def get_robot_mode(self) -> int:
        """Get current robot mode.

        Returns:
            Mode integer. Common values:
            1 = Initialising
            2 = Brake released
            3 = Disabled
            4 = Enabled
            5 = Backdrive
            6 = Running
            7 = Recording
            8 = Error
            9 = Paused
            10 = Jogging
        """
        response: str = self._send_and_check('RobotMode()')
        return self._parse_return_value(response, int)

    def move_j(
            self, x: float, y: float, z: float,
            rx: float, ry: float, rz: float,
            speed: float = 50.0) -> None:
        """Send a joint move command to Cartesian target.

        Args:
            x, y, z: Target position in mm.
            rx, ry, rz: Target orientation in degrees.
            speed: Move speed as percentage 1-100.
        """
        clamped_speed = int(max(1, min(100, speed)))
        self._send_and_check(
            f'SpeedFactor({clamped_speed})'
        )
        self._send_and_check(
            f'MovJ({x},{y},{z},{rx},{ry},{rz})'
        )

    def _send_and_check(self, command: str) -> str:
        """Send command and verify ErrorID is 0.

        Args:
            command: ASCII command string without newline.

        Returns:
            Full response string.

        Raises:
            DashboardError: If ErrorID is non-zero.
            RuntimeError: If not connected.
        """
        if not self.sock:
            raise RuntimeError('Not connected -- call connect() first')

        raw: bytes = (command + '\n').encode('ascii')
        self.sock.sendall(raw)
        self.logger.debug(f'Sent: {command}')

        response: str = self._read_response()
        self.logger.debug(f'Received: {response}')

        error_id: int = self._parse_error_id(response)
        if error_id != 0:
            raise DashboardError(
                f'Command "{command}" failed with ErrorID {error_id}: ' +
                f'{response}'
            )
        return response

    def _read_response(self) -> str:
        """Read response from dashboard port.

        Returns:
            Response string stripped of whitespace.
        """
        sock: socket.socket = cast(socket.socket, self.sock)
        data: bytes = sock.recv(self.BUFFER_SIZE)
        return data.decode('ascii').strip()

    def _parse_error_id(self, response: str) -> int:
        """Extract ErrorID from response string.

        Response format: ErrorID,{values},CommandName(params);

        Args:
            response: Raw response string from robot.

        Returns:
            ErrorID as integer.
        """
        try:
            return int(response.split(',')[0])
        except (ValueError, IndexError) as e:
            raise DashboardError(
                f'Could not parse ErrorID from response: {response}'
            ) from e

    def _parse_return_value(self, response: str, cast_type: type = str) -> Any:
        """Extract return value from response string.

        Response format: ErrorID,{value},CommandName(params);

        Args:
            response: Raw response string from robot.
            cast_type: Type to cast the extracted value to.

        Returns:
            Extracted return value cast to cast_type.
        """
        try:
            # Extract content between { and }
            start: int = response.index('{') + 1
            end: int = response.index('}')
            value: str = response[start:end].strip()
            return cast_type(value)

        except (ValueError, IndexError) as e:
            raise DashboardError(
                f'Could not parse return value from response: {response}'
            ) from e
