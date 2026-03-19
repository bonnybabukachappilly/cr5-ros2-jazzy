from enum import IntEnum


class RobotMode(IntEnum):
    """CR5 robot operating modes from TCP-IP Protocol V4.6.5."""

    UNKNOWN = 0
    INIT = 1
    BRAKE_OPEN = 2
    POWER_OFF = 3
    DISABLED = 4
    ENABLED = 5
    BACKDRIVE = 6
    RUNNING = 7
    SINGLE_MOVE = 8
    ERROR = 9
    PAUSED = 10
    COLLISION = 11
