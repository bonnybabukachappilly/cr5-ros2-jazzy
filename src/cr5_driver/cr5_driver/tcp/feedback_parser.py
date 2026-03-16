"""
FeedbackParser -- parses Dobot CR5 real-time feedback packets.

Unpacks the 1440-byte binary struct from port 30004 into a
structured Python dataclass using offsets defined in the
Dobot TCP-IP Remote Control Interface Guide V4.6.5.

Packet structure (little-endian throughout):
    Offset 0000: MessageSize     uint16   (always 1440)
    Offset 0008: DigitalInputs   uint64
    Offset 0016: DigitalOutputs  uint64
    Offset 0024: RobotMode       uint64   (5=enabled/idle)
    Offset 0032: TimeStamp       uint64   (ms)
    Offset 0048: TestValue       uint64   (0x0123456789ABCDEF)
    Offset 0064: SpeedScaling    double   (speed ratio %)
    Offset 0192: QTarget[6]      double   (target joint positions, deg)
    Offset 0432: QActual[6]      double   (actual joint positions, deg)
    Offset 0480: QDActual[6]     double   (actual joint speeds, deg/s)
    Offset 0528: IActual[6]      double   (actual joint current, A)
    Offset 0624: ToolVectorActual[6] double (actual TCP pose, mm/deg)
    Offset 0672: TCPSpeedActual[6]  double (actual TCP speed)
    Offset 1026: EnableStatus    uint8    (1=enabled)
    Offset 1029: ErrorStatus     uint8    (0=no error)
    Offset 1038: CollisionState  uint8    (0=no collision)
    Offset 1120: MActual[6]      double   (actual joint torques, Nm)
"""

from dataclasses import dataclass, field
import struct
from typing import List


PACKET_SIZE = 1440

# ── Byte offsets from Dobot TCP-IP Protocol V4.6.5 Section 3 ────────────────
OFFSET_MESSAGE_SIZE = 0       # uint16  -- always 1440
OFFSET_DIGITAL_INPUTS = 8     # uint64
OFFSET_DIGITAL_OUTPUTS = 16   # uint64
OFFSET_ROBOT_MODE = 24        # uint64
OFFSET_TIMESTAMP = 32         # uint64  -- ms
OFFSET_TEST_VALUE = 48        # uint64  -- 0x0123456789ABCDEF
OFFSET_SPEED_SCALING = 64     # double  -- speed ratio %
OFFSET_Q_TARGET = 192         # double[6] -- target joint positions deg
OFFSET_Q_ACTUAL = 432         # double[6] -- actual joint positions deg
OFFSET_QD_ACTUAL = 480        # double[6] -- actual joint speeds deg/s
OFFSET_I_ACTUAL = 528         # double[6] -- actual joint current A
OFFSET_TOOL_VECTOR_ACTUAL = 624  # double[6] -- TCP pose mm/deg
OFFSET_TCP_SPEED_ACTUAL = 672    # double[6] -- TCP speed
OFFSET_ENABLE_STATUS = 1026   # uint8   -- 1=enabled
OFFSET_ERROR_STATUS = 1029    # uint8   -- 0=no error
OFFSET_COLLISION_STATE = 1038  # uint8   -- 0=no collision
OFFSET_M_ACTUAL = 1120        # double[6] -- actual joint torques Nm

# Test value from documentation -- used to verify packet integrity
EXPECTED_TEST_VALUE = 0x0123456789ABCDEF


@dataclass
class RobotFeedback:
    """
    Structured representation of one CR5 feedback packet.

    All joint arrays follow the order [J1, J2, J3, J4, J5, J6].

    """

    # Header
    message_size: int = 0
    robot_mode: int = 0
    timestamp_ms: int = 0
    speed_scaling: float = 0.0

    # Joint state
    joint_positions: List[float] = field(default_factory=lambda: [0.0] * 6)
    joint_speeds: List[float] = field(default_factory=lambda: [0.0] * 6)
    joint_currents: List[float] = field(default_factory=lambda: [0.0] * 6)
    joint_torques: List[float] = field(default_factory=lambda: [0.0] * 6)
    joint_positions_target: List[float] = field(
        default_factory=lambda: [0.0] * 6
    )

    # Cartesian state
    tcp_pose: List[float] = field(default_factory=lambda: [0.0] * 6)
    tcp_speed: List[float] = field(default_factory=lambda: [0.0] * 6)

    # I/O and status
    digital_inputs: int = 0
    digital_outputs: int = 0
    enabled: bool = False
    error_status: int = 0
    collision_state: int = 0


class FeedbackParser:
    """
    Parses raw 1440-byte feedback packets into RobotFeedback objects.

    Uses a stateful buffer to handle TCP stream fragmentation.
    Validates packet size and test value before parsing.

    """

    @staticmethod
    def parse(raw: bytes) -> RobotFeedback:
        """
        Parse a single 1440-byte packet into a RobotFeedback object.

        Parameters
        ----------
        raw : bytes
            Exactly 1440 bytes from the feedback port.

        Returns
        -------
        RobotFeedback
            Parsed robot state.

        Raises
        ------
        ValueError
            If packet size is wrong or test value fails validation.

        """
        if len(raw) != PACKET_SIZE:
            raise ValueError(
                f'Expected {PACKET_SIZE} bytes, got {len(raw)}'
            )

        # Validate test value -- confirms packet integrity
        test_value = struct.unpack_from('<Q', raw, OFFSET_TEST_VALUE)[0]
        if test_value != EXPECTED_TEST_VALUE:
            raise ValueError(
                f'Packet integrity check failed. '
                f'TestValue={hex(test_value)}, '
                f'expected={hex(EXPECTED_TEST_VALUE)}'
            )

        fb = RobotFeedback()

        # Header fields
        fb.message_size = struct.unpack_from('<H', raw, OFFSET_MESSAGE_SIZE)[0]
        fb.robot_mode = struct.unpack_from('<Q', raw, OFFSET_ROBOT_MODE)[0]
        fb.timestamp_ms = struct.unpack_from('<Q', raw, OFFSET_TIMESTAMP)[0]
        fb.speed_scaling = struct.unpack_from(
            '<d', raw, OFFSET_SPEED_SCALING)[0]

        # I/O
        fb.digital_inputs = struct.unpack_from(
            '<Q', raw, OFFSET_DIGITAL_INPUTS)[0]
        fb.digital_outputs = struct.unpack_from(
            '<Q', raw, OFFSET_DIGITAL_OUTPUTS)[0]

        # Joint arrays -- 6 doubles each
        fb.joint_positions_target = FeedbackParser._unpack_doubles(
            raw, OFFSET_Q_TARGET, 6
        )
        fb.joint_positions = FeedbackParser._unpack_doubles(
            raw, OFFSET_Q_ACTUAL, 6
        )
        fb.joint_speeds = FeedbackParser._unpack_doubles(
            raw, OFFSET_QD_ACTUAL, 6
        )
        fb.joint_currents = FeedbackParser._unpack_doubles(
            raw, OFFSET_I_ACTUAL, 6
        )
        fb.joint_torques = FeedbackParser._unpack_doubles(
            raw, OFFSET_M_ACTUAL, 6
        )

        # Cartesian state
        fb.tcp_pose = FeedbackParser._unpack_doubles(
            raw, OFFSET_TOOL_VECTOR_ACTUAL, 6
        )
        fb.tcp_speed = FeedbackParser._unpack_doubles(
            raw, OFFSET_TCP_SPEED_ACTUAL, 6
        )

        # Status bytes
        fb.enabled = bool(
            struct.unpack_from('<B', raw, OFFSET_ENABLE_STATUS)[0]
        )
        fb.error_status = struct.unpack_from('<B', raw, OFFSET_ERROR_STATUS)[0]
        fb.collision_state = struct.unpack_from(
            '<B', raw, OFFSET_COLLISION_STATE
        )[0]

        return fb

    @staticmethod
    def _unpack_doubles(raw: bytes, offset: int, count: int) -> List[float]:
        """
        Unpack an array of little-endian doubles from raw bytes.

        Parameters
        ----------
        raw : bytes
            Full packet bytes.
        offset : int
            Starting byte offset of the array.
        count : int
            Number of doubles to unpack.

        Returns
        -------
        List[float]
            List of unpacked float values.

        """
        return list(
            struct.unpack_from(f'<{count}d', raw, offset)
        )
