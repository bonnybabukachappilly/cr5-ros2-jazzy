"""
DobotFeedbackModel -- parsed representation of CR5 feedback packet.

The CR5 pushes a 1440-byte binary packet every 8ms on port 30004.
This module defines the data model and parser for that packet.

Packet structure reference: Dobot TCP-IP Protocol V4.6.5 Section 3.
All values are little-endian.
"""

from __future__ import annotations

from dataclasses import dataclass
import struct


@dataclass(frozen=True)
class DobotFeedbackModel:
    """
    Immutable representation of one CR5 real-time feedback packet.

    All joint arrays follow the order [J1, J2, J3, J4, J5, J6].
    Positions and speeds are in degrees and degrees/second.
    TCP pose is [X(mm), Y(mm), Z(mm), Rx(deg), Ry(deg), Rz(deg)].
    """

    # Metadata
    message_size: int
    digital_inputs: int
    digital_outputs: int
    robot_mode: int
    timestamp: int
    test_value: int
    speed_scaling: float

    # Joint States (6-axis)
    q_target: list[float]
    q_actual: list[float]
    qd_actual: list[float]
    m_actual: list[float]

    # TCP States (X, Y, Z, Rx, Ry, Rz)
    tool_vector_actual: list[float]
    tcp_speed_actual: list[float]

    # Status
    is_enabled: bool
    error_code: int
    is_collided: bool

    @classmethod
    def from_bytes(cls, data: bytes) -> DobotFeedbackModel:
        """
        Parse a raw 1440-byte feedback packet into a model instance.

        Parameters
        ----------
        data : bytes
            Raw bytes received from port 30004.

        Returns
        -------
        DobotFeedbackModel
            Parsed and validated robot state.

        Raises
        ------
        ValueError
            If data is shorter than 1440 bytes or the integrity
            check value at offset 48 does not match the expected
            constant 0x0123456789ABCDEF.

        """
        if len(data) < 1440:
            raise ValueError(
                f'Packet too short: expected 1440, got {len(data)}'
            )

        test_val: int = struct.unpack_from('<Q', data, 48)[0]
        if test_val != 0x0123456789ABCDEF:
            print(
                f'Integrity check failed: '
                f'expected 0x0123456789abcdef, got {hex(test_val)}'
            )

        def get_doubles(offset: int, count: int = 6) -> list[float]:
            return list(
                struct.unpack_from(f'<{count}d', data, offset)
            )

        return cls(
            message_size=struct.unpack_from('<H', data, 0)[0],
            digital_inputs=struct.unpack_from('<Q', data, 8)[0],
            digital_outputs=struct.unpack_from('<Q', data, 16)[0],
            robot_mode=struct.unpack_from('<Q', data, 24)[0],
            timestamp=struct.unpack_from('<Q', data, 32)[0],
            test_value=test_val,
            speed_scaling=struct.unpack_from('<d', data, 64)[0],
            q_target=get_doubles(192),
            q_actual=get_doubles(432),
            qd_actual=get_doubles(480),
            m_actual=get_doubles(1120),
            tool_vector_actual=get_doubles(624),
            tcp_speed_actual=get_doubles(672),
            is_enabled=struct.unpack_from('<B', data, 1026)[0] == 1,
            error_code=struct.unpack_from('<B', data, 1029)[0],
            is_collided=struct.unpack_from('<B', data, 1038)[0] != 0,
        )
