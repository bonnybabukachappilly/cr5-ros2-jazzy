"""
Unit tests for FeedbackParser.

Tests packet parsing, integrity validation, and sync recovery
without requiring a physical robot connection.
"""

import struct
import unittest

from cr5_driver.tcp.feedback_parser import (
    EXPECTED_TEST_VALUE,
    FeedbackParser,
    OFFSET_ENABLE_STATUS,
    OFFSET_ERROR_STATUS,
    OFFSET_M_ACTUAL,
    OFFSET_Q_ACTUAL,
    OFFSET_ROBOT_MODE,
    OFFSET_SPEED_SCALING,
    OFFSET_TEST_VALUE,
    OFFSET_TOOL_VECTOR_ACTUAL,
    PACKET_SIZE,
    RobotFeedback,
)


def build_test_packet(
        joint_positions=None,
        tcp_pose=None,
        robot_mode=5,
        enabled=True,
        error_status=0,
        speed_scaling=50.0,
        torques=None,
        corrupt_test_value=False) -> bytes:
    """
    Build a valid 1440-byte test packet with known values.

    Parameters
    ----------
    joint_positions : list, optional
        Six joint positions in degrees. Defaults to zeros.
    tcp_pose : list, optional
        Six TCP pose values. Defaults to zeros.
    robot_mode : int, optional
        Robot mode value. Defaults to 5 (enabled).
    enabled : bool, optional
        Enable status. Defaults to True.
    error_status : int, optional
        Error status byte. Defaults to 0.
    speed_scaling : float, optional
        Speed scaling percentage. Defaults to 50.0.
    torques : list, optional
        Six joint torques. Defaults to zeros.
    corrupt_test_value : bool, optional
        If True, writes wrong test value to trigger integrity fail.

    Returns
    -------
    bytes
        A 1440-byte packet with the specified values.

    """
    buf = bytearray(PACKET_SIZE)

    # Write MessageSize header
    struct.pack_into('<H', buf, 0, PACKET_SIZE)

    # Write test value (integrity check)
    test_val = 0xDEADBEEF if corrupt_test_value else EXPECTED_TEST_VALUE
    struct.pack_into('<Q', buf, OFFSET_TEST_VALUE, test_val)

    # Write robot mode
    struct.pack_into('<Q', buf, OFFSET_ROBOT_MODE, robot_mode)

    # Write speed scaling
    struct.pack_into('<d', buf, OFFSET_SPEED_SCALING, speed_scaling)

    # Write joint positions
    positions: list[float] = joint_positions or [0.0] * 6
    for i, pos in enumerate(positions):
        struct.pack_into('<d', buf, OFFSET_Q_ACTUAL + i * 8, pos)

    # Write TCP pose
    pose: list[float] = tcp_pose or [0.0] * 6
    for i, val in enumerate(pose):
        struct.pack_into('<d', buf, OFFSET_TOOL_VECTOR_ACTUAL + i * 8, val)

    # Write torques
    torque_vals: list[float] = torques or [0.0] * 6
    for i, val in enumerate(torque_vals):
        struct.pack_into('<d', buf, OFFSET_M_ACTUAL + i * 8, val)

    # Write status bytes
    struct.pack_into('<B', buf, OFFSET_ENABLE_STATUS, int(enabled))
    struct.pack_into('<B', buf, OFFSET_ERROR_STATUS, error_status)

    return bytes(buf)


class TestFeedbackParserParsing(unittest.TestCase):
    """Tests for correct parsing of known packet values."""

    def test_parse_joint_positions(self) -> None:
        """Parser extracts correct joint positions from packet."""
        expected: list[float] = [10.0, 20.0, 30.0, 40.0, 50.0, 60.0]
        raw: bytes = build_test_packet(joint_positions=expected)
        result: RobotFeedback = FeedbackParser.parse(raw)
        for i, (got, exp) in enumerate(
                zip(result.joint_positions, expected)):
            self.assertAlmostEqual(
                got, exp, places=4,
                msg=f'Joint {i+1} position mismatch'
            )

    def test_parse_tcp_pose(self) -> None:
        """Parser extracts correct TCP Cartesian pose from packet."""
        expected: list[float] = [100.0, -200.0, 500.0, -90.0, 0.0, 45.0]
        raw: bytes = build_test_packet(tcp_pose=expected)
        result: RobotFeedback = FeedbackParser.parse(raw)
        for i, (got, exp) in enumerate(zip(result.tcp_pose, expected)):
            self.assertAlmostEqual(
                got, exp, places=4,
                msg=f'TCP pose field {i} mismatch'
            )

    def test_parse_robot_mode(self) -> None:
        """Parser extracts robot mode correctly."""
        raw: bytes = build_test_packet(robot_mode=5)
        result: RobotFeedback = FeedbackParser.parse(raw)
        self.assertEqual(result.robot_mode, 5)

    def test_parse_enabled_true(self) -> None:
        """Parser reports enabled=True when enable status byte is 1."""
        raw: bytes = build_test_packet(enabled=True)
        result: RobotFeedback = FeedbackParser.parse(raw)
        self.assertTrue(result.enabled)

    def test_parse_enabled_false(self) -> None:
        """Parser reports enabled=False when enable status byte is 0."""
        raw: bytes = build_test_packet(enabled=False)
        result: RobotFeedback = FeedbackParser.parse(raw)
        self.assertFalse(result.enabled)

    def test_parse_error_status(self) -> None:
        """Parser extracts error status byte correctly."""
        raw: bytes = build_test_packet(error_status=8)
        result: RobotFeedback = FeedbackParser.parse(raw)
        self.assertEqual(result.error_status, 8)

    def test_parse_speed_scaling(self) -> None:
        """Parser extracts speed scaling correctly."""
        raw: bytes = build_test_packet(speed_scaling=75.0)
        result: RobotFeedback = FeedbackParser.parse(raw)
        self.assertAlmostEqual(result.speed_scaling, 75.0, places=4)

    def test_parse_joint_torques(self) -> None:
        """Parser extracts joint torques correctly."""
        expected: list[float] = [1.1, 2.2, 3.3, 4.4, 5.5, 6.6]
        raw: bytes = build_test_packet(torques=expected)
        result: RobotFeedback = FeedbackParser.parse(raw)
        for i, (got, exp) in enumerate(
                zip(result.joint_torques, expected)):
            self.assertAlmostEqual(
                got, exp, places=4,
                msg=f'Joint {i+1} torque mismatch'
            )


class TestFeedbackParserValidation(unittest.TestCase):
    """Tests for packet validation and error handling."""

    def test_wrong_packet_size_raises(self) -> None:
        """Parser raises ValueError for packets not 1440 bytes."""
        raw: bytes = build_test_packet()
        with self.assertRaises(ValueError) as ctx:
            FeedbackParser.parse(raw[:800])
        self.assertIn('Expected 1440', str(ctx.exception))

    def test_corrupt_test_value_raises(self) -> None:
        """Parser raises ValueError when integrity check fails."""
        raw: bytes = build_test_packet(corrupt_test_value=True)
        with self.assertRaises(ValueError) as ctx:
            FeedbackParser.parse(raw)
        self.assertIn('integrity check failed', str(ctx.exception))

    def test_empty_bytes_raises(self) -> None:
        """Parser raises ValueError for empty input."""
        with self.assertRaises(ValueError):
            FeedbackParser.parse(b'')

    def test_valid_packet_does_not_raise(self) -> None:
        """Parser accepts a valid packet without raising."""
        raw: bytes = build_test_packet()
        try:
            FeedbackParser.parse(raw)
        except ValueError:
            self.fail('parse() raised ValueError on valid packet')


class TestFeedbackParserEdgeCases(unittest.TestCase):
    """Tests for edge cases and boundary values."""

    def test_all_zero_joint_positions(self) -> None:
        """Parser handles all-zero joint positions correctly."""
        raw: bytes = build_test_packet(joint_positions=[0.0] * 6)
        result: RobotFeedback = FeedbackParser.parse(raw)
        for pos in result.joint_positions:
            self.assertAlmostEqual(pos, 0.0, places=4)

    def test_negative_joint_positions(self) -> None:
        """Parser handles negative joint positions correctly."""
        expected: list[float] = [-180.0, -90.0, -45.0, -30.0, -10.0, -5.0]
        raw: bytes = build_test_packet(joint_positions=expected)
        result: RobotFeedback = FeedbackParser.parse(raw)
        for i, (got, exp) in enumerate(
                zip(result.joint_positions, expected)):
            self.assertAlmostEqual(got, exp, places=4)

    def test_joint_positions_length(self) -> None:
        """Parser always returns exactly 6 joint positions."""
        raw: bytes = build_test_packet()
        result: RobotFeedback = FeedbackParser.parse(raw)
        self.assertEqual(len(result.joint_positions), 6)

    def test_tcp_pose_length(self) -> None:
        """Parser always returns exactly 6 TCP pose values."""
        raw: bytes = build_test_packet()
        result: RobotFeedback = FeedbackParser.parse(raw)
        self.assertEqual(len(result.tcp_pose), 6)


if __name__ == '__main__':
    unittest.main()
