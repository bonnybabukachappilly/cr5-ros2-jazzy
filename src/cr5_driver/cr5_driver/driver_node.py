"""
DriverNode -- ROS2 node for Dobot CR5 TCP/IP driver.

Connects to the CR5 over TCP, publishes robot state as ROS2
topics, and exposes robot control as ROS2 services.

Topics published:
    /cr5/joint_states  (sensor_msgs/JointState)
    /cr5/robot_state   (cr5_msgs/RobotState)

Services:
    /cr5/enable        (cr5_msgs/EnableRobot)
    /cr5/disable       (cr5_msgs/DisableRobot)
    /cr5/clear_error   (cr5_msgs/ClearError)
    /cr5/set_speed     (cr5_msgs/SetSpeed)
    /cr5/move_j        (cr5_msgs/MoveJ)
    /cr5/robot_mode    (cr5_msgs/RobotMode)

Parameters:
    robot_ip    (string, default: '192.168.5.1')
    mock_mode   (bool,   default: false)
"""

from _thread import LockType
import math
import threading

import rclpy.executors
from rclpy.callback_groups import ReentrantCallbackGroup

import rclpy
from rclpy.node import Node
from rclpy.publisher import Publisher
from rclpy.qos import QoSProfile, ReliabilityPolicy

from sensor_msgs.msg import JointState
from std_msgs.msg import Header

from cr5_msgs.msg import RobotState
from cr5_msgs.srv import (
    ClearError,
    DisableRobot,
    EnableRobot,
    MoveJ,
    RobotMode,
    SetSpeed,
)

from cr5_driver.tcp.dashboard_client import DashboardClient, DashboardError
from cr5_driver.tcp.feedback_client import FeedbackClient
from cr5_driver.tcp.feedback_parser import FeedbackParser, RobotFeedback

JOINT_NAMES: list[str] = ['J1', 'J2', 'J3', 'J4', 'J5', 'J6']


class DriverNode(Node):
    """
    ROS2 node that interfaces with the Dobot CR5 over TCP/IP.

    In normal mode connects to the real robot.
    In mock mode publishes simulated data for development
    without hardware.
    """

    def __init__(self) -> None:
        """Initialise the driver node and all ROS2 interfaces."""
        super().__init__('cr5_driver')

        # ── Parameters ───────────────────────────────────────────
        self.robot_ip = self.declare_parameter(
            'robot_ip', '192.168.5.1'
        ).value
        self.mock_mode = self.declare_parameter(
            'mock_mode', False
        ).value

        if self.mock_mode:
            self.get_logger().warn(
                'Running in MOCK MODE -- no real robot connected'
            )

        self._cb_group = ReentrantCallbackGroup()

        # ── QoS profile for high-frequency topics ────────────────
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            depth=10
        )

        # ── Publishers ───────────────────────────────────────────
        self.joint_state_pub: Publisher = self.create_publisher(
            JointState,
            '/cr5/joint_states',
            sensor_qos,
            callback_group=self._cb_group
        )
        self.robot_state_pub: Publisher = self.create_publisher(
            RobotState,
            '/cr5/robot_state',
            sensor_qos,
            callback_group=self._cb_group
        )

        # ── Services ─────────────────────────────────────────────
        self.create_service(
            EnableRobot, '/cr5/enable', self._handle_enable,
            callback_group=self._cb_group
        )
        self.create_service(
            DisableRobot, '/cr5/disable', self._handle_disable,
            callback_group=self._cb_group
        )
        self.create_service(
            ClearError, '/cr5/clear_error', self._handle_clear_error,
            callback_group=self._cb_group
        )
        self.create_service(
            SetSpeed, '/cr5/set_speed', self._handle_set_speed,
            callback_group=self._cb_group
        )
        self.create_service(
            MoveJ, '/cr5/move_j', self._handle_move_j,
            callback_group=self._cb_group
        )
        self.create_service(
            RobotMode, '/cr5/robot_mode', self._handle_robot_mode,
            callback_group=self._cb_group
        )

        # ── Internal state ───────────────────────────────────────
        self._latest_feedback = None
        self._feedback_lock: LockType = threading.Lock()

        # ── TCP clients ──────────────────────────────────────────
        if not self.mock_mode:
            self.dashboard = DashboardClient(self.robot_ip)
            self.feedback = FeedbackClient(
                self.robot_ip, self._on_feedback_packet
            )
            self._connect()
        else:
            self.dashboard = None
            self.feedback = None
            self._start_mock_feedback()

        self.get_logger().info('CR5 driver node ready')

    def _connect(self) -> None:
        """Connect to robot dashboard and start feedback stream."""
        self.get_logger().info(f'Connecting to CR5 at {self.robot_ip}')
        self.dashboard.connect()
        self.feedback.start()
        self.get_logger().info('Connected to CR5')

    def destroy_node(self) -> None:
        """Clean up connections on node shutdown."""
        if self.feedback:
            self.feedback.stop()
        if self.dashboard:
            self.dashboard.disconnect()
        super().destroy_node()

    # ── Feedback callback ─────────────────────────────────────────

    def _on_feedback_packet(self, raw: bytes) -> None:
        """
        Parse incoming feedback packet and publish ROS2 topics.

        Called from feedback thread at ~125Hz.

        Parameters
        ----------
        raw : bytes
            Raw 1440-byte feedback packet.

        """
        try:
            fb = FeedbackParser.parse(raw)
            with self._feedback_lock:
                self._latest_feedback = fb
            self._publish_joint_states(fb)
            self._publish_robot_state(fb)
        except ValueError as e:
            self.get_logger().warn(f'Feedback parse error: {e}')

    def _publish_joint_states(self, fb: RobotFeedback) -> None:
        """
        Publish joint positions as sensor_msgs/JointState.

        Parameters
        ----------
        fb : RobotFeedback
            Parsed feedback data.

        """
        msg = JointState()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.name = JOINT_NAMES
        msg.position = [
            math.radians(p) for p in fb.joint_positions
        ]
        msg.velocity = [
            math.radians(v) for v in fb.joint_speeds
        ]
        msg.effort = list(fb.joint_torques)
        self.joint_state_pub.publish(msg)

    def _publish_robot_state(self, fb: RobotFeedback) -> None:
        """
        Publish full robot state as cr5_msgs/RobotState.

        Parameters
        ----------
        fb : RobotFeedback
            Parsed feedback data.

        """
        msg = RobotState()
        msg.joint_positions = fb.joint_positions
        msg.joint_speeds = fb.joint_speeds
        msg.joint_torques = fb.joint_torques
        msg.tcp_pose = fb.tcp_pose
        msg.enabled = fb.enabled
        msg.mode = int(fb.robot_mode)
        msg.error_code = str(fb.error_status)
        msg.last_update = float(fb.timestamp_ms) / 1000.0
        self.robot_state_pub.publish(msg)

    # ── Mock mode ─────────────────────────────────────────────────

    def _start_mock_feedback(self) -> None:
        """Start a timer that publishes simulated robot state."""
        self._mock_angle = 0.0
        self.create_timer(0.008, self._mock_feedback_callback)

    def _mock_feedback_callback(self) -> None:
        """Publish simulated sinusoidal joint motion for testing."""
        self._mock_angle += 0.01
        fb = RobotFeedback()
        fb.robot_mode = 5
        fb.enabled = True
        fb.speed_scaling = 50.0
        fb.joint_positions = [
            math.degrees(math.sin(self._mock_angle + i * 0.5))
            for i in range(6)
        ]
        fb.tcp_pose = [0.0, -245.0, 1047.0, -90.0, 0.0, -179.0]
        self._publish_joint_states(fb)
        self._publish_robot_state(fb)

    # ── Service handlers ──────────────────────────────────────────

    def _handle_enable(
            self,
            request: EnableRobot.Request,
            response: EnableRobot.Response
    ) -> EnableRobot.Response:
        """
        Handle /cr5/enable service request.

        Parameters
        ----------
        request : EnableRobot.Request
            Service request (no fields).
        response : EnableRobot.Response
            Service response to populate.

        Returns
        -------
        EnableRobot.Response
            Response with success flag and message.

        """
        if self.mock_mode:
            response.success = True
            response.message = 'Mock mode: robot enabled'
            return response
        try:
            self.dashboard.enable_robot()
            response.success = True
            response.message = 'Robot enabled'
        except DashboardError as e:
            response.success = False
            response.message = str(e)
        return response

    def _handle_disable(
            self,
            request: DisableRobot.Request,
            response: DisableRobot.Response
    ) -> DisableRobot.Response:
        """
        Handle /cr5/disable service request.

        Parameters
        ----------
        request : DisableRobot.Request
            Service request (no fields).
        response : DisableRobot.Response
            Service response to populate.

        Returns
        -------
        DisableRobot.Response
            Response with success flag and message.

        """
        if self.mock_mode:
            response.success = True
            response.message = 'Mock mode: robot disabled'
            return response
        try:
            self.dashboard.disable_robot()
            response.success = True
            response.message = 'Robot disabled'
        except DashboardError as e:
            response.success = False
            response.message = str(e)
        return response

    def _handle_clear_error(
            self,
            request: ClearError.Request,
            response: ClearError.Response
    ) -> ClearError.Response:
        """
        Handle /cr5/clear_error service request.

        Parameters
        ----------
        request : ClearError.Request
            Service request (no fields).
        response : ClearError.Response
            Service response to populate.

        Returns
        -------
        ClearError.Response
            Response with success flag and message.

        """
        if self.mock_mode:
            response.success = True
            response.message = 'Mock mode: errors cleared'
            return response
        try:
            self.dashboard.clear_error()
            response.success = True
            response.message = 'Errors cleared'
        except DashboardError as e:
            response.success = False
            response.message = str(e)
        return response

    def _handle_set_speed(
            self,
            request: SetSpeed.Request,
            response: SetSpeed.Response
    ) -> SetSpeed.Response:
        """
        Handle /cr5/set_speed service request.

        Parameters
        ----------
        request : SetSpeed.Request
            Request containing speed_percent field.
        response : SetSpeed.Response
            Service response to populate.

        Returns
        -------
        SetSpeed.Response
            Response with success flag and message.

        """

        if self.mock_mode:
            response.success = True
            response.message = f'Mock mode: speed set to {request.speed_percent}%'
            return response
        try:
            _speed = int(request.speed_percent)
            self.get_logger().info(f'Requesting for speed ratio of {_speed}%')
            self.dashboard.set_speed(_speed)
            response.success = True
            response.message = f'Speed set to {_speed}%'
        except DashboardError as e:
            response.success = False
            response.message = str(e)
        return response

    def _handle_move_j(
            self,
            request: MoveJ.Request,
            response: MoveJ.Response
    ) -> MoveJ.Response:
        """
        Handle /cr5/move_j service request.

        Parameters
        ----------
        request : MoveJ.Request
            Request containing target pose and speed.
        response : MoveJ.Response
            Service response to populate.

        Returns
        -------
        MoveJ.Response
            Response with success flag and message.

        """
        if self.mock_mode:
            response.success = True
            response.message = 'Mock mode: move_j accepted'
            return response
        try:
            t = request.target
            self.dashboard.move_j(
                t.x, t.y, t.z, t.rx, t.ry, t.rz,
                request.speed
            )
            response.success = True
            response.message = 'MovJ command sent'
        except DashboardError as e:
            response.success = False
            response.message = str(e)
        return response

    def _handle_robot_mode(
            self,
            request: RobotMode.Request,
            response: RobotMode.Response
    ) -> RobotMode.Response:
        """
        Handle /cr5/robot_mode service request.

        Parameters
        ----------
        request : RobotMode.Request
            Service request (no fields).
        response : RobotMode.Response
            Service response to populate.

        Returns
        -------
        RobotMode.Response
            Response with mode and description.

        """
        mode_descriptions = {
            1: 'Initialising',
            2: 'Brake released',
            3: 'Disabled',
            4: 'Enabled',
            5: 'Enabled and idle',
            6: 'Running',
            7: 'Recording',
            8: 'Error',
            9: 'Paused',
            10: 'Jogging',
        }
        if self.mock_mode:
            response.mode = 5
            response.description = 'Mock mode: ' + \
                mode_descriptions.get(5, 'Unknown')
            return response
        try:
            mode = self.dashboard.get_robot_mode()
            response.mode = mode
            response.description = mode_descriptions.get(mode, 'Unknown')
        except DashboardError as e:
            response.mode = -1
            response.description = str(e)
        return response


def main(args=None) -> None:
    """
    Entry point for the cr5_driver node.

    Parameters
    ----------
    args : list, optional
        Command line arguments passed to rclpy.

    """
    rclpy.init(args=args)
    node = DriverNode()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
