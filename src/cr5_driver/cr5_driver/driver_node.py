"""
DriverNode -- ROS2 node for Dobot CR5 TCP/IP driver.

Single responsibility: orchestrate all driver components.
Wires together publishers, services, feedback and mock mode.
Does not contain any robot logic itself.

Parameters:
    robot_ip  (string, default: '192.168.5.1')
    mock_mode (bool,   default: false)
"""


from threading import Thread

from cr5_driver.publishers.joint_state_publisher import JointStatePublisher
from cr5_driver.publishers.robot_state_publisher import RobotStatePublisher
from cr5_driver.services import ControlService, SettingsService
from cr5_driver.tcp import (
    DobotDashboardClient,
    DobotFeedbackClient,
    get_feedback_client
)


import rclpy
import rclpy.executors
from rclpy.node import Node


class DriverNode(Node):
    """
    Thin orchestrator node for the CR5 driver.

    Creates and wires together all driver components:
    - DashboardClient for robot control commands
    - FeedbackClient for real-time robot state
    - JointStatePublisher for /cr5/joint_states
    - RobotStatePublisher for /cr5/robot_state
    - RobotServices for all ROS2 service handlers
    - MockFeedback for development without hardware
    """

    def __init__(self) -> None:
        """Initialise the driver node and all components."""
        super().__init__('cr5_driver', namespace='cr5')

        # 1. Declare the parameter with a default value
        self.declare_parameter('robot_ip', '192.168.5.1')

        # 2. Get the parameter value

        _robot_ip: str = self.get_parameter(
            'robot_ip').get_parameter_value().string_value

        # 3. Publishers
        self._joint_pub = JointStatePublisher(self)
        self._state_pub = RobotStatePublisher(self)

        # 4. TCP Clients
        # Feedback Clients
        self._feedback: DobotFeedbackClient = get_feedback_client(
            host=_robot_ip, log=self.get_logger())

        self._feedback.register_callback(callbacks=self._joint_pub.publish)
        self._feedback.register_callback(callbacks=self._state_pub.publish)

        self._feedback_th = Thread(
            target=self._feedback.start, name='feedback_socket', daemon=True)
        self._feedback_th.start()

        # Dashboard Clients
        self._dashboard: DobotDashboardClient = DobotDashboardClient(
            host=_robot_ip, log=self.get_logger()
        )
        self._dashboard_th = Thread(
            target=self._dashboard.start, name='dashboard_client', daemon=True)
        self._dashboard_th.start()

        # 5. Services
        ControlService(self)
        SettingsService(self)

        # ── Dashboard + Services ─────────────────────────────────

        # ── Feedback or Mock ─────────────────────────────────────

        self.get_logger().info('CR5 driver node ready')

    def destroy_node(self) -> None:
        """Clean up all connections on shutdown."""
        if self._feedback:
            self._feedback.stop()
            self._feedback_th.join()

        if self._dashboard:
            self._dashboard.stop()
            self._dashboard_th.join()

        super().destroy_node()


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
