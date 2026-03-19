"""
JointStatePublisher -- publishes CR5 joint state as ROS2 topic.

Single responsibility: convert RobotFeedback data into
sensor_msgs/JointState and publish to joint_states.
"""

import math

from cr5_driver.robot.feedback_model import DobotFeedbackModel

from rclpy.clock import Clock
from rclpy.impl.rcutils_logger import RcutilsLogger
from rclpy.node import Node
from rclpy.publisher import Publisher
from rclpy.qos import QoSProfile, ReliabilityPolicy

from sensor_msgs.msg import JointState

from std_msgs.msg import Header

JOINT_NAMES: list[str] = ['J1', 'J2', 'J3', 'J4', 'J5', 'J6']


class JointStatePublisher:
    """
    Publishes robot joint state to joint_states.

    Converts joint positions from degrees to radians as
    required by the ROS2 sensor_msgs/JointState convention.
    """

    TOPIC = 'joint_states'

    def __init__(self, node: Node) -> None:
        """
        Initialise publisher on the given node.

        Parameters
        ----------
        node : Node
            ROS2 node to attach the publisher to.

        """
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            depth=10
        )

        self._pub: Publisher = node.create_publisher(
            JointState, self.TOPIC, qos)
        self._clock: Clock = node.get_clock()
        self._logger: RcutilsLogger = node.get_logger()

    def publish(self, data: DobotFeedbackModel) -> None:
        """
        Publish joint state from feedback data.

        Parameters
        ----------
        data : DobotFeedbackModel
            Parsed robot feedback containing full robot state.

        """
        msg = JointState()
        msg.header = Header()
        msg.header.stamp = self._clock.now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.name = JOINT_NAMES
        msg.position = [math.radians(p) for p in data.q_actual]
        msg.velocity = [math.radians(v) for v in data.qd_actual]
        msg.effort = list(data.m_actual)
        self._pub.publish(msg)
