"""
RobotStatePublisher -- publishes CR5 full state as ROS2 topic.

Single responsibility: convert RobotFeedback data into
cr5_msgs/RobotState and publish to robot_state.
"""


from cr5_driver.robot.feedback_model import DobotFeedbackModel
from cr5_driver.robot.state import RobotMode

from cr5_msgs.msg import RobotState

from rclpy.clock import Clock
from rclpy.node import Node
from rclpy.publisher import Publisher
from rclpy.qos import QoSProfile, ReliabilityPolicy


class RobotStatePublisher:
    """
    Publishes full robot state to robot_state.

    Includes status, robot mode and error code.
    """

    TOPIC = 'robot_state'

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
            RobotState, self.TOPIC, qos)
        self._clock: Clock = node.get_clock()

    def publish(self, data: DobotFeedbackModel) -> None:
        """
        Publish robot state from feedback data.

        Parameters
        ----------
        data : DobotFeedbackModel
            Parsed robot feedback containing full robot state.

        """
        msg = RobotState()
        msg.enabled = data.is_enabled
        msg.mode = RobotMode(data.robot_mode).name
        msg.error_code = data.error_code
        msg.last_update = float(data.timestamp) / 1000.0
        self._pub.publish(msg)
