"""
ControlService -- ROS2 service handlers for CR5 robot control.

Registers and handles all control-related ROS2 services that
map directly to Dobot dashboard commands on port 29999.

Services registered:
    clear_error       -- Clear robot error state
    disable_robot     -- Disable the robot arm
    enable_robot      -- Enable the robot arm with optional load
    request_control   -- Switch robot to TCP control mode
    stop              -- Stop current execution
    pause             -- Pause current execution
    continue          -- Continue paused execution
    e_stop/activate   -- Activate emergency stop
    e_stop/release    -- Release emergency stop
    drag/enable       -- Enter drag mode
    drag/disable      -- Exit drag mode
"""

from functools import partial


from cr5_driver.robot.dashboard_model import DobotDashboardModel
from cr5_driver.tcp import DobotDashboardClient

from cr5_msgs.srv import EmptyRequest, EnableRobot
from cr5_msgs.srv._empty_request import (
    EmptyRequest_Request as EReq_req,
    EmptyRequest_Response as EReq_Resp,
)
from cr5_msgs.srv._enable_robot import (
    EnableRobot_Request as ER_req,
    EnableRobot_Response as ER_resp,
)

from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.impl.rcutils_logger import RcutilsLogger
from rclpy.node import Node


class ControlService:
    """
    ROS2 service handlers for CR5 robot control commands.

    All services delegate to DobotDashboardClient which manages
    the TCP connection to port 29999. Uses ReentrantCallbackGroup
    to allow concurrent service execution.
    """

    def __init__(self, node: Node) -> None:
        """
        Initialise and register all control services on the node.

        Parameters
        ----------
        node : Node
            ROS2 node to register services on.

        """
        self._log: RcutilsLogger = node.get_logger()
        self._log.info('Starting control services...')

        cb = ReentrantCallbackGroup()

        node.create_service(
            EmptyRequest, 'clear_error',
            self._clear_error, callback_group=cb
        )
        node.create_service(
            EmptyRequest, 'disable_robot',
            self._disable_robot, callback_group=cb
        )
        node.create_service(
            EnableRobot, 'enable_robot',
            self._enable_robot, callback_group=cb
        )
        node.create_service(
            EmptyRequest, 'request_control',
            self._request_control, callback_group=cb
        )
        node.create_service(
            EmptyRequest, 'stop',
            self._stop, callback_group=cb
        )
        node.create_service(
            EmptyRequest, 'pause',
            self._pause, callback_group=cb
        )
        node.create_service(
            EmptyRequest, 'continue',
            self._continue, callback_group=cb
        )
        node.create_service(
            EmptyRequest, 'e_stop/activate',
            partial(self._estop, en=True), callback_group=cb
        )
        node.create_service(
            EmptyRequest, 'e_stop/release',
            partial(self._estop, en=False), callback_group=cb
        )
        node.create_service(
            EmptyRequest, 'drag/enable',
            partial(self._drag, en=True), callback_group=cb
        )
        node.create_service(
            EmptyRequest, 'drag/disable',
            partial(self._drag, en=False), callback_group=cb
        )

        self._dashboard: DobotDashboardClient = DobotDashboardClient()
        self._log.info('Control services started.')

    # ── Generic helper ────────────────────────────────────────────

    def _execute(
            self,
            response: EReq_Resp,
            cmd: str,
            success_msg: str,
            fail_msg: str
    ) -> EReq_Resp:
        """
        Execute a dashboard command and populate the response.

        Handles socket exceptions and maps success/failure to
        the response fields. All simple service handlers delegate
        to this method to avoid repetition.

        Parameters
        ----------
        response : EReq_Resp
            ROS2 service response to populate.
        cmd : str
            Dashboard command string to send.
        success_msg : str
            Message to set on success.
        fail_msg : str
            Prefix message to set on failure.

        Returns
        -------
        EReq_Resp
            Populated response object.

        """
        try:
            resp: DobotDashboardModel = self._dashboard.send_command(
                cmd=cmd
            )
            response.success = resp.is_success
            response.message = success_msg if resp.is_success else (
                f'{fail_msg} Error {resp.error_id}: {resp.raw}'
            )
        except Exception as e:
            response.success = False
            response.message = f'Socket exception: {e}'
        return response

    # ── Service handlers ──────────────────────────────────────────

    def _clear_error(
            self, _: EReq_req, response: EReq_Resp
    ) -> EReq_Resp:
        """Handle clear_error service -- clear robot error state."""
        return self._execute(
            response,
            cmd='ClearError()',
            success_msg='Error cleared successfully.',
            fail_msg='Failed to clear error.'
        )

    def _disable_robot(
            self, _: EReq_req,
            response: EReq_Resp) -> EReq_Resp:
        """Handle disable_robot service -- disable the robot arm."""
        return self._execute(
            response,
            cmd='DisableRobot()',
            success_msg='Robot disabled.',
            fail_msg='Failed to disable robot.'
        )

    def _enable_robot(
            self, request: ER_req,
            response: ER_resp) -> ER_resp:
        """
        Handle enable_robot service -- enable arm with optional load.

        Parameters
        ----------
        request : ER_req
            Request containing load, x/y/z offsets and verify flag.
        response : ER_resp
            Service response to populate.

        Returns
        -------
        ER_resp
            Populated response object.

        """
        cmd: str = (
            f'EnableRobot({request.load},'
            f'{request.x_offset},{request.y_offset},{request.z_offset}'
            f'{",1" if request.verify else ""})'
        )
        try:
            resp: DobotDashboardModel = self._dashboard.send_command(
                cmd=cmd
            )
            response.success = resp.is_success
            response.message = 'Robot enabled.' if resp.is_success else (
                f'Failed to enable robot. Error {resp.error_id}: {resp.raw}'
            )
        except Exception as e:
            response.success = False
            response.message = f'Socket exception: {e}'
        return response

    def _request_control(
            self, _: EReq_req,
            response: EReq_Resp) -> EReq_Resp:
        """Handle request_control service -- switch to TCP mode."""
        return self._execute(
            response,
            cmd='RequestControl()',
            success_msg='Robot switched to TCP mode.',
            fail_msg='Failed to switch to TCP mode.'
        )

    def _stop(
            self, _: EReq_req,
            response: EReq_Resp) -> EReq_Resp:
        """Handle stop service -- stop current execution."""
        return self._execute(
            response,
            cmd='Stop()',
            success_msg='Execution stopped.',
            fail_msg='Failed to stop execution.'
        )

    def _pause(
            self, _: EReq_req,
            response: EReq_Resp) -> EReq_Resp:
        """Handle pause service -- pause current execution."""
        return self._execute(
            response,
            cmd='Pause()',
            success_msg='Execution paused.',
            fail_msg='Failed to pause execution.'
        )

    def _continue(
            self, _: EReq_req,
            response: EReq_Resp) -> EReq_Resp:
        """Handle continue service -- resume paused execution."""
        return self._execute(
            response,
            cmd='Continue()',
            success_msg='Execution resumed.',
            fail_msg='Failed to resume execution.'
        )

    def _estop(
            self, _: EReq_req,
            response: EReq_Resp, en: bool) -> EReq_Resp:
        """
        Handle e_stop services -- activate or release emergency stop.

        Parameters
        ----------
        _ : EReq_req
            Unused request.
        response : EReq_Resp
            Service response to populate.
        en : bool
            True to activate, False to release.

        Returns
        -------
        EReq_Resp
            Populated response object.

        """
        action: str = 'Activating' if en else 'Releasing'
        fail_action: str = 'activate' if en else 'release'
        return self._execute(
            response,
            cmd=f'EmergencyStop({int(en)})',
            success_msg=f'{action} emergency stop.',
            fail_msg=f'Failed to {fail_action} emergency stop.'
        )

    def _drag(
            self, _: EReq_req,
            response: EReq_Resp, en: bool) -> EReq_Resp:
        """
        Handle drag services -- enter or exit drag mode.

        Parameters
        ----------
        _ : EReq_req
            Unused request.
        response : EReq_Resp
            Service response to populate.
        en : bool
            True to enter drag mode, False to exit.

        Returns
        -------
        EReq_Resp
            Populated response object.

        """
        action: str = 'Entering' if en else 'Exiting'
        fail_action: str = 'enter' if en else 'exit'
        return self._execute(
            response,
            cmd='StartDrag()' if en else 'StopDrag()',
            success_msg=f'{action} drag mode.',
            fail_msg=f'Failed to {fail_action} drag mode.'
        )
