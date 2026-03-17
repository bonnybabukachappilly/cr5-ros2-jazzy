
from functools import partial
from typing import Optional

from cr5_driver.robot.dashboard_model import DobotDashboardModel
from cr5_driver.tcp import DobotDashboardClient

from cr5_msgs.srv import (
    EnableRobot, EmptyRequest
)
from cr5_msgs.srv._empty_request import (
    EmptyRequest_Request as EReq_req, EmptyRequest_Response as EReq_Resp
)
from cr5_msgs.srv._enable_robot import (
    EnableRobot_Request as ER_req, EnableRobot_Response as ER_resp
)


from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.impl.rcutils_logger import RcutilsLogger
from rclpy.node import Node


class ControlService:
    def __init__(self, node: Node) -> None:
        self._log: RcutilsLogger = node.get_logger()

        self._log.info('Starting control services...')

        cb = MutuallyExclusiveCallbackGroup()

        # Clear Error
        node.create_service(
            srv_type=EmptyRequest,
            srv_name='clear_error',
            callback=self._clear_error,
            callback_group=cb
        )

        # Disable Robot
        node.create_service(
            srv_type=EmptyRequest,
            srv_name='disable_robot',
            callback=self.disable_robot,
            callback_group=cb
        )

        # Enable Robot
        node.create_service(
            srv_type=EnableRobot,
            srv_name='enable_robot',
            callback=self._enable_robot,
            callback_group=cb
        )

        # RequestControl
        node.create_service(
            srv_type=EmptyRequest,
            srv_name='request_control',
            callback=self._request_control,
            callback_group=cb
        )

        # Stop
        node.create_service(
            srv_type=EmptyRequest,
            srv_name='stop',
            callback=self._stop,
            callback_group=cb
        )

        # Pause
        node.create_service(
            srv_type=EmptyRequest,
            srv_name='pause',
            callback=self._pause,
            callback_group=cb
        )

        # Continue
        node.create_service(
            srv_type=EmptyRequest,
            srv_name='continue',
            callback=self._continue,
            callback_group=cb
        )

        # E-Stop Enable
        node.create_service(
            srv_type=EmptyRequest,
            srv_name='e_stop/activate',
            callback=partial(self._estop, en=True),
            callback_group=cb
        )

        # E-Stop Release
        node.create_service(
            srv_type=EmptyRequest,
            srv_name='e_stop/release',
            callback=partial(self._estop, en=False),
            callback_group=cb
        )

        # Drag Enable
        node.create_service(
            srv_type=EmptyRequest,
            srv_name='drag/enable',
            callback=partial(self._drag, en=True),
            callback_group=cb
        )

        # Drag Release
        node.create_service(
            srv_type=EmptyRequest,
            srv_name='drag/disable',
            callback=partial(self._drag, en=False),
            callback_group=cb
        )

        self._log.info('Control Services are started.')

        self._dashboard: DobotDashboardClient = DobotDashboardClient()

    def _clear_error(self, _: EReq_req, response: EReq_Resp) -> EReq_Resp:
        resp: Optional[DobotDashboardModel] = None

        try:
            resp = self._dashboard.send_command(
                cmd='ClearError()')

        except Exception as e:
            response.message = f'Socket exception occurred: {e}'
            response.success = False
            return response

        if resp.is_success:
            response.message = 'Successfully cleared error'
            response.success = True
        else:
            response.message = 'Failed to clear the error. ' + \
                f'Error Id: {resp.error_id}' + f'message: {resp.raw}'
            response.success = False

        return response

    def disable_robot(self, _: EReq_req, response: EReq_Resp) -> EReq_Resp:
        resp: Optional[DobotDashboardModel] = None

        try:
            resp = self._dashboard.send_command(
                cmd='DisableRobot()')

        except Exception as e:
            response.message = f'Socket exception occurred: {e}'
            response.success = False
            return response

        if resp.is_success:
            response.message = 'Robot Disabled'
            response.success = True
        else:
            response.message = 'Failed to disable the robot. ' + \
                f'Error Id: {resp.error_id}' + f'message: {resp.raw}'
            response.success = False

        return response

    def _enable_robot(self, request: ER_req, response: ER_resp) -> ER_resp:
        _x: float = request.x_offset
        _y: float = request.y_offset
        _z: float = request.z_offset

        _load: float = request.load
        _cmd: str = f'EnableRobot({_load},{_x},{_y},{_z})'

        if request.verify:
            _cmd = f'EnableRobot({_load},{_x},{_y},{_z},1)'

        resp: Optional[DobotDashboardModel] = None
        try:
            resp = self._dashboard.send_command(cmd=_cmd)

        except Exception as e:
            response.message = f'Socket exception occurred: {e}'
            response.success = False
            return response

        if resp.is_success:
            response.message = 'Robot Enabled'
            response.success = True
        else:
            response.message = (
                f'Failed to enable the robot. Error Id: {resp.error_id}'
                + f'message: {resp.raw}'
            )
            response.success = False

        return response

    def _request_control(self, _: EReq_req, response: EReq_Resp) -> EReq_Resp:

        resp: Optional[DobotDashboardModel] = None
        try:
            resp = self._dashboard.send_command(
                cmd='RequestControl()')

        except Exception as e:
            response.message = f'Socket exception occurred: {e}'
            response.success = False
            return response

        if resp.is_success:
            response.message = 'Robot switched to TCP'
            response.success = True
        else:
            response.message = 'Unable to switch robot to TCP. ' + \
                f'Error Id: {resp.error_id}' + f'message: {resp.raw}'
            response.success = False

        return response

    def _stop(self, _: EReq_req, response: EReq_Resp) -> EReq_Resp:
        resp: Optional[DobotDashboardModel] = None

        try:
            resp = self._dashboard.send_command(
                cmd='Stop()')

        except Exception as e:
            response.message = f'Socket exception occurred: {e}'
            response.success = False
            return response

        if resp.is_success:
            response.message = 'Stopped the execution'
            response.success = True
        else:
            response.message = 'Unable to stop the execution.' + \
                f'Error Id: {resp.error_id}' + f'message: {resp.raw}'
            response.success = False

        return response

    def _pause(self, _: EReq_req, response: EReq_Resp) -> EReq_Resp:
        resp: Optional[DobotDashboardModel] = None

        try:
            resp = self._dashboard.send_command(
                cmd='Pause()')

        except Exception as e:
            response.message = f'Socket exception occurred: {e}'
            response.success = False
            return response

        if resp.is_success:
            response.message = 'Paused the execution'
            response.success = True
        else:
            response.message = 'Unable to pause the execution.' + \
                f'Error Id: {resp.error_id}' + f'message: {resp.raw}'
            response.success = False

        return response

    def _continue(self, _: EReq_req, response: EReq_Resp) -> EReq_Resp:
        resp: Optional[DobotDashboardModel] = None

        try:
            resp = self._dashboard.send_command(
                cmd='ClearError()')

        except Exception as e:
            response.message = f'Socket exception occurred: {e}'
            response.success = False
            return response

        if resp.is_success:
            response.message = 'Continuing the execution'
            response.success = True
        else:
            response.message = 'Unable to continuing the execution. ' + \
                f'Error Id: {resp.error_id}' + f'message: {resp.raw}'
            response.success = False

        return response

    def _estop(self, _: EReq_req, response: EReq_Resp, en: bool) -> EReq_Resp:
        resp: Optional[DobotDashboardModel] = None

        try:
            resp = self._dashboard.send_command(
                cmd=f'EmergencyStop({int(en)})')

        except Exception as e:
            response.message = f'Socket exception occurred: {e}'
            response.success = False
            return response

        if resp.is_success:
            msg: str = 'Activating' if en else 'Deactivating'
            response.message = f'{msg} the E-Stop'
            response.success = True
        else:
            msg: str = 'Activate' if en else 'Deactivate'
            response.message = f'Unable to {msg} E-Stop. ' + \
                f'Error Id: {resp.error_id}' + f'message: {resp.raw}'
            response.success = False

        return response

    def _drag(self, _: EReq_req, response: EReq_Resp, en: bool) -> EReq_Resp:
        _cmd: str = 'StartDrag()' if en else 'StopDrag()'
        resp: Optional[DobotDashboardModel] = None
        try:
            resp = self._dashboard.send_command(cmd=_cmd)

        except Exception as e:
            response.message = f'Socket exception occurred: {e}'
            response.success = False
            return response

        if resp.is_success:
            msg: str = 'Entering' if en else 'Exiting'
            response.message = f'{msg} the drag mode'
            response.success = True
        else:
            msg = 'Enter' if en else 'Exit'
            response.message = (
                f'Unable to {msg} the Drag. Error Id: {resp.error_id}'
                + f'message: {resp.raw}'
            )
            response.success = False

        return response
