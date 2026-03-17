
from typing import Optional

from cr5_driver.robot.dashboard_model import DobotDashboardModel
from cr5_msgs.srv import (
    SetSpeed, SetPayload, SetMotionParams
)

from cr5_msgs.srv._set_speed import (
    SetSpeed_Request as SS_req, SetSpeed_Response as SS_resp
)
from cr5_msgs.srv._set_payload import (
    SetPayload_Request as SP_req, SetPayload_Response as SP_resp
)
from cr5_msgs.srv._set_motion_params import (
    SetMotionParams_Request as SM_req, SetMotionParams_Response as SM_resp
)

from cr5_driver.tcp import DobotDashboardClient

from rclpy.impl.rcutils_logger import RcutilsLogger
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup


class SettingsService:
    def __init__(self, node: Node) -> None:
        self._log: RcutilsLogger = node.get_logger()

        self._log.info('Starting control services...')

        cb = ReentrantCallbackGroup()

        # Set Speed
        node.create_service(
            srv_type=SetSpeed,
            srv_name='set_speed',
            callback=self._set_speed,
            callback_group=cb
        )

        # Set Payload
        node.create_service(
            srv_type=SetPayload,
            srv_name='set_payload',
            callback=self._set_payload,
            callback_group=cb
        )

        # Motion Parameters
        node.create_service(
            srv_type=SetMotionParams,
            srv_name='set_motion_params',
            callback=self._set_motion_param,
            callback_group=cb
        )

        self._log.info('Settings Services are started.')

        self._dashboard: DobotDashboardClient = DobotDashboardClient()

    def _set_speed(self, request: SS_req, response: SS_resp) -> SS_resp:
        # sourcery skip: class-extract-method
        ratio: int = request.speed_percent

        resp: Optional[DobotDashboardModel] = None
        try:
            if 0 < ratio <= 100:
                raise ValueError('Should be between 1 - 100')
            resp = self._dashboard.send_command(cmd=f'SpeedFactor({ratio})')

        except Exception as e:
            response.message = f'Socket exception occurred: {e}'
            response.success = False
            return response

        if resp.is_success:
            response.message = f'Speed set to: {ratio}'
            response.success = True
        else:
            response.message = (
                f'Failed to set the robot speed. Error Id: {resp.error_id}'
                + f'message: {resp.raw}'
            )
            response.success = False

        return response

    def _set_payload(self, request: SP_req, response: SP_resp) -> SP_resp:
        load: float = request.load
        _x: float = request.x_offset
        _y: float = request.y_offset
        _z: float = request.z_offset

        resp: Optional[DobotDashboardModel] = None
        try:
            resp = self._dashboard.send_command(
                cmd=f'SetPayload({load},{_x},{_y},{_z})')

        except Exception as e:
            response.message = f'Socket exception occurred: {e}'
            response.success = False
            return response

        if resp.is_success:
            response.message = f'Payload set to: {load}'
            response.success = True
        else:
            response.message = (
                f'Failed to set the robot payload. Error Id: {resp.error_id}'
                + f'message: {resp.raw}'
            )
            response.success = False

        return response

    def _set_motion_param(self, request: SM_req, response: SM_resp) -> SM_resp:
        ratio: int = request.percent
        param: str = request.param

        resp: Optional[DobotDashboardModel] = None
        try:
            match param:
                case 'AccJ':
                    resp = self._dashboard.send_command(cmd=f'AccJ({ratio})')
                case 'AccL':
                    resp = self._dashboard.send_command(cmd=f'AccL({ratio})')
                case 'VelJ':
                    resp = self._dashboard.send_command(cmd=f'VelJ({ratio})')
                case 'VelL':
                    resp = self._dashboard.send_command(cmd=f'VelL({ratio})')
                case 'CP':
                    resp = self._dashboard.send_command(cmd=f'CP({ratio})')
                case _:
                    raise ValueError('Parameter do not match')

        except Exception as e:
            response.message = f'Socket exception occurred: {e}'
            response.success = False
            return response

        if resp.is_success:
            response.message = f'Speed set to: {ratio}'
            response.success = True
        else:
            response.message = (
                f'Failed to set the robot speed. Error Id: {resp.error_id}'
                + f'message: {resp.raw}'
            )
            response.success = False

        return response
