"""
SettingsService -- ROS2 service handlers for CR5 robot settings.

Registers and handles all settings-related ROS2 services that
map directly to Dobot dashboard commands on port 29999.

Services registered:
    set_speed         -- Set global speed factor (1-100%)
    set_payload       -- Set robot payload and centre of mass
    set_motion_params -- Set motion parameters (AccJ/AccL/VelJ/VelL/CP)
"""

from typing import Any


from cr5_driver.robot.dashboard_model import DobotDashboardModel
from cr5_driver.tcp import DobotDashboardClient

from cr5_msgs.srv import SetMotionParams, SetPayload, SetSpeed
from cr5_msgs.srv._set_motion_params import (
    SetMotionParams_Request as SM_req,
    SetMotionParams_Response as SM_resp,
)
from cr5_msgs.srv._set_payload import (
    SetPayload_Request as SP_req,
    SetPayload_Response as SP_resp,
)
from cr5_msgs.srv._set_speed import (
    SetSpeed_Request as SS_req,
    SetSpeed_Response as SS_resp,
)

from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.impl.rcutils_logger import RcutilsLogger
from rclpy.node import Node

VALID_MOTION_PARAMS: frozenset[str] = frozenset(
    {'AccJ', 'AccL', 'VelJ', 'VelL', 'CP'})


class SettingsService:
    """
    ROS2 service handlers for CR5 robot settings commands.

    All services delegate to DobotDashboardClient which manages
    the TCP connection to port 29999. Uses ReentrantCallbackGroup
    to allow concurrent service execution.
    """

    def __init__(self, node: Node) -> None:
        """
        Initialise and register all settings services on the node.

        Parameters
        ----------
        node : Node
            ROS2 node to register services on.

        """
        self._log: RcutilsLogger = node.get_logger()
        self._log.info('Starting settings services...')

        cb = ReentrantCallbackGroup()

        node.create_service(
            SetSpeed, 'set_speed',
            self._set_speed, callback_group=cb
        )
        node.create_service(
            SetPayload, 'set_payload',
            self._set_payload, callback_group=cb
        )
        node.create_service(
            SetMotionParams, 'set_motion_params',
            self._set_motion_params, callback_group=cb
        )

        self._dashboard: DobotDashboardClient = DobotDashboardClient()
        self._log.info('Settings services started.')

    # ── Generic helper ────────────────────────────────────────────

    def _execute(
            self,
            response: SS_resp | SP_resp | SM_resp,
            cmd: str,
            success_msg: str,
            fail_msg: str
    ) -> Any:
        """
        Execute a dashboard command and populate the response.

        Parameters
        ----------
        response : SS_resp
            ROS2 service response to populate.
        cmd : str
            Dashboard command string to send.
        success_msg : str
            Message to set on success.
        fail_msg : str
            Prefix message to set on failure.

        Returns
        -------
        Any
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

    def _set_speed(
            self, request: SS_req, response: SS_resp
    ) -> SS_resp:
        """
        Handle set_speed service -- set global speed factor.

        Parameters
        ----------
        request : SS_req
            Request containing speed_percent (1-100).
        response : SS_resp
            Service response to populate.

        Returns
        -------
        SS_resp
            Populated response object.

        """
        ratio: int = request.speed_percent

        if not (1 <= ratio <= 100):
            response.success = False
            response.message = (
                f'Invalid speed: {ratio}. Must be between 1 and 100.'
            )
            return response

        return self._execute(
            response,
            cmd=f'SpeedFactor({ratio})',
            success_msg=f'Speed set to {ratio}%.',
            fail_msg='Failed to set speed.'
        )

    def _set_payload(
            self, request: SP_req, response: SP_resp
    ) -> SP_resp:
        """
        Handle set_payload service -- set payload and centre of mass.

        Parameters
        ----------
        request : SP_req
            Request containing load (kg) and x/y/z offsets (mm).
        response : SP_resp
            Service response to populate.

        Returns
        -------
        SP_resp
            Populated response object.

        """
        return self._execute(
            response,
            cmd=(
                f'SetPayload({request.load},'
                f'{request.x_offset},{request.y_offset},{request.z_offset})'
            ),
            success_msg=f'Payload set to {request.load}kg.',
            fail_msg='Failed to set payload.'
        )

    def _set_motion_params(
            self, request: SM_req, response: SM_resp
    ) -> SM_resp:
        """
        Handle set_motion_params service -- set motion parameter.

        Valid parameters: AccJ, AccL, VelJ, VelL, CP.

        Parameters
        ----------
        request : SM_req
            Request containing param name and percent value (1-100).
        response : SM_resp
            Service response to populate.

        Returns
        -------
        SM_resp
            Populated response object.

        """
        param: str = request.param
        ratio: int = request.percent

        if param not in VALID_MOTION_PARAMS:
            response.success = False
            response.message = (
                f'Invalid parameter: {param}. '
                f'Valid options: {sorted(VALID_MOTION_PARAMS)}'
            )
            return response

        if not (1 <= ratio <= 100):
            response.success = False
            response.message = (
                f'Invalid value: {ratio}. Must be between 1 and 100.'
            )
            return response

        return self._execute(
            response,
            cmd=f'{param}({ratio})',
            success_msg=f'{param} set to {ratio}%.',
            fail_msg=f'Failed to set {param}.'
        )
