"""
CR5 TCP client modules.

Provides TCP clients for communicating with the Dobot CR5
over its network ports:

- DobotDashboardClient: port 29999 -- command and control
- DobotFeedbackClient: port 30004 -- real-time state feedback
"""

from .dashboard_client import DobotDashboardClient
from .feedback_client import DobotFeedbackClient

__all__: list[str] = [
    'DobotDashboardClient',
    'DobotFeedbackClient',
]
