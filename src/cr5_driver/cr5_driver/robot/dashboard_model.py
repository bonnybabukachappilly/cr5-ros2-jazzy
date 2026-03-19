"""
DobotDashboardModel -- parsed representation of CR5 dashboard response.

Handles the string-based response format from port 29999:
    ErrorID,{return_values},CommandName(param1,param2,...);
"""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class DobotDashboardModel:
    """
    Immutable representation of a Dobot dashboard command response.

    Return values are stored as strings -- callers are responsible
    for casting to the appropriate type (e.g. int for RobotMode).
    """

    error_id: int
    return_values: list[str]
    command_name: str
    params: list[str]
    raw: str

    @property
    def is_success(self) -> bool:
        """Return True if the command was accepted (ErrorID == 0)."""
        return self.error_id == 0

    @classmethod
    def from_str(cls, raw_str: str) -> DobotDashboardModel:
        """
        Parse a raw response string into a model instance.

        Parameters
        ----------
        raw_str : str
            Raw string received from port 29999.

        Returns
        -------
        DobotDashboardModel
            Parsed and structured response.

        Raises
        ------
        ValueError
            If the string does not match the expected protocol format.

        """
        clean_str: str = raw_str.strip().rstrip(';')
        parts: list[str] = clean_str.split(',', 2)

        if len(parts) < 3:
            raise ValueError(
                f'Malformed robot response: {raw_str}'
            )

        try:
            error_id: int = int(parts[0])

            ret_vals: list[str] = []
            ret_part: str = parts[1]
            if '{' in ret_part and '}' in ret_part:
                inner: str = ret_part[
                    ret_part.find('{') + 1: ret_part.find('}')
                ]
                ret_vals = [
                    v.strip() for v in inner.split(',') if v.strip()
                ]

            cmd_part: str = parts[2]
            name: str = cmd_part.split('(', 1)[0]

            params: list[str] = []
            if '(' in cmd_part and ')' in cmd_part:
                start: int = cmd_part.find('(') + 1
                end: int = cmd_part.rfind(')')
                param_str: str = cmd_part[start:end]
                params = [
                    p.strip() for p in param_str.split(',') if p.strip()
                ]

            return cls(
                error_id=error_id,
                return_values=ret_vals,
                command_name=name,
                params=params,
                raw=raw_str
            )

        except (IndexError, ValueError) as e:
            raise ValueError(
                f"Failed to parse response '{raw_str}': {e}"
            ) from e
