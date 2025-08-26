"""
ROS2 interface parsing utilities.

Contains classes and functions for parsing ROS2 action interface files.
"""

from typing import Optional
from rosidl_adapter.parser import (
    parse_message_string,
    ACTION_REQUEST_RESPONSE_SEPARATOR,
    Field,
    MessageSpecification,
)


class InterfaceTextLine:
    """A convenience class for a single text line in an interface file."""

    def __init__(
        self,
        pkg_name: str,
        msg_name: str,
        line_text: str,
    ):
        if line_text in (ACTION_REQUEST_RESPONSE_SEPARATOR,):
            msg_spec = None
        else:
            msg_spec = parse_message_string(
                pkg_name=pkg_name,
                msg_name=msg_name,
                message_string=line_text,
            )
            if msg_spec and len(msg_spec.fields) > 1:
                raise ValueError("'line_text' must be only one line")
        self._msg_spec: Optional[MessageSpecification] = msg_spec
        self._raw_line_text = line_text

    def __str__(self) -> str:
        return self._raw_line_text

    def is_comment(self) -> bool:
        return self._msg_spec and self._msg_spec.annotations.get("comment", False)

    def is_trailing_comment(self) -> bool:
        return self._is_field_trailing_comment()

    def _is_field_trailing_comment(self) -> bool:
        return self._field and self._field.annotations.get("comment", False)

    @property
    def trailing_comment(self) -> Optional[str]:
        if self._is_field_trailing_comment():
            return self._field.annotations["comment"][0]
        else:
            return None

    @property
    def _field(self) -> Optional[Field]:
        if self._msg_spec and self._msg_spec.fields:
            return self._msg_spec.fields[0]

    @property
    def field_name(self) -> Optional[str]:
        if self._field:
            return self._field.name
        return None

    @property
    def field_type(self) -> Optional[str]:
        if self._field:
            return str(self._field.type)
        return None
