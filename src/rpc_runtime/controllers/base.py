"""Common controller utilities and exceptions."""

from __future__ import annotations


class ControllerFault(RuntimeError):  # noqa: N818
    """Raised when a controller cannot produce a valid torque command."""

    def __init__(self, message: str, *, recoverable: bool = False) -> None:
        """Initialise the controller fault."""
        super().__init__(message)
        self.recoverable = recoverable
