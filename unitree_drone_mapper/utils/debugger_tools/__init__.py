"""
debugger_tools/ — Debug and status utilities for the DronePi pipeline.

This package provides:
    - DebugSaver: Saves intermediate PLY files for inspection
    - PipelineStatus: Tracks execution stage for error reporting
    - write_failure_status: Writes metadata.json on pipeline failure

Usage:
    from debugger_tools import DebugSaver, PipelineStatus, write_failure_status
"""

from .debug_saver import DebugSaver
from .status import PipelineStatus, PipelineStage, write_failure_status

__all__ = [
    'DebugSaver',
    'PipelineStatus',
    'PipelineStage',
    'write_failure_status',
]
