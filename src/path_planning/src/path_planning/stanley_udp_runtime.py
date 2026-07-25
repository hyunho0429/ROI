"""Compatibility import for the Stanley UDP runtime.

The latest UDP runner implementation lives in ``pure_pursuit_udp_runtime`` for
backward-compatible file names, but the controller used on this branch is
Stanley.
"""

from path_planning.pure_pursuit_udp_runtime import (  # noqa: F401
    DEFAULT_GLOBAL_INFO,
    DEFAULT_PATH,
    _projection,
    argument_parser,
    main,
    run,
)


__all__ = [
    "DEFAULT_GLOBAL_INFO",
    "DEFAULT_PATH",
    "argument_parser",
    "main",
    "run",
]
