"""Compatibility import for the Pure Pursuit UDP runtime.

The competition controller was changed from Stanley to Pure Pursuit.  This
module remains so older launch commands and imports do not fail.
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
