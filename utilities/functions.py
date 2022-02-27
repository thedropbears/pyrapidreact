import math
import typing
from typing import Callable, Sequence

import numpy as np


def constrain_angle(angle: float) -> float:
    """Wrap an angle to the interval [-pi,pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


# Workaround for numpy typing bug
interpolate = typing.cast(
    Callable[[float, Sequence[float], Sequence[float]], float],
    np.interp,
)
