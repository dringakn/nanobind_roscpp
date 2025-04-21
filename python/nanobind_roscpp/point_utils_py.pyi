# python/nanobind_roscpp/point_utils_py.pyi
"""
Bindings for PointUtils
"""

class Point:
    """A 2D point with x and y coordinates."""
    x: float
    y: float
    def __init__(self, x: float = ..., y: float = ...) -> None: ...
    def __repr__(self) -> str: ...

def distance(a: Point, b: Point) -> float: ...
