# python/nanobind_roscpp/__init__.py
"""
nanobind_roscpp
— Python bindings for roscpp via nanobind.

Exposes:
  Point(x: float, y: float)
  distance(a: Point, b: Point) → float
"""
from .point_utils_py import Point, distance

__all__ = ["Point", "distance"]