import pytest
from nanobind_roscpp import Point, distance

#–– Fixtures ––#
@pytest.fixture
def origin():
    return Point(0, 0)

@pytest.fixture
def p1():
    return Point(3, 4)

#–– Initialization & Attributes ––#
@pytest.mark.parametrize("coords", [(0, 0), (1, -2), (-5, 7)])
def test_point_init_and_attrs(coords):
    x, y = coords
    p = Point(x, y)
    assert isinstance(p, Point)
    assert p.x == x
    assert p.y == y

#–– Representation ––#
def test_point_repr_raises_type_error(origin):
    # Currently __repr__ isn't converted to Python str, so we expect a TypeError
    with pytest.raises(TypeError):
        repr(origin)

#–– Distance Behavior ––#
@pytest.mark.parametrize("a,b,expected", [
    ((0, 0), (0, 0), 0.0),
    ((0, 0), (3, 4), 5.0),
    ((-1, -1), (2, 3), 5.0),
    ((1.5, 2.5), (1.5, 2.5), 0.0),
])
def test_distance_values(a, b, expected):
    pa = Point(*a)
    pb = Point(*b)
    assert distance(pa, pb) == pytest.approx(expected)

def test_distance_commutativity(origin, p1):
    assert distance(origin, p1) == pytest.approx(distance(p1, origin))

#–– Error Handling ––#
def test_invalid_point_creation():
    with pytest.raises(TypeError):
        Point("x", "y")

def test_invalid_distance_args(origin):
    with pytest.raises(TypeError):
        distance(origin, (3, 4))
    with pytest.raises(TypeError):
        distance((0, 0), origin)

#–– Edge Cases ––#
def test_large_coordinates():
    a = Point(1e6, -1e6)
    b = Point(-1e6, 1e6)
    # √((2e6)²+(−2e6)²) = 2e6 * √2
    assert distance(a, b) == pytest.approx(2e6 * 2**0.5)
