import numpy as np

from t3mpt3d.navigation_node import plank_visible

RES = 0.05
A, B = (1.5, 0.5), (1.5, 1.5)   # vertical plank at x = 1.5


def wall_grid():
    occ = np.zeros((40, 40), bool)   # 2 x 2 m, origin (0, 0)
    occ[:, 20] = True                # wall at x = 1.0..1.05
    return occ


def visible(robot, vision_radius, a=A, b=B):
    return plank_visible(robot, a, b, vision_radius, wall_grid(), 0.0, 0.0, RES)


def test_close_same_side():
    assert visible((1.2, 1.0), 0.5)            # 0.3 m from the middle


def test_wall_blocks():
    assert not visible((0.5, 1.0), 1.5)        # in range, wall in between


def test_past_end_uses_endpoint():
    assert visible((1.3, 1.9), 0.5)            # clipped to B, 0.45 m


def test_too_far():
    assert not visible((1.2, 1.0), 0.2)        # 0.3 m > 0.2 m


def test_zero_length_plank():
    assert visible((1.2, 1.0), 0.5, a=(1.5, 1.0), b=(1.5, 1.0))


def test_returns_bool():
    assert isinstance(visible((1.2, 1.0), 0.5), (bool, np.bool_))
