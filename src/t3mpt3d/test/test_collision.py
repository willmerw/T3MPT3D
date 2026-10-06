import numpy as np

from t3mpt3d.navigation_node import disk_offsets, points_free, segment_free


def make_grid(h, w):
    return np.zeros((h, w), bool), np.zeros((h, w), bool)


def test_disk_offsets():
    dr, dc = disk_offsets(0.25, 0.05)
    assert len(dr) == 81
    assert ((dr == 0) & (dc == 0)).any()
    assert dr.min() == dc.min() == -5 and dr.max() == dc.max() == 5
    assert len(disk_offsets(0.12, 0.05)[0]) == 21


def test_points_free():
    occ, unk = make_grid(20, 20)
    occ[5, 5] = True
    unk[15, 15] = True
    dr, dc = disk_offsets(0.2, 0.1)
    pts = np.array([
        [0.55, 0.55],   # on the obstacle
        [0.55, 0.75],   # 2 cells away, inside R
        [0.55, 0.85],   # 3 cells away
        [0.05, 0.05],   # disk sticks out of the map
        [1.55, 1.55],   # centre on unknown
        [1.55, 1.25],   # unknown inside disk, not at centre
        [-0.5, 1.0],    # outside the map
    ])
    free = points_free(pts, occ, unk, 0.0, 0.0, 0.1, dr, dc)
    assert list(free) == [False, False, True, False, False, True, False]


def wall_grid():
    occ, unk = make_grid(40, 40)
    occ[:, 20] = True   # wall at x = 1.0..1.05
    return occ, unk


def seg(a, b, occ, unk, spacing=0.27, R=0.25, res=0.05):
    dr, dc = disk_offsets(R, res)
    return segment_free(a, b, spacing, occ, unk, 0.0, 0.0, res, dr, dc)


def test_segment_free():
    occ, unk = wall_grid()
    assert seg((0.5, 0.3), (0.5, 1.7), occ, unk)        # parallel to wall
    assert not seg((0.5, 1.0), (1.5, 1.0), occ, unk)    # crosses wall
    assert seg((0.85, 1.0), (0.4, 1.0), occ, unk)       # start near wall skipped
    assert not seg((0.4, 1.0), (0.85, 1.0), occ, unk)   # end near wall checked
    assert seg((0.5, 1.0), (0.5, 1.0), occ, unk)        # zero length


def test_segment_one_cell_gap():
    occ, unk = wall_grid()
    occ[20, 20] = False
    assert not seg((0.5, 1.025), (1.5, 1.025), occ, unk)


def test_segment_gap_with_plan_path_formula():
    # Same R and spacing as plan_path; keep in sync if that formula changes.
    bot_radius, margin, res = 0.113, 0.1, 0.05
    c = bot_radius + margin
    R = c + 0.71 * res
    spacing = 2 * np.sqrt(R**2 - c**2)
    occ, unk = wall_grid()
    occ[19:21, 20] = False   # 0.1 m gap, robot needs ~0.43 m
    assert not seg((0.5, 1.0), (1.5, 1.0), occ, unk, spacing, R, res)
