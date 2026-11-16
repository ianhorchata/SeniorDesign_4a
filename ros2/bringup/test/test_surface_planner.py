"""Tests of the surface planner on small hand-built voxel worlds.

Run with:  python3 -m pytest ros2/bringup/test
"""

import numpy as np
import pytest

from bringup.surface_planner import (PlannerConfig, PlanningError, SurfacePlanner,
                                     voxels_from_centers)

RES = 0.05


def slab(xs, ys, z):
    """Voxels of a horizontal sheet: xs and ys are ranges of voxel indices."""
    return [(x, y, z) for x in xs for y in ys]


def top(x, y, z):
    """Metric position on top of voxel (x, y, z)."""
    return ((x + 0.5) * RES, (y + 0.5) * RES, (z + 1) * RES)


def distance_to(points, voxels):
    """Smallest horizontal distance from any path point to any of the voxel centers."""
    centers = (np.array(voxels)[:, :2] + 0.5) * RES
    return np.linalg.norm(points[:, None, :2] - centers[None, :, :], axis=2).min()


def test_voxels_from_centers_single_and_merged_nodes():
    centers = [
        (0.025, 0.025, 0.025),      # voxel (0, 0, 0)
        (-0.025, 0.075, -0.075),    # voxel (-1, 1, -2)
        (0.25, 0.05, 0.05),         # merged node of 2 voxels per side at (4..5, 0..1, 0..1)
    ]
    voxels = {tuple(v) for v in voxels_from_centers(centers, RES).tolist()}
    merged = {(x, y, z) for x in (4, 5) for y in (0, 1) for z in (0, 1)}
    assert voxels == {(0, 0, 0), (-1, 1, -2)} | merged


def test_open_floor_gives_a_straight_path():
    planner = SurfacePlanner(slab(range(60), range(60), 0))
    plan = planner.plan(top(10, 10, 0), top(50, 40, 0))
    assert np.allclose(plan.points[0], top(10, 10, 0))
    assert np.allclose(plan.points[-1], top(50, 40, 0))
    assert len(plan.waypoints) == 2
    assert plan.length == pytest.approx(np.hypot(40, 30) * RES, rel=0.01)
    spacing = np.linalg.norm(np.diff(plan.points, axis=0), axis=1)
    assert spacing.max() <= PlannerConfig().path_spacing + 1e-9


def test_path_goes_through_the_door_and_keeps_its_distance():
    floor = slab(range(80), range(80), 0)
    door = range(34, 46)        # 0.6 m wide
    wall = [(40, y, z) for y in range(80) if y not in door for z in range(1, 30)]
    planner = SurfacePlanner(floor + wall)
    plan = planner.plan(top(10, 10, 0), top(70, 70, 0))
    crossing = plan.points[np.argmin(np.abs(plan.points[:, 0] - 40.5 * RES))]
    assert door[0] * RES < crossing[1] < (door[-1] + 1) * RES
    # Voxel centers are half a voxel inside the wall.
    assert distance_to(plan.points, wall) >= planner.config.clearance + RES / 2 - 1e-6
    assert plan.length > np.hypot(60, 60) * RES


def test_wall_without_door_cannot_be_passed():
    floor = slab(range(80), range(80), 0)
    wall = [(40, y, z) for y in range(80) for z in range(1, 30)]
    planner = SurfacePlanner(floor + wall)
    with pytest.raises(PlanningError, match='cannot be reached'):
        planner.plan(top(10, 10, 0), top(70, 70, 0))


def test_path_stays_away_from_the_edge_of_the_floor():
    planner = SurfacePlanner(slab(range(40), range(40), 0))
    plan = planner.plan((0.0, 0.0, 0.05), (2.0, 0.0, 0.05))
    clearance = planner.config.clearance
    assert plan.points[:, :2].min() >= clearance - 1e-6
    assert plan.points[:, :2].max() <= 40 * RES - clearance + 1e-6


def test_goal_off_the_map_is_rejected():
    planner = SurfacePlanner(slab(range(40), range(40), 0))
    with pytest.raises(PlanningError, match='goal'):
        planner.plan(top(20, 20, 0), (5.0, 5.0, 0.0))


@pytest.mark.parametrize('height, passable', [(2, False), (5, False), (10, False), (11, True), (30, True)])
def test_headroom(height, passable):
    """A sheet across the floor blocks the robot unless it hangs above the headroom (10 voxels).

    This is also what a wall looks like whose lowest part the lidar did not see.
    """
    floor = slab(range(60), range(30), 0)
    sheet = slab(range(25, 35), range(30), height)
    planner = SurfacePlanner(floor + sheet)
    start, goal = top(8, 15, 0), top(52, 15, 0)
    if passable:
        plan = planner.plan(start, goal)
        assert plan.points[:, 2].max() == pytest.approx(RES)
    else:
        with pytest.raises(PlanningError):
            planner.plan(start, goal)


@pytest.mark.parametrize('step, passable', [(1, True), (2, False)])
def test_step_height(step, passable):
    """One voxel up is within max_step, two are not."""
    low = slab(range(30), range(30), 0)
    high = slab(range(30, 60), range(30), step)
    planner = SurfacePlanner(low + high)
    start, goal = top(8, 15, 0), top(52, 15, step)
    if passable:
        plan = planner.plan(start, goal)
        assert plan.points[-1, 2] == pytest.approx((step + 1) * RES, abs=RES / 2)
    else:
        with pytest.raises(PlanningError, match='cannot be reached'):
            planner.plan(start, goal)


def two_floors(with_ramp):
    """Lower floor 5 x 2 m, upper floor 1 m above its left 2 m, ramp 1 m wide between them."""
    lower = slab(range(100), range(40), 0)
    upper = slab(range(40), range(40), 20)
    ramp = [(x, y, round((79 - x) * 0.5)) for x in range(40, 80) for y in range(10, 30)]
    return lower + upper + (ramp if with_ramp else [])


def test_ramp_connects_two_floors():
    planner = SurfacePlanner(two_floors(with_ramp=True))
    start, goal = top(90, 4, 0), top(10, 20, 20)
    plan = planner.plan(start, goal)
    assert np.allclose(plan.points[0], start, atol=RES)
    assert np.allclose(plan.points[-1], goal, atol=RES)
    # Up the ramp, between its sides.
    climbing = plan.points[(plan.points[:, 2] > 0.2) & (plan.points[:, 2] < 0.9)]
    assert len(climbing) > 10
    assert climbing[:, 1].min() >= 10 * RES + planner.config.clearance - 1e-6
    assert climbing[:, 1].max() <= 30 * RES - planner.config.clearance + 1e-6
    # No jump in height: the ramp rises 0.05 m per 0.1 m of path, plus the voxel steps.
    assert np.abs(np.diff(plan.points[:, 2])).max() <= 1.5 * RES
    assert plan.length > 3.5


def test_floors_without_ramp_are_not_connected():
    planner = SurfacePlanner(two_floors(with_ramp=False))
    with pytest.raises(PlanningError, match='cannot be reached'):
        planner.plan(top(90, 4, 0), top(10, 20, 20))


def test_goal_snaps_to_the_floor_at_its_own_height():
    """The same x, y exists on both floors: z decides which one is meant."""
    planner = SurfacePlanner(two_floors(with_ramp=True))
    x, y = 10, 20
    low = planner.plan(top(90, 4, 0), top(x, y, 0))
    high = planner.plan(top(90, 4, 0), top(x, y, 20))
    assert low.points[-1, 2] == pytest.approx(RES, abs=1e-6)
    assert high.points[-1, 2] == pytest.approx(21 * RES, abs=1e-6)


def test_gaps_in_the_floor_are_closed():
    """A floor seen only as scattered voxels is still one floor."""
    sparse = [(x, y, 0) for x in range(60) for y in range(60) if (x + 2 * y) % 5 == 0]
    start, goal = top(10, 10, 0), top(50, 45, 0)
    plan = SurfacePlanner(sparse).plan(start, goal)
    assert plan.length == pytest.approx(np.hypot(40, 35) * RES, rel=0.02)
    with pytest.raises(PlanningError):
        SurfacePlanner(sparse, PlannerConfig(floor_gap=0.0)).plan(start, goal)


def test_gap_filling_does_not_extend_the_floor():
    """Filling must not push the floor out over a drop."""
    planner = SurfacePlanner(slab(range(40), range(40), 0))
    assert planner.positions[:, :2].min() >= planner.config.clearance - 1e-6
    assert planner.positions[:, :2].max() <= 40 * RES - planner.config.clearance + 1e-6


def test_stray_voxels_below_the_floor_do_not_break_it():
    """Single voxels a little under the real floor, where the floor itself was not seen."""
    floor = [(x, y, 0) for x in range(60) for y in range(60) if (x, y) not in {(20, 30), (40, 31)}]
    strays = [(20, 30, -2), (40, 31, -3)]
    plan = SurfacePlanner(floor + strays).plan(top(5, 30, 0), top(55, 30, 0))
    assert len(plan.waypoints) == 2


def test_low_obstacle_on_the_floor_is_avoided():
    floor = slab(range(60), range(60), 0)
    box = [(x, y, z) for x in range(28, 32) for y in range(26, 34) for z in (1, 2)]
    planner = SurfacePlanner(floor + box)
    plan = planner.plan(top(5, 30, 0), top(55, 30, 0))
    assert len(plan.waypoints) > 2
    assert distance_to(plan.points, box) >= planner.config.clearance + RES / 2 - 1e-6


def test_paths_prefer_the_middle_of_a_corridor():
    """A 1.5 m wide corridor: the path runs along its center line, not along a wall."""
    floor = slab(range(100), range(30), 0)
    walls = [(x, y, z) for x in range(100) for y in (0, 29) for z in range(1, 20)]
    plan = SurfacePlanner(floor + walls).plan(top(10, 6, 0), top(90, 6, 0))
    middle = plan.points[(plan.points[:, 0] > 1.5) & (plan.points[:, 0] < 3.5)]
    assert middle[:, 1].min() > 9 * RES
