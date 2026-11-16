"""Path planning on the drivable surfaces of a 3D voxel map (no ROS in here).

The map is the set of occupied voxels of an OctoMap. The planner works in three steps:

1. Surface graph. A cell is floor when its voxel is occupied and the voxels above
   it are not, up to the headroom. Small gaps that the sensor left in the floor
   are closed. A floor cell is drivable when the floor continues around it for at
   least the clearance: no obstacle, no step that is too high, no drop and no
   unmapped space closer than that. Drivable cells are connected to their 8
   horizontal neighbours, whose floor may be up to max_step higher or lower.
   Floors on different levels are separate sheets of cells, and a ramp connects
   them because it rises a little per cell.
2. A* search over that graph. The cost of an edge is its 3D length, increased
   near obstacles so that paths keep to the middle of corridors.
3. Shortcutting. Runs of cells are replaced by straight lines wherever the
   straight line stays on drivable cells and is not more expensive.

Voxel (ix, iy, iz) covers [ix, ix + 1) * resolution in x, likewise in y and z.
A path point lies on top of its floor voxel, at z = (iz + 1) * resolution.
"""

import heapq
import math
from bisect import bisect_left
from dataclasses import dataclass

import numpy as np

# The 8 horizontal directions: (dx, dy).
DIRECTIONS = [(dx, dy) for dx in (-1, 0, 1) for dy in (-1, 0, 1) if (dx, dy) != (0, 0)]


class PlanningError(Exception):
    """No path could be planned. The message says why."""


@dataclass
class PlannerConfig:
    resolution: float = 0.05           # m, voxel size of the map
    headroom: float = 0.50             # m, height above the floor in which nothing may be occupied.
    #                                    More than the robot height: the map only holds what the
    #                                    lidar hit, and it misses the lowest part of distant walls.
    max_step: float = 0.05             # m, largest floor height change between neighbouring cells
    floor_gap: float = 0.30            # m, gaps in the mapped floor narrower than this count as floor
    clearance: float = 0.20            # m, minimum distance from the robot center to an obstacle
    #                                    or to the end of the known floor (robot radius + margin)
    preferred_clearance: float = 0.45  # m, paths are not penalised beyond this distance
    clearance_weight: float = 2.0      # extra cost per metre at minimum clearance, as a factor
    snap_distance: float = 0.5         # m, how far start/goal may be from a drivable cell
    path_spacing: float = 0.10         # m, distance between the points of the returned path


@dataclass
class Plan:
    points: np.ndarray      # (n, 3) path on the floor surface, path_spacing apart, start to goal
    waypoints: np.ndarray   # (m, 3) corner points that the straight segments run between
    length: float           # m, along the path


def voxels_from_centers(centers, resolution):
    """Voxel indices (n, 3) from the centers of the occupied OctoMap nodes.

    OctoMap merges 8 equal neighbours into one node of twice the size. Such a node is
    recognisable from its center alone: in units of half a voxel a single voxel has odd
    coordinates, a node of 2 voxels per side has 2 * odd, 4 voxels has 4 * odd and so on.
    """
    half = np.rint(np.asarray(centers, dtype=np.float64) / (resolution / 2)).astype(np.int64)
    half = half.reshape(-1, 3)
    side = half[:, 0] & -half[:, 0]     # lowest set bit = voxels per side
    voxels = [(half[side == 1] - 1) // 2]
    for s in np.unique(side[side > 1]):
        corner = (half[side == s] - s) // 2
        offsets = np.stack(np.meshgrid(*[np.arange(s)] * 3, indexing='ij'), axis=-1).reshape(-1, 3)
        voxels.append((corner[:, None, :] + offsets[None, :, :]).reshape(-1, 3))
    return np.unique(np.concatenate(voxels), axis=0)


def _find(sorted_values, query):
    """Index of each query value in sorted_values, or -1 where it is absent."""
    if len(sorted_values) == 0:
        return np.full(len(query), -1, dtype=np.int64)
    index = np.minimum(np.searchsorted(sorted_values, query), len(sorted_values) - 1)
    return np.where(sorted_values[index] == query, index, -1)


def _contains(sorted_values, query):
    return _find(sorted_values, query) >= 0


class SurfacePlanner:
    """Drivable surface graph of one voxel map, and A* on it."""

    def __init__(self, voxels, config=None):
        self.config = config or PlannerConfig()
        cfg = self.config
        res = cfg.resolution
        voxels = np.asarray(voxels, dtype=np.int64).reshape(-1, 3)
        if len(voxels) == 0:
            raise PlanningError('The map is empty.')

        # All lengths in whole voxels.
        self._headroom = max(1, math.ceil(cfg.headroom / res - 1e-6))
        self._step = max(0, math.floor(cfg.max_step / res + 1e-6))
        self._gap = max(0, math.ceil(cfg.floor_gap / res - 1e-6))
        reach = math.ceil(cfg.preferred_clearance / res) + 1

        # Cells are handled as positions in a (virtual) grid around the map, flattened so
        # that z runs fastest: the cell above is +1 and a sorted array is ordered by
        # column. The empty border keeps neighbour offsets from wrapping around.
        pad = np.array([self._gap + 2, self._gap + 2, self._headroom + self._step + 2])
        self._origin = voxels.min(axis=0) - pad
        self._shape = tuple(int(n) for n in voxels.max(axis=0) - self._origin + 1 + pad)
        ny, nz = self._shape[1], self._shape[2]
        self._offsets = [(dx * ny + dy) * nz for dx, dy in DIRECTIONS]
        occupied = np.unique(np.ravel_multi_index(tuple((voxels - self._origin).T), self._shape))

        # 1. Floor seen by the sensor: occupied, with nothing occupied in the headroom above.
        is_floor = np.ones(len(occupied), dtype=bool)
        for k in range(1, self._headroom + 1):
            is_floor &= ~_contains(occupied, occupied + k)
        cells = self._fill_gaps(self._drop_strays(occupied[is_floor]), occupied)

        # 2. Which cells can the robot stand on with its whole footprint?
        neighbour = self._neighbours(cells)
        clearance = self._clearance(neighbour, reach) * res
        keep = clearance >= cfg.clearance
        if not keep.any():
            raise PlanningError('The map has no drivable floor yet.')
        renumber = np.cumsum(keep) - 1
        neighbour = np.where((neighbour >= 0) & keep[neighbour], renumber[neighbour], -1)[:, keep]
        self._flat = cells[keep]                 # ascending, so cells can be found by bisection
        self._flat_list = self._flat.tolist()
        self._clearance = clearance[keep]
        index = np.stack(np.unravel_index(self._flat, self._shape), axis=1)
        self._index_list = index.tolist()
        self._step_order = sorted(range(-self._step, self._step + 1), key=abs)
        self.positions = (index + self._origin + np.array([0.5, 0.5, 1.0])) * res

        # 3. Graph. Cost factor of a cell: 1 in the open, 1 + clearance_weight at minimum clearance.
        span = max(cfg.preferred_clearance - cfg.clearance, 1e-6)
        self._factor = 1.0 + cfg.clearance_weight * np.clip(
            (cfg.preferred_clearance - self._clearance) / span, 0.0, 1.0)
        self._factor_list = self._factor.tolist()
        count = len(self._flat)
        height = index[:, 2]
        sources, targets, costs = [], [], []
        for (dx, dy), target in zip(DIRECTIONS, neighbour):
            source = np.nonzero(target >= 0)[0]
            target = target[source]
            length = res * np.sqrt(dx * dx + dy * dy + (height[target] - height[source]) ** 2)
            sources.append(source)
            targets.append(target)
            costs.append(length * 0.5 * (self._factor[source] + self._factor[target]))
        sources = np.concatenate(sources)
        by_source = np.argsort(sources, kind='stable')
        self._edge_start = np.searchsorted(sources[by_source], np.arange(count + 1)).tolist()
        self._edge_target = np.concatenate(targets)[by_source].tolist()
        self._edge_cost = np.concatenate(costs)[by_source].tolist()

    # ------------------------------------------------------------------ surface

    def _neighbours(self, cells):
        """For each of the 8 directions, the index of the cell next door or -1.

        The floor may be up to max_step higher or lower there; the nearest height wins.
        """
        result = np.full((len(DIRECTIONS), len(cells)), -1, dtype=np.int64)
        for row, offset in zip(result, self._offsets):
            for dz in sorted(range(-self._step, self._step + 1), key=abs):
                todo = np.nonzero(row < 0)[0]
                row[todo] = _find(cells, cells[todo] + offset + dz)
        return result

    def _drop_strays(self, floor):
        """Remove floor cells that disagree in height with the floor around them.

        A scan that is slightly off leaves single voxels a little below or above the
        real floor. Such a cell has neighbours, but at heights it does not connect to.
        Where it only hid a gap in the real floor, _fill_gaps closes that gap again.
        """
        agree = np.zeros(len(floor), dtype=np.int64)
        conflict = np.zeros(len(floor), dtype=np.int64)
        for offset in self._offsets:
            same_level = np.zeros(len(floor), dtype=bool)
            other_level = np.zeros(len(floor), dtype=bool)
            for dz in range(-self._headroom, self._headroom + 1):
                there = _contains(floor, floor + offset + dz)
                if abs(dz) <= self._step:
                    same_level |= there
                else:
                    other_level |= there
            agree += same_level
            conflict += other_level & ~same_level
        return floor[conflict <= agree]

    def _fill_gaps(self, floor, occupied):
        """Add the floor cells that the sensor missed between the ones it saw.

        A lidar close to the ground sees the floor as scattered points. Gaps narrower
        than floor_gap are closed: the floor is grown sideways by that distance, and
        then taken back by the same distance from wherever it still borders the unknown.
        """
        headroom, gap = self._headroom, self._gap
        if gap == 0 or len(floor) == 0:
            return floor
        cells = floor
        frontier = floor
        for _ in range(gap):
            new = np.unique(np.concatenate([frontier + offset for offset in self._offsets]))
            new = new[~_contains(cells, new)]
            fits = np.ones(len(new), dtype=bool)
            for dz in range(-headroom, headroom + 1):     # the column has no floor at this level yet
                fits &= ~_contains(cells, new + dz)
            for k in range(headroom + 1):                 # and nothing is in the way
                fits &= ~_contains(occupied, new + k)
            new = new[fits]
            # Neighbours at different heights can propose the same column: keep one.
            new = new[np.concatenate([[True], np.diff(new) > headroom])] if len(new) else new
            if len(new) == 0:
                break
            cells = np.union1d(cells, new)
            frontier = new

        # Cells that border the unknown: a direction with no floor and nothing occupied.
        neighbour = self._neighbours(cells)
        open_side = np.zeros(len(cells), dtype=bool)
        for row, offset in zip(neighbour, self._offsets):
            unknown = np.nonzero(row < 0)[0]
            for dz in range(-self._step, headroom + 1):
                unknown = unknown[~_contains(occupied, cells[unknown] + offset + dz)]
            open_side[unknown] = True
        taken_back = open_side.copy()
        frontier = np.nonzero(open_side)[0]
        for _ in range(gap - 1):
            around = neighbour[:, frontier].ravel()
            around = np.unique(around[around >= 0])
            frontier = around[~taken_back[around]]
            taken_back[frontier] = True
        return cells[_contains(floor, cells) | ~taken_back]

    def _clearance(self, neighbour, reach):
        """Distance in cells from each cell to where the drivable floor ends.

        The floor ends in every direction without a neighbour: at an obstacle, a step
        that is too high, a drop or unmapped space. Distances are spread from there
        across the floor, cell to cell, as the offset to the nearest such place.
        Cells further away than reach get reach.
        """
        count = neighbour.shape[1]
        far = 4 * reach
        offset_x = np.full(count, far, dtype=np.int64)
        offset_y = np.full(count, far, dtype=np.int64)
        squared = offset_x ** 2 + offset_y ** 2
        for (dx, dy), row in zip(DIRECTIONS, neighbour):
            closer = (row < 0) & (dx * dx + dy * dy < squared)
            offset_x[closer], offset_y[closer] = dx, dy
            squared[closer] = dx * dx + dy * dy
        active = np.nonzero(squared < 2 * far * far)[0]
        for _ in range(reach):
            changed = np.zeros(count, dtype=bool)
            for (dx, dy), row in zip(DIRECTIONS, neighbour):
                target = row[active]
                source = active[target >= 0]
                target = target[target >= 0]
                x, y = offset_x[source] - dx, offset_y[source] - dy
                closer = x * x + y * y < squared[target]
                target = target[closer]
                offset_x[target], offset_y[target] = x[closer], y[closer]
                squared[target] = offset_x[target] ** 2 + offset_y[target] ** 2
                changed[target] = True
            active = np.nonzero(changed)[0]
            if len(active) == 0:
                break
        # From the cell center to the border of the first cell that is not floor.
        return np.minimum(np.sqrt(squared) - 0.5, reach)

    # ------------------------------------------------------------------ queries

    @property
    def clearances(self):
        """Clearance in metres of every drivable cell, in the order of positions."""
        return self._clearance

    def nearest_cell(self, point):
        """Index of the drivable cell closest to point, or None if none is within snap_distance."""
        distance = np.linalg.norm(self.positions - np.asarray(point, dtype=float), axis=1)
        nearest = int(np.argmin(distance))
        return nearest if distance[nearest] <= self.config.snap_distance else None

    def plan(self, start, goal):
        """Plan from start to goal, both (x, y, z) in metres. Raises PlanningError."""
        start_cell = self.nearest_cell(start)
        if start_cell is None:
            raise PlanningError(
                'The start {} is not on mapped, drivable floor.'.format(np.round(start, 2).tolist()))
        goal_cell = self.nearest_cell(goal)
        if goal_cell is None:
            raise PlanningError(
                'The goal {} is not on mapped, drivable floor.'.format(np.round(goal, 2).tolist()))
        cells, cost = self._astar(start_cell, goal_cell)
        if cells is None:
            raise PlanningError('The goal cannot be reached from the start on the mapped floor.')
        corners = self._shortcut(cells, cost)
        points = self._trace_points(corners)
        length = float(np.linalg.norm(np.diff(points, axis=0), axis=1).sum())
        return Plan(points=points, waypoints=self.positions[corners], length=length)

    # ------------------------------------------------------------------ search

    def _astar(self, start, goal):
        """Cheapest cell sequence from start to goal and the cost so far at each of its cells."""
        # Straight-line distance never overestimates, because every cost factor is >= 1.
        heuristic = np.linalg.norm(self.positions - self.positions[goal], axis=1).tolist()
        edge_start, edge_target, edge_cost = self._edge_start, self._edge_target, self._edge_cost
        cost_so_far = {start: 0.0}
        parent = {}
        closed = set()
        frontier = [(heuristic[start], start)]
        while frontier:
            _, cell = heapq.heappop(frontier)
            if cell == goal:
                path = [cell]
                while cell in parent:
                    cell = parent[cell]
                    path.append(cell)
                path.reverse()
                return path, [cost_so_far[c] for c in path]
            if cell in closed:
                continue
            closed.add(cell)
            cost_here = cost_so_far[cell]
            for edge in range(edge_start[cell], edge_start[cell + 1]):
                neighbour = edge_target[edge]
                new_cost = cost_here + edge_cost[edge]
                if new_cost < cost_so_far.get(neighbour, math.inf):
                    cost_so_far[neighbour] = new_cost
                    parent[neighbour] = cell
                    heapq.heappush(frontier, (new_cost + heuristic[neighbour], neighbour))
        return None, None

    # ------------------------------------------------------------------ straight lines

    def _trace(self, a, b):
        """Cells under the straight line from cell a to cell b, following the floor.

        Returns (cells, cost), or (None, inf) if the line leaves the drivable cells.
        The cost is that of driving the line, counted like the edges of the graph.
        The line is walked from cell border to cell border in whole numbers, so that
        a line through the corner of four cells takes the diagonal.
        """
        res = self.config.resolution
        ny, nz = self._shape[1], self._shape[2]
        flat, factor = self._flat_list, self._factor_list
        x, y, z = self._index_list[a]
        end_x, end_y, _ = self._index_list[b]
        step_x = 1 if end_x > x else -1
        step_y = 1 if end_y > y else -1
        count_x, count_y = abs(end_x - x), abs(end_y - y)
        length = res * math.hypot(count_x, count_y)
        cell = a
        cells = [a]
        cost = 0.0
        entered = 0.0       # fraction of the line at which the current cell was entered
        rise_in = 0.0       # half of the height change on entering it, m
        done_x = done_y = 0
        while done_x < count_x or done_y < count_y:
            # Which border comes first: the next one in x or the next one in y?
            order = (1 + 2 * done_x) * count_y - (1 + 2 * done_y) * count_x
            if order <= 0:
                left = (0.5 + done_x) / count_x
                x += step_x
                done_x += 1
            if order >= 0:
                left = (0.5 + done_y) / count_y
                y += step_y
                done_y += 1
            # Continue on the floor cell of the new column, within the step height.
            base = (x * ny + y) * nz + z
            for dz in self._step_order:
                index = bisect_left(flat, base + dz)
                if index < len(flat) and flat[index] == base + dz:
                    break
            else:
                return None, math.inf
            rise_out = 0.5 * res * abs(dz)
            cost += factor[cell] * math.hypot((left - entered) * length, rise_in + rise_out)
            cell, z, entered, rise_in = index, z + dz, left, rise_out
            cells.append(cell)
        if cell != b:
            return None, math.inf
        cost += factor[cell] * math.hypot((1.0 - entered) * length, rise_in)
        return cells, cost

    def _shortcut(self, cells, cost):
        """Indices of the cells to keep as corners after replacing runs by straight lines."""
        corners = [cells[0]]
        anchor = 0
        last = len(cells) - 1
        while anchor < last:
            def visible(index):
                _, line_cost = self._trace(cells[anchor], cells[index])
                return line_cost <= (cost[index] - cost[anchor]) * (1 + 1e-9)

            # The next cell is always reachable. Bisect for a far cell that still is.
            reach = last
            if not visible(last):
                low, high = anchor + 1, last
                while high - low > 1:
                    middle = (low + high) // 2
                    if visible(middle):
                        low = middle
                    else:
                        high = middle
                reach = low
            corners.append(cells[reach])
            anchor = reach
        return corners

    def _trace_points(self, corners):
        """Evenly spaced points along the corner-to-corner lines, on the floor surface."""
        if len(corners) == 1:
            return self.positions[corners].copy()
        res = self.config.resolution
        points = [self.positions[corners[0]]]
        for a, b in zip(corners[:-1], corners[1:]):
            cells, _ = self._trace(a, b)
            if cells is None:       # neighbouring cells of the A* path
                cells = [a, b]
            start, end = self.positions[a], self.positions[b]
            direction = end[:2] - start[:2]
            length = float(np.linalg.norm(direction))
            heights = self.positions[cells][:, 2]
            # Distance along the line at which each of its cells is passed.
            along = (self.positions[cells][:, :2] - start[:2]) @ direction / max(length, 1e-9)
            count = max(1, math.ceil(length / self.config.path_spacing))
            for i in range(1, count + 1):
                s = length * i / count
                xy = start[:2] + direction * (i / count)
                points.append(np.array([xy[0], xy[1], np.interp(s, along, heights)]))
        points = np.array(points)
        # Voxel heights come in steps of one voxel; average them over a short distance.
        window = max(1, int(round(6 * res / self.config.path_spacing))) | 1
        if window > 1 and len(points) > 2:
            padded = np.pad(points[:, 2], window // 2, mode='edge')
            points[:, 2] = np.convolve(padded, np.ones(window) / window, mode='valid')
        return points
