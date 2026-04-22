"""Mini SLAM with incremental mapping, dynamic grid bounds, and A* replanning."""

from dataclasses import dataclass, field
import heapq
import math
import threading
import time
from typing import Dict, List, Optional, Set, Tuple

import numpy as np

from agrotechsimapi import HighLevelSimClient


GridCell = Tuple[int, int]

IP = "127.0.0.1"
PORT = "1233"

GRID_RESOLUTION_M = 0.25

LIDAR_UPDATE_HZ = 5.0
LIDAR_PERIOD_S = 1.0 / LIDAR_UPDATE_HZ
LIDAR_RANGE_MAX_M = 8.0
LIDAR_NUM_RAYS = 360
LIDAR_HIT_MIN_DISTANCE_M = 0.35
LIDAR_MAX_HIT_EPS_M = 0.05

OBSTACLE_INFLATION_CELLS = 1
DRONE_CLEAR_RADIUS_M = 0.6
GOAL_CLEAR_RADIUS_M = 0.35
PLAN_PADDING_CELLS = 16
MAX_JUMP_AHEAD_CELLS = 24

GOAL_ODOM_M = (13.0, 1.0)
GOAL_REACHED_RADIUS_M = 0.2
FLIGHT_HEIGHT_M = 1.75
MISSION_TIMEOUT_S = 180.0


def world_to_cell(x: float, y: float) -> GridCell:
    """Convert XY coordinates in meters to a grid cell index."""
    return int(round(x / GRID_RESOLUTION_M)), int(round(y / GRID_RESOLUTION_M))


def cell_to_world(cell: GridCell) -> Tuple[float, float]:
    """Convert a grid cell index to XY coordinates in meters."""
    return cell[0] * GRID_RESOLUTION_M, cell[1] * GRID_RESOLUTION_M


def heuristic(cell_a: GridCell, cell_b: GridCell) -> float:
    """Estimate A* remaining cost with Euclidean distance."""
    return math.hypot(cell_a[0] - cell_b[0], cell_a[1] - cell_b[1])


@dataclass
class DynamicOccupancyGrid:
    """Incremental occupancy map with dynamic planning bounds."""

    occupied: Set[GridCell] = field(default_factory=set)
    obstacle_history: List[GridCell] = field(default_factory=list)
    min_x: Optional[int] = None
    max_x: Optional[int] = None
    min_y: Optional[int] = None
    max_y: Optional[int] = None

    def include_cell(self, cell: GridCell) -> None:
        """Expand internal bounds to include a cell."""
        x_cell, y_cell = cell
        if self.min_x is None:
            self.min_x = self.max_x = x_cell
            self.min_y = self.max_y = y_cell
            return
        self.min_x = min(self.min_x, x_cell)
        self.max_x = max(self.max_x, x_cell)
        self.min_y = min(self.min_y, y_cell)
        self.max_y = max(self.max_y, y_cell)

    def mark_obstacle(self, cell: GridCell) -> None:
        """Mark obstacle cell and append to historical log once."""
        self.include_cell(cell)
        if cell not in self.occupied:
            self.occupied.add(cell)
            self.obstacle_history.append(cell)

    def mark_obstacles(self, cells: Set[GridCell]) -> None:
        """Add a batch of obstacle cells."""
        for cell in cells:
            self.mark_obstacle(cell)

    def planning_bounds(
        self, start: GridCell, goal: GridCell, padding_cells: int = PLAN_PADDING_CELLS
    ) -> Tuple[int, int, int, int]:
        """Return dynamic planning rectangle expanded by padding."""
        self.include_cell(start)
        self.include_cell(goal)

        min_x = min(self.min_x if self.min_x is not None else start[0], start[0], goal[0]) - padding_cells
        max_x = max(self.max_x if self.max_x is not None else start[0], start[0], goal[0]) + padding_cells
        min_y = min(self.min_y if self.min_y is not None else start[1], start[1], goal[1]) - padding_cells
        max_y = max(self.max_y if self.max_y is not None else start[1], start[1], goal[1]) + padding_cells
        return min_x, max_x, min_y, max_y


def in_dynamic_bounds(cell: GridCell, bounds: Tuple[int, int, int, int]) -> bool:
    """Check whether cell is inside dynamic planning rectangle."""
    min_x, max_x, min_y, max_y = bounds
    return min_x <= cell[0] <= max_x and min_y <= cell[1] <= max_y


def neighbors(cell: GridCell, bounds: Tuple[int, int, int, int]) -> List[Tuple[GridCell, float]]:
    """Return 8-connected neighbors inside dynamic bounds."""
    x_cell, y_cell = cell
    result: List[Tuple[GridCell, float]] = []
    for dx, dy in (
        (-1, 0),
        (1, 0),
        (0, -1),
        (0, 1),
        (-1, -1),
        (-1, 1),
        (1, -1),
        (1, 1),
    ):
        nxt = (x_cell + dx, y_cell + dy)
        if not in_dynamic_bounds(nxt, bounds):
            continue
        step_cost = math.sqrt(2.0) if dx != 0 and dy != 0 else 1.0
        result.append((nxt, step_cost))
    return result


def reconstruct_path(came_from: Dict[GridCell, GridCell], current: GridCell) -> List[GridCell]:
    """Rebuild A* path from parent links."""
    path = [current]
    while current in came_from:
        current = came_from[current]
        path.append(current)
    path.reverse()
    return path


def astar_path(
    start: GridCell,
    goal: GridCell,
    blocked: Set[GridCell],
    bounds: Tuple[int, int, int, int],
) -> Optional[List[GridCell]]:
    """Plan path with A* inside dynamic bounds."""
    if start == goal:
        return [start]
    if goal in blocked:
        return None
    if not in_dynamic_bounds(start, bounds) or not in_dynamic_bounds(goal, bounds):
        return None

    open_heap: List[Tuple[float, GridCell]] = []
    heapq.heappush(open_heap, (heuristic(start, goal), start))

    came_from: Dict[GridCell, GridCell] = {}
    g_score: Dict[GridCell, float] = {start: 0.0}
    closed: Set[GridCell] = set()

    while open_heap:
        _, current = heapq.heappop(open_heap)
        if current in closed:
            continue
        if current == goal:
            return reconstruct_path(came_from, current)

        closed.add(current)

        for nxt, step_cost in neighbors(current, bounds):
            if nxt in blocked or nxt in closed:
                continue
            tentative_g = g_score[current] + step_cost
            if tentative_g >= g_score.get(nxt, float("inf")):
                continue
            came_from[nxt] = current
            g_score[nxt] = tentative_g
            heapq.heappush(open_heap, (tentative_g + heuristic(nxt, goal), nxt))

    return None


def add_inflated_obstacle(occupied: Set[GridCell], center: GridCell, radius_cells: int) -> None:
    """Mark obstacle and neighbors around it."""
    cx, cy = center
    for dx in range(-radius_cells, radius_cells + 1):
        for dy in range(-radius_cells, radius_cells + 1):
            occupied.add((cx + dx, cy + dy))


def clear_cells_in_radius(occupied: Set[GridCell], center: GridCell, radius_cells: int) -> None:
    """Clear cells in a circle-like radius around center."""
    cx, cy = center
    for dx in range(-radius_cells, radius_cells + 1):
        for dy in range(-radius_cells, radius_cells + 1):
            if dx * dx + dy * dy > radius_cells * radius_cells:
                continue
            occupied.discard((cx + dx, cy + dy))


def bresenham_line(start: GridCell, end: GridCell) -> List[GridCell]:
    """Return integer grid cells on a line between two points."""
    x0, y0 = start
    x1, y1 = end

    dx = abs(x1 - x0)
    dy = -abs(y1 - y0)
    sx = 1 if x0 < x1 else -1
    sy = 1 if y0 < y1 else -1
    err = dx + dy

    cells: List[GridCell] = []
    while True:
        cells.append((x0, y0))
        if x0 == x1 and y0 == y1:
            break
        e2 = 2 * err
        if e2 >= dy:
            err += dy
            x0 += sx
        if e2 <= dx:
            err += dx
            y0 += sy
    return cells


def has_line_of_sight(start: GridCell, end: GridCell, blocked: Set[GridCell]) -> bool:
    """Check if straight segment crosses blocked cells."""
    cells = bresenham_line(start, end)
    if len(cells) <= 2:
        return True
    return not any(cell in blocked for cell in cells[1:-1])


def select_jump_target_index(
    current_cell: GridCell,
    path: List[GridCell],
    next_index: int,
    blocked: Set[GridCell],
) -> int:
    """Pick farthest visible waypoint to avoid step-by-step motion."""
    max_index = min(len(path) - 1, next_index + MAX_JUMP_AHEAD_CELLS)
    for idx in range(max_index, next_index - 1, -1):
        if has_line_of_sight(current_cell, path[idx], blocked):
            return idx
    return next_index


def build_planning_blocked(
    occupied: Set[GridCell], current_cell: GridCell, goal_cell: GridCell
) -> Set[GridCell]:
    """Build blocked set with local bubbles around drone and goal."""
    blocked = set(occupied)
    clear_cells_in_radius(
        blocked,
        current_cell,
        max(1, int(round(DRONE_CLEAR_RADIUS_M / GRID_RESOLUTION_M))),
    )
    clear_cells_in_radius(
        blocked,
        goal_cell,
        max(1, int(round(GOAL_CLEAR_RADIUS_M / GRID_RESOLUTION_M))),
    )
    return blocked


def lidar_hits_to_cells(client: HighLevelSimClient) -> Set[GridCell]:
    """Convert one lidar scan to world-grid obstacle cells."""
    scan = client.getLidarScan(
        angle_min=-math.pi,
        angle_max=math.pi,
        range_min=0.1,
        range_max=LIDAR_RANGE_MAX_M,
        num_ranges=LIDAR_NUM_RAYS,
        is_clear=True,
        range_error=0.0,
    )
    distances = np.asarray(scan, dtype=float)
    if distances.size == 0:
        return set()

    odom = client.getOdomOpticflow()
    drone_x, drone_y = float(odom[0]), float(odom[1])
    yaw_ccw = float(client.getRPY()[2])

    cos_yaw = math.cos(yaw_ccw)
    sin_yaw = math.sin(yaw_ccw)
    angles = np.linspace(-math.pi, math.pi, num=distances.size, endpoint=False)

    cells: Set[GridCell] = set()
    for distance, angle in zip(distances, angles):
        if distance <= LIDAR_HIT_MIN_DISTANCE_M:
            continue
        if distance == 0.0:
            continue
        if distance >= (LIDAR_RANGE_MAX_M - LIDAR_MAX_HIT_EPS_M):
            continue

        x_drone = float(distance * -math.cos(angle + math.pi / 2.0))
        y_drone = float(distance * math.sin(angle + math.pi / 2.0))

        x_world = drone_x + x_drone * cos_yaw - y_drone * sin_yaw
        y_world = drone_y + x_drone * sin_yaw + y_drone * cos_yaw

        hit_cell = world_to_cell(x_world, y_world)
        add_inflated_obstacle(cells, hit_cell, OBSTACLE_INFLATION_CELLS)

    return cells


def start_waypoint_navigation(
    client: HighLevelSimClient, target_xy: Tuple[float, float]
) -> Tuple[threading.Thread, Dict[str, bool]]:
    """Start blocking gotoXYodom in a background thread."""
    state = {"done": False, "ok": False}

    def _worker() -> None:
        try:
            state["ok"] = client.gotoXYodom(target_xy[0], target_xy[1])
        finally:
            state["done"] = True

    worker = threading.Thread(target=_worker, daemon=True)
    worker.start()
    return worker, state


def stop_active_goto(client: HighLevelSimClient) -> None:
    """Stop active go_to_xy command via dedicated API or fallback."""
    if hasattr(client, "stop_go_to_xy"):
        client.stop_go_to_xy()
    elif hasattr(client, "stopGoToXY"):
        client.stopGoToXY()
    else:
        client.abort()


def main() -> None:
    """Run mini SLAM mission with incremental map and dynamic replanning."""
    client = HighLevelSimClient(drone_name="EDU_EXTENDED")
    connected = False

    grid = DynamicOccupancyGrid()

    active_thread: Optional[threading.Thread] = None
    active_state: Optional[Dict[str, bool]] = None
    active_target_index: Optional[int] = None

    path: List[GridCell] = []
    next_index = 1
    need_replan = True

    try:
        client.connect(IP, PORT)
        connected = True
        client.armDrone()
        client.altholdOn()

        print("Takeoff:", client.takeoff())
        print(f"Set height {FLIGHT_HEIGHT_M}:", client.setHeight(FLIGHT_HEIGHT_M))
        print("Reset odometry:", client.setZeroOdomOpticflow())

        goal_cell = world_to_cell(*GOAL_ODOM_M)
        grid.include_cell((0, 0))
        grid.include_cell(goal_cell)

        deadline = time.monotonic() + MISSION_TIMEOUT_S

        while time.monotonic() < deadline:
            cycle_started = time.monotonic()

            odom = client.getOdomOpticflow()
            current_xy = (float(odom[0]), float(odom[1]))
            current_cell = world_to_cell(*current_xy)
            grid.include_cell(current_cell)

            try:
                scan_cells = lidar_hits_to_cells(client)
                if scan_cells:
                    grid.mark_obstacles(scan_cells)
            except Exception as error:
                print(f"[slam] Lidar update skipped: {error}")

            blocked = build_planning_blocked(grid.occupied, current_cell, goal_cell)

            if math.dist(current_xy, GOAL_ODOM_M) <= GOAL_REACHED_RADIUS_M:
                print("[slam] Goal reached.")
                break

            if active_thread is not None and active_state is not None and active_state["done"]:
                if active_state["ok"] and active_target_index is not None:
                    next_index = max(next_index, active_target_index + 1)
                else:
                    need_replan = True
                active_thread = None
                active_state = None
                active_target_index = None

            if (
                active_thread is not None
                and active_thread.is_alive()
                and active_target_index is not None
                and active_target_index < len(path)
            ):
                target_cell = path[active_target_index]
                if not has_line_of_sight(current_cell, target_cell, blocked):
                    print("[slam] Obstacle appeared on active segment. Stopping go_to_xy.")
                    stop_active_goto(client)
                    active_thread.join(timeout=2.0)
                    active_thread = None
                    active_state = None
                    active_target_index = None
                    need_replan = True

            if not need_replan and path and next_index < len(path):
                if any(cell in blocked for cell in path[next_index:]):
                    need_replan = True

            if need_replan and active_thread is None:
                bounds = grid.planning_bounds(current_cell, goal_cell)
                new_path = astar_path(current_cell, goal_cell, blocked, bounds)
                if not new_path:
                    print("[slam] No path found now. Hover and rescan.")
                    elapsed = time.monotonic() - cycle_started
                    time.sleep(max(0.0, LIDAR_PERIOD_S - elapsed))
                    continue
                path = new_path
                next_index = 1
                need_replan = False
                print(
                    f"[slam] Replanned path: {len(path)} cells | "
                    f"map points: {len(grid.obstacle_history)}"
                )

            if active_thread is None and path and next_index < len(path):
                jump_index = select_jump_target_index(current_cell, path, next_index, blocked)
                target_cell = path[jump_index]
                target_xy = cell_to_world(target_cell)

                print(
                    f"[slam] Go to cell {target_cell} "
                    f"(idx {jump_index}/{len(path) - 1})"
                )
                active_thread, active_state = start_waypoint_navigation(client, target_xy)
                active_target_index = jump_index

            elapsed = time.monotonic() - cycle_started
            time.sleep(max(0.0, LIDAR_PERIOD_S - elapsed))
        else:
            print("[slam] Mission timeout reached.")

        if active_thread is not None and active_thread.is_alive():
            stop_active_goto(client)
            active_thread.join(timeout=2.0)

        print("Final approach to goal:", client.gotoXYodom(GOAL_ODOM_M[0], GOAL_ODOM_M[1]))
    finally:
        if active_thread is not None and active_thread.is_alive():
            stop_active_goto(client)
            active_thread.join(timeout=2.0)

        if connected:
            try:
                if client.getArm():
                    print("Landing:", client.boarding())
                    time.sleep(2.0)
            finally:
                client.disconnect()


if __name__ == "__main__":
    main()
