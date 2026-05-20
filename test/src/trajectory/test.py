import numpy as np
import math
import heapq

from scipy.interpolate import CubicSpline
from scipy.interpolate import splprep, splev


class Astar:
    def __init__(self, warehouse, cell_size):
        self.warehouse = warehouse
        self.cell_size = cell_size

        self.x_min = -self.warehouse.width * 0.5
        self.y_min = -self.warehouse.height * 0.5

    # ======================================================
    # EXISTING GRID BUILD
    # ======================================================

    def build_grid(self, agv_size):

        nx = int(self.warehouse.width / self.cell_size)
        ny = int(self.warehouse.height / self.cell_size)

        grid = np.zeros((ny, nx), dtype=np.uint8)

        xs = self.x_min + (np.arange(nx) + 0.5) * self.cell_size
        ys = self.y_min + (np.arange(ny) + 0.5) * self.cell_size

        for obstacle in self.warehouse.obstacles:
            xmin = obstacle.xmin - agv_size
            xmax = obstacle.xmax + agv_size
            ymin = obstacle.ymin - agv_size
            ymax = obstacle.ymax + agv_size

            col_min = np.searchsorted(xs, xmin, side="left")
            col_max = np.searchsorted(xs, xmax, side="right")
            row_min = np.searchsorted(ys, ymin, side="left")
            row_max = np.searchsorted(ys, ymax, side="right")

            col_min = max(0, col_min)
            col_max = min(nx, col_max)
            row_min = max(0, row_min)
            row_max = min(ny, row_max)

            grid[row_min:row_max, col_min:col_max] = 1

        return grid

    # ======================================================
    # COORDINATE CONVERSION
    # ======================================================

    def world_to_grid(self, x, y):
        col = int((x - self.x_min) / self.cell_size)
        row = int((y - self.y_min) / self.cell_size)
        return row, col

    def grid_to_world(self, row, col):
        x = self.x_min + (col + 0.5) * self.cell_size
        y = self.y_min + (row + 0.5) * self.cell_size
        return x, y

    # ======================================================
    # STANDARD A*
    # ======================================================

    def path(self, grid, start, goal):
        def heuristic(a, b):
            return math.hypot(a[0] - b[0], a[1] - b[1])

        ny, nx = grid.shape

        neighbors = [
            (-1, 0, 1.0),
            (1, 0, 1.0),
            (0, -1, 1.0),
            (0, 1, 1.0),
            (-1, -1, math.sqrt(2)),
            (-1, 1, math.sqrt(2)),
            (1, -1, math.sqrt(2)),
            (1, 1, math.sqrt(2)),
        ]

        open_set = []
        heapq.heappush(open_set, (0, start))

        came_from = {}
        g_score = {start: 0}

        while open_set:
            _, current = heapq.heappop(open_set)

            if current == goal:
                path = []
                while current in came_from:
                    path.append(current)
                    current = came_from[current]
                path.append(start)
                path = [(x - 0.5, y - 0.5) for x, y in path]

                return path[::-1]

            for dy, dx, base_cost in neighbors:
                ny_ = current[0] + dy
                nx_ = current[1] + dx

                if 0 <= ny_ < ny and 0 <= nx_ < nx:
                    if grid[ny_, nx_] == 1:
                        continue

                    tentative = g_score[current] + base_cost
                    neighbor = (ny_, nx_)

                    if neighbor not in g_score or tentative < g_score[neighbor]:
                        g_score[neighbor] = tentative
                        f = tentative + heuristic(neighbor, goal)
                        heapq.heappush(open_set, (f, neighbor))
                        came_from[neighbor] = current

        return None

    # ======================================================
    # HYBRID A* WITH TURN RADIUS CONSTRAINT
    # ======================================================

    def path_turn_radius(
        self,
        grid,
        start_xyyaw,
        goal_xyyaw,
        turn_radius,
        heading_bins=36,
        step_length=None,
        goal_tolerance=1.5,
        heading_tolerance_deg=20,
    ):
        """
        start_xyyaw = (x, y, yaw_rad)
        goal_xyyaw  = (x, y, yaw_rad)

        turn_radius in meters
        """

        if step_length is None:
            step_length = self.cell_size

        heading_tol = math.radians(heading_tolerance_deg)

        ny, nx = grid.shape

        # ---------------------------------------------
        # helpers
        # ---------------------------------------------

        def wrap_pi(a):
            while a > math.pi:
                a -= 2 * math.pi
            while a < -math.pi:
                a += 2 * math.pi
            return a

        def heading_idx(yaw):
            yaw = wrap_pi(yaw)
            if yaw < 0:
                yaw += 2 * math.pi
            return int((yaw / (2 * math.pi)) * heading_bins) % heading_bins

        def state_key(x, y, yaw):
            row, col = self.world_to_grid(x, y)
            return (row, col, heading_idx(yaw))

        def heuristic(x, y):
            return math.hypot(goal_xyyaw[0] - x, goal_xyyaw[1] - y)

        def reached(x, y, yaw):
            d = math.hypot(goal_xyyaw[0] - x, goal_xyyaw[1] - y)
            dyaw = abs(wrap_pi(yaw - goal_xyyaw[2]))
            return d <= goal_tolerance and dyaw <= heading_tol

        def valid(x, y):
            row, col = self.world_to_grid(x, y)

            if row < 0 or row >= ny or col < 0 or col >= nx:
                return False

            if grid[row, col] == 1:
                return False

            return True

        # ---------------------------------------------
        # motion primitives
        # ---------------------------------------------

        dtheta = step_length / turn_radius

        def propagate(x, y, yaw, mode):
            """
            mode:
             -1 left
              0 straight
             +1 right
            """

            if mode == 0:
                nx_ = x + step_length * math.cos(yaw)
                ny_ = y + step_length * math.sin(yaw)
                nyaw = yaw
                return nx_, ny_, nyaw

            sign = -1 if mode == -1 else 1

            nyaw = yaw + sign * dtheta

            nx_ = x + sign * turn_radius * (
                math.sin(nyaw) - math.sin(yaw)
            )

            ny_ = y - sign * turn_radius * (
                math.cos(nyaw) - math.cos(yaw)
            )

            return nx_, ny_, wrap_pi(nyaw)

        # ---------------------------------------------
        # open set
        # ---------------------------------------------

        open_set = []
        came_from = {}
        g_score = {}

        sx, sy, syaw = start_xyyaw

        start_key = state_key(sx, sy, syaw)
        g_score[start_key] = 0.0

        heapq.heappush(
            open_set,
            (heuristic(sx, sy), (sx, sy, syaw)),
        )

        visited = {}

        while open_set:
            _, current = heapq.heappop(open_set)

            x, y, yaw = current
            ck = state_key(x, y, yaw)

            if ck in visited:
                continue
            visited[ck] = True

            if reached(x, y, yaw):
                return self._reconstruct_turn_path(
                    came_from, current
                )

            for mode in (-1, 0, 1):
                nx_, ny_, nyaw = propagate(x, y, yaw, mode)

                if not valid(nx_, ny_):
                    continue

                nk = state_key(nx_, ny_, nyaw)

                tentative = g_score[ck] + step_length

                if nk not in g_score or tentative < g_score[nk]:
                    g_score[nk] = tentative

                    priority = tentative + heuristic(nx_, ny_)

                    heapq.heappush(
                        open_set,
                        (priority, (nx_, ny_, nyaw)),
                    )

                    came_from[(nx_, ny_, nyaw)] = current

        return None

    def _reconstruct_turn_path(self, came_from, current):
        path = [current]

        while current in came_from:
            current = came_from[current]
            path.append(current)

        path.reverse()
        return path

    # ======================================================
    # OPTIONAL SMOOTHING
    # ======================================================

    def cubic_spline_path(self, path, num_points, smoothing=10.0):
        """
        Smoothed spline path using scipy splprep/splev.

        Parameters
        ----------
        path : list[(x,y)] or list[(x,y,yaw)]
            Input path points.

        num_points : int
            Number of output interpolated samples.

        smoothing : float
            Higher = smoother / wider turns.
            Lower = follows original points more tightly.
            0 = exact interpolation.

        Returns
        -------
        list[(x,y)]
        """

        if path is None or len(path) < 3:
            return path

        pts = np.array(path, dtype=float)

        x = pts[:, 0]
        y = pts[:, 1]

        # remove duplicate consecutive points
        keep = [0]
        for i in range(1, len(pts)):
            if np.hypot(x[i] - x[keep[-1]], y[i] - y[keep[-1]]) > 1e-6:
                keep.append(i)

        x = x[keep]
        y = y[keep]

        if len(x) < 3:
            return list(zip(x, y))

        # spline fit
        tck, u = splprep(
            [x, y],
            s=smoothing,
            k=min(3, len(x) - 1)
        )

        u_fine = np.linspace(0.0, 1.0, num_points)

        x_smooth, y_smooth = splev(u_fine, tck)

        return list(zip(x_smooth, y_smooth))
