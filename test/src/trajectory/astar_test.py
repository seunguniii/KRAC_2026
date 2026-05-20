"""
Compare classic A* vs turn-radius constrained A*
in an EMPTY warehouse (no obstacles).

Waypoint chain:
(0,0) -> (10,0) -> (20,-10) -> (20,20) -> (10,0) -> (0,0)
"""

import math
import matplotlib.pyplot as plt

from test import Astar

# =====================================================
# EMPTY MAP
# =====================================================

class Warehouse:
    def __init__(self, width, height):
        self.width = width
        self.height = height
        self.obstacles = []   # explicitly empty


# =====================================================
# USER WAYPOINTS
# =====================================================

wps = [
    (0, 0),
    (10, 0),
    (20, -10),
    (20, 10),
    (10, 0),
    (0, 0),
]

# =====================================================
# CREATE EMPTY ENVIRONMENT
# =====================================================

warehouse = Warehouse(width=80, height=80)

planner = Astar(warehouse, cell_size=1.0)

# zero obstacles -> empty occupancy grid
grid = planner.build_grid(agv_size=0.0)

# =====================================================
# HELPERS
# =====================================================

def heading_between(a, b):
    return math.atan2(b[1] - a[1], b[0] - a[0])


def path_length(path):
    if not path or len(path) < 2:
        return 0.0

    total = 0.0
    for i in range(1, len(path)):
        x0, y0 = path[i - 1][:2]
        x1, y1 = path[i][:2]
        total += math.hypot(x1 - x0, y1 - y0)

    return total


# =====================================================
# CLASSIC A*
# =====================================================

classic = []

for i in range(len(wps) - 1):
    s = planner.world_to_grid(*wps[i])
    g = planner.world_to_grid(*wps[i + 1])

    seg = planner.path(grid, s, g)

    seg_world = [planner.grid_to_world(r, c) for r, c in seg]

    if i > 0:
        seg_world = seg_world[1:]

    classic.extend(seg_world)

spline = planner.cubic_spline_path(classic, 1000)

# =====================================================
# TURN-RADIUS A*
# =====================================================

TURN_RADIUS = 5.0
turn = []

for i in range(len(wps) - 1):
    A = wps[i]
    B = wps[i + 1]

    yaw_start = heading_between(A, B)

    # no specified target yaw:
    # choose outbound direction toward next leg
    if i < len(wps) - 2:
        yaw_goal = heading_between(B, wps[i + 2])
    else:
        yaw_goal = yaw_start

    seg = planner.path_turn_radius(
        grid=grid,
        start_xyyaw=(A[0], A[1], yaw_start),
        goal_xyyaw=(B[0], B[1], yaw_goal),
        turn_radius=TURN_RADIUS,
        heading_bins=36,
        step_length=1.0,
        goal_tolerance=1.,
        heading_tolerance_deg=45,
    )

    if seg is None:
        print("Failed segment:", A, "->", B)
        continue

    if i > 0:
        seg = seg[1:]

    turn.extend(seg)


# =====================================================
# METRICS
# =====================================================

print("Classic A* length:", round(path_length(classic), 2))
print("Turn-radius A* length:", round(path_length(turn), 2))


# =====================================================
# PLOT
# =====================================================

plt.figure(figsize=(10, 10))

# classic
cx = [p[0] for p in classic]
cy = [p[1] for p in classic]
plt.plot(cx, cy, linewidth=2, label="Classic A*")

# constrained
tx = [p[0] for p in turn]
ty = [p[1] for p in turn]
plt.plot(tx, ty, linewidth=2, label="Turn-Radius A*")

tx = [p[0] for p in spline]
ty = [p[1] for p in spline]
plt.plot(tx, ty, linewidth=2, label="Splined A*")

# waypoints
wx = [p[0] for p in wps]
wy = [p[1] for p in wps]
plt.scatter(wx, wy, s=80, label="Setpoints")

for i, p in enumerate(wps):
    plt.text(p[0] + 0.25, p[1] + 0.25, str(i))

plt.title("Comparisons of A* paths")
plt.xlabel("X")
plt.ylabel("Y")
plt.axis("equal")
plt.grid(True)
plt.legend()
plt.show()
