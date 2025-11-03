import math
import heapq
import yaml
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import PoseStamped
from ament_index_python.packages import get_package_share_directory


class PathPlannerPublisher(Node):
    """
    A* planner on an inflated occupancy grid, with optional smoothing and interpolation.
    Listens to 'path_valid' (std_msgs/Bool) and auto-replans with larger inflation when invalid.
    """

    def __init__(self):
        super().__init__('path_planner')

        # -------- Parameters --------
        self.declare_parameter('planner_name', 'A*')
        self.declare_parameter('path_topic', 'planned_path')
        self.declare_parameter('env_file', 'env1.yaml')

        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('resolution', 0.10)            # m/cell
        self.declare_parameter('inflate_radius', 0.0)         # if 0 -> robot_radius + extra_clearance
        self.declare_parameter('extra_clearance', 0.05)       # meters beyond robot_radius
        self.declare_parameter('eight_connected', True)       # allow diagonals
        self.declare_parameter('interpolate_ds', 0.05)        # m between output samples
        self.declare_parameter('smooth', True)                # line-of-sight shortcutting on inflated grid

        # Auto-replan wiring
        self.declare_parameter('auto_replan_on_invalid', True)
        self.declare_parameter('replan_cooldown_sec', 1.0)
        self.declare_parameter('replan_inflate_step', 0.03)   # how much to widen each time path is invalid
        self.declare_parameter('max_inflate_radius', 2.0)     # safety cap

        # Getting the parameters
        self.planner_name = self.get_parameter('planner_name').get_parameter_value().string_value
        self.path_topic = self.get_parameter('path_topic').get_parameter_value().string_value
        self.env_file = self.get_parameter('env_file').get_parameter_value().string_value

        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        self.resolution = float(self.get_parameter('resolution').value)
        self._inflate_param = float(self.get_parameter('inflate_radius').value)
        self.extra_clearance = float(self.get_parameter('extra_clearance').value)
        self.eight_connected = bool(self.get_parameter('eight_connected').value)
        self.interpolate_ds = float(self.get_parameter('interpolate_ds').value)
        self.smooth = bool(self.get_parameter('smooth').value)

        self.auto_replan = bool(self.get_parameter('auto_replan_on_invalid').value)
        self.replan_cooldown = float(self.get_parameter('replan_cooldown_sec').value)
        self.replan_inflate_step = float(self.get_parameter('replan_inflate_step').value)
        self.max_inflate_radius = float(self.get_parameter('max_inflate_radius').value)

        # Loading the YAML
        pkg_share = get_package_share_directory('path_planner_node')
        env_path = f"{pkg_share}/config/{self.env_file}"
        with open(env_path, 'r') as f:
            self.env_config = yaml.safe_load(f)

        self.start = [float(x) for x in self.env_config.get('start', [0.0, 0.0])]
        self.goal = [float(x) for x in self.env_config.get('goal', [10.0, 10.0])]
        self.obstacles = [list(map(float, o)) for o in self.env_config.get('obstacles', [])]  # [cx,cy,w,h]
        self.env_name = self.env_config.get('name', 'default_env')
        self.robot_radius = float(self.env_config.get('robot_radius', 0.345))
        self.goal_tolerance = float(self.env_config.get('goal_tolerance', 0.5))

        # Inflate radius: explicit param if > 0, else robot_radius + clearance
        self.inflate_radius = self._inflate_param if self._inflate_param > 0.0 else (self.robot_radius + self.extra_clearance)

        # Optional bounds [xmin, ymin, xmax, ymax]; else derive
        yaml_bounds = self.env_config.get('bounds')
        self.bounds = [float(b) for b in yaml_bounds] if yaml_bounds else self._auto_bounds()

        # Grid build
        self.grid = self._make_grid(self.bounds, self.resolution)
        self._rasterize_and_inflate(self.grid, self.obstacles, self.inflate_radius)

        # IO
        self.path_pub = self.create_publisher(Path, self.path_topic, 10)
        self.obstacle_pub = self.create_publisher(MarkerArray, self.env_name, 10)
        self.valid_sub = self.create_subscription(Bool, 'path_valid', self.valid_cb, 10)

        self.timer = self.create_timer(1.0, self.publish_path)
        self.last_replan_time = self.get_clock().now()
        self.pending_replan = False

        self.get_logger().info(
            f"[{self.planner_name}] frame={self.frame_id} res={self.resolution} "
            f"inflation={self.inflate_radius:.3f} bounds={self.bounds} start={self.start} goal={self.goal}"
        )

    # Calling back
    def valid_cb(self, msg: Bool):
        if not self.auto_replan or msg.data:
            return
        now = self.get_clock().now()
        if (now - self.last_replan_time).nanoseconds < int(self.replan_cooldown * 1e9):
            return
        self.last_replan_time = now

        # Increase inflation (wider berth) and rebuild grid
        new_inflation = min(self.max_inflate_radius, self.inflate_radius + self.replan_inflate_step)
        if new_inflation > self.inflate_radius:
            self.inflate_radius = new_inflation
            self.get_logger().warn(f"[planner] Invalid path -> increasing inflation to {self.inflate_radius:.3f} and replanning")
            self.grid = self._make_grid(self.bounds, self.resolution)
            self._rasterize_and_inflate(self.grid, self.obstacles, self.inflate_radius)
            self.pending_replan = True

    # Timer
    def publish_path(self):
        if self.pending_replan:
            self.pending_replan = False
        path_msg = self.compute_path()
        self.path_pub.publish(path_msg)
        self.get_logger().info(f"Published path with {len(path_msg.poses)} poses.")
        self.obstacle_pub.publish(self.get_obstacle_markers())
        self.get_logger().info(f"Published {len(self.obstacles)} obstacle markers.")

    # Grid Utilities
    def _auto_bounds(self):
        xs = [self.start[0], self.goal[0]]
        ys = [self.start[1], self.goal[1]]
        for (cx, cy, w, h) in self.obstacles:
            xs += [cx - w/2, cx + w/2]
            ys += [cy - h/2, cy + h/2]
        margin = 1.0
        xmin, xmax = min(xs) - margin, max(xs) + margin
        ymin, ymax = min(ys) - margin, max(ys) + margin
        if xmax - xmin < 2 * self.resolution: xmax = xmin + 2 * self.resolution
        if ymax - ymin < 2 * self.resolution: ymax = ymin + 2 * self.resolution
        return [xmin, ymin, xmax, ymax]

    def _make_grid(self, bounds, res):
        xmin, ymin, xmax, ymax = bounds
        nx = max(1, int(math.ceil((xmax - xmin) / res)))
        ny = max(1, int(math.ceil((ymax - ymin) / res)))
        data = [[0 for _ in range(nx)] for _ in range(ny)]  # 0=free,1=occ
        return {"xmin": xmin, "ymin": ymin, "xmax": xmax, "ymax": ymax,
                "nx": nx, "ny": ny, "res": res, "data": data}

    def _world_to_grid(self, x, y):
        gx = int((x - self.grid["xmin"]) / self.grid["res"])
        gy = int((y - self.grid["ymin"]) / self.grid["res"])
        return gx, gy

    def _grid_to_world(self, gx, gy):
        x = self.grid["xmin"] + (gx + 0.5) * self.grid["res"]
        y = self.grid["ymin"] + (gy + 0.5) * self.grid["res"]
        return [x, y]

    def _in_bounds(self, gx, gy):
        return 0 <= gx < self.grid["nx"] and 0 <= gy < self.grid["ny"]

    def _set_occ_rect(self, cx, cy, w, h):
        x0, x1 = cx - w/2.0, cx + w/2.0
        y0, y1 = cy - h/2.0, cy + h/2.0
        gx0, gy0 = self._world_to_grid(x0, y0)
        gx1, gy1 = self._world_to_grid(x1, y1)
        gx0, gx1 = sorted((gx0, gx1))
        gy0, gy1 = sorted((gy0, gy1))
        for gy in range(max(0, gy0), min(self.grid["ny"], gy1 + 1)):
            for gx in range(max(0, gx0), min(self.grid["nx"], gx1 + 1)):
                self.grid["data"][gy][gx] = 1

    def _inflate(self, radius):
        cells = max(1, int(math.ceil(radius / self.grid["res"])))
        ny, nx = self.grid["ny"], self.grid["nx"]
        src = self.grid["data"]
        dst = [[0 for _ in range(nx)] for _ in range(ny)]
        disk = [(dx, dy) for dy in range(-cells, cells+1) for dx in range(-cells, cells+1)
                if dx*dx + dy*dy <= cells*cells]
        for gy in range(ny):
            for gx in range(nx):
                if src[gy][gx] == 1:
                    for dx, dy in disk:
                        gx2, gy2 = gx + dx, gy + dy
                        if 0 <= gx2 < nx and 0 <= gy2 < ny:
                            dst[gy2][gx2] = 1
        self.grid["data"] = dst

    def _rasterize_and_inflate(self, grid, obstacles, inflate_radius):
        for (cx, cy, w, h) in obstacles:
            self._set_occ_rect(cx, cy, w, h)
        if inflate_radius > 0.0:
            self._inflate(inflate_radius)

    # A*
    def _neighbors(self, gx, gy):
        steps = [(-1,0),(1,0),(0,-1),(0,1)]
        if self.eight_connected:
            steps += [(-1,-1),(-1,1),(1,-1),(1,1)]
        for dx, dy in steps:
            x2, y2 = gx + dx, gy + dy
            if not self._in_bounds(x2, y2): continue
            if self.grid["data"][y2][x2] != 0: continue
            yield x2, y2, math.hypot(dx, dy)

    @staticmethod
    def _heuristic(a, b):
        ax, ay = a; bx, by = b
        return math.hypot(ax - bx, ay - by)

    def _astar(self, s_g, g_g):
        sx, sy = s_g; gx, gy = g_g
        if not self._in_bounds(sx, sy) or not self._in_bounds(gx, gy): return None
        if self.grid["data"][sy][sx] != 0 or self.grid["data"][gy][gx] != 0: return None

        open_heap = []
        heapq.heappush(open_heap, (0.0, (sx, sy)))
        g_cost = {(sx, sy): 0.0}
        parent = {(sx, sy): None}

        while open_heap:
            _, node = heapq.heappop(open_heap)
            if node == (gx, gy):
                path = []
                cur = node
                while cur is not None:
                    path.append(cur)
                    cur = parent[cur]
                return list(reversed(path))
            for nx, ny, step in self._neighbors(*node):
                cand = g_cost[node] + step
                if (nx, ny) not in g_cost or cand < g_cost[(nx, ny)]:
                    g_cost[(nx, ny)] = cand
                    parent[(nx, ny)] = node
                    f = cand + self._heuristic((nx, ny), (gx, gy))
                    heapq.heappush(open_heap, (f, (nx, ny)))
        return None

    def _line_free(self, p0, p1):
        (x0, y0), (x1, y1) = p0, p1
        gx0, gy0 = self._world_to_grid(x0, y0)
        gx1, gy1 = self._world_to_grid(x1, y1)
        dx = abs(gx1 - gx0); sx = 1 if gx0 < gx1 else -1
        dy = -abs(gy1 - gy0); sy = 1 if gy0 < gy1 else -1
        err = dx + dy; x, y = gx0, gy0
        while True:
            if not self._in_bounds(x, y) or self.grid["data"][y][x] != 0: return False
            if x == gx1 and y == gy1: break
            e2 = 2 * err
            if e2 >= dy: err += dy; x += sx
            if e2 <= dx: err += dx; y += sy
        return True

    def _smooth_path(self, pts):
        if len(pts) <= 2: return pts
        out = [pts[0]]; i = 0
        while i < len(pts) - 1:
            j = len(pts) - 1
            while j > i + 1 and not self._line_free(pts[i], pts[j]):
                j -= 1
            out.append(pts[j]); i = j
        return out

    @staticmethod
    def _interpolate_path(pts, ds):
        if not pts: return []
        out = [pts[0]]
        for k in range(1, len(pts)):
            x0, y0 = out[-1]; x1, y1 = pts[k]
            seg = math.hypot(x1 - x0, y1 - y0)
            if seg < 1e-9: continue
            u = 0.0
            while u + ds <= seg:
                u += ds; t = u / seg
                out.append([x0 + t*(x1 - x0), y0 + t*(y1 - y0)])
            if math.hypot(out[-1][0] - x1, out[-1][1] - y1) > 1e-6:
                out.append([x1, y1])
        return out

    # Path Computation
    def generate_path(self):
        sg = self._world_to_grid(*self.start)
        gg = self._world_to_grid(*self.goal)
        grid_path = self._astar(sg, gg)

        if not grid_path:
            self.get_logger().warn("A* failed; trying straight-line fallback.")
            world_pts = [self.start, self.goal] if self._line_free(self.start, self.goal) else [self.start]
        else:
            world_pts = [self._grid_to_world(gx, gy) for (gx, gy) in grid_path]
            if self.smooth:
                world_pts = self._smooth_path(world_pts)
        
            if self._line_free(self.start, world_pts[0]):
                world_pts[0] = list(self.start)
            else:
                world_pts = [list(self.start)] + world_pts
            if self._line_free(world_pts[-1], self.goal):
                world_pts[-1] = list(self.goal)
            else:
                world_pts = world_pts + [list(self.goal)]

        world_pts = self._interpolate_path(world_pts, self.interpolate_ds)
        for p in world_pts:
            yield p

    def compute_path(self):
        stamp = self.get_clock().now().to_msg()
        def to_pose(p):
            ps = PoseStamped()
            ps.header.frame_id = self.frame_id
            ps.header.stamp = stamp
            ps.pose.position.x = float(p[0])
            ps.pose.position.y = float(p[1])
            ps.pose.orientation.w = 1.0
            return ps
        path = Path()
        path.header.frame_id = self.frame_id
        path.header.stamp = stamp
        for p in self.generate_path():
            path.poses.append(to_pose(p))
        return path

    # Visualization
    def get_obstacle_markers(self):
        ma = MarkerArray()
        for i, (x, y, w, h) in enumerate(self.obstacles):
            m = Marker()
            m.header.frame_id = self.frame_id
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns = "obstacles"; m.id = i
            m.type = Marker.CUBE; m.action = Marker.ADD
            m.pose.position.x = x; m.pose.position.y = y; m.pose.position.z = 0.5
            m.pose.orientation.w = 1.0
            m.scale.x = w; m.scale.y = h; m.scale.z = max(w, h)
            m.color.a = 1.0; m.color.r = 1.0; m.color.g = 0.0; m.color.b = 1.0
            ma.markers.append(m)
        return ma


def main(args=None):
    rclpy.init(args=args)
    node = PathPlannerPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
