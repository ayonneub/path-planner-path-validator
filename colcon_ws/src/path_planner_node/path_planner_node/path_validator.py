import math
import yaml
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from ament_index_python.packages import get_package_share_directory


class PathValidatorPublisher(Node):
    """
    Grid-based path validator:
      - Rebuilds an occupancy grid from env YAML.
      - Inflates obstacles by a slightly *smaller* radius than the planner
        (to avoid edge-touch false positives).
      - For each path segment, checks line-of-sight on the inflated grid.
      - Publishes 'path_valid' (std_msgs/Bool) and RViz debug markers.
    """

    def __init__(self):
        super().__init__('path_validator')

        # parameters
        self.declare_parameter('planner_name', 'A*')
        self.declare_parameter('path_topic', 'planned_path')
        self.declare_parameter('env_file', 'env1.yaml')

        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('max_step', 2.0)          # max spacing allowed between poses (m)

        # Start/goal adherence
        self.declare_parameter('start_goal_slack', 0.25) # extra meters beyond goal_tolerance
        self.declare_parameter('enforce_start_goal', True)

        # Grid used for validation (match planner resolution)
        self.declare_parameter('resolution', 0.10)

        # Inflate radius used by validator:
        # If 0.0 -> used robot_radius from YAML.
        # than the planner's radius by adding a deflate_margin.
        self.declare_parameter('validate_inflate_radius', 0.0)
        self.declare_parameter('deflate_margin', 0.02)   # meters

        # Get Parameter
        self.planner_name = self.get_parameter('planner_name').get_parameter_value().string_value
        self.path_topic = self.get_parameter('path_topic').get_parameter_value().string_value
        self.env_file = self.get_parameter('env_file').get_parameter_value().string_value

        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        self.max_step = float(self.get_parameter('max_step').value)

        self.start_goal_slack = float(self.get_parameter('start_goal_slack').value)
        self.enforce_start_goal = bool(self.get_parameter('enforce_start_goal').value)
        self.resolution = float(self.get_parameter('resolution').value)
        self.validate_inflate_radius_param = float(self.get_parameter('validate_inflate_radius').value)
        self.deflate_margin = float(self.get_parameter('deflate_margin').value)

        # Load Environment
        pkg_share = get_package_share_directory('path_planner_node')
        env_path = f"{pkg_share}/config/{self.env_file}"
        with open(env_path, 'r') as f:
            self.env_config = yaml.safe_load(f)

        self.start = [float(x) for x in self.env_config.get('start', [0.0, 0.0])]
        self.goal = [float(x) for x in self.env_config.get('goal', [10.0, 10.0])]
        # obstacles: [cx, cy, w, h], center-based
        self.obstacles = [list(map(float, o)) for o in self.env_config.get('obstacles', [])]
        self.env_name = self.env_config.get('name', 'default_env')
        self.robot_radius = float(self.env_config.get('robot_radius', 0.345))
        self.goal_tolerance = float(self.env_config.get('goal_tolerance', 0.5))

        # Use robot_radius when user doesn't override validate_inflate_radius
        base_inflate = self.validate_inflate_radius_param if self.validate_inflate_radius_param > 0.0 else self.robot_radius
        self.validate_inflate_radius = max(0.0, base_inflate - max(0.0, self.deflate_margin))

        # Build bounds and grid
        bounds = self.env_config.get('bounds')
        self.bounds = [float(b) for b in bounds] if bounds else self._auto_bounds()
        self.grid = self._make_grid(self.bounds, self.resolution)
        self._rasterize_and_inflate(self.grid, self.obstacles, self.validate_inflate_radius)

        # ---------- IO ----------
        self.viz_path_pub = self.create_publisher(MarkerArray, self.env_name, 10)
        self.debug_pub = self.create_publisher(MarkerArray, f"{self.env_name}/validation_debug", 10)
        self.valid_pub = self.create_publisher(Bool, 'path_valid', 10)

        self.path_sub = self.create_subscription(Path, self.path_topic, self.path_callback, 10)

        self.get_logger().info(
            f"[validator] frame={self.frame_id} res={self.resolution} "
            f"inflate={self.validate_inflate_radius:.3f} (base={base_inflate:.3f}, deflate={self.deflate_margin:.3f}) "
            f"bounds={self.bounds} goal_tol={self.goal_tolerance} slack={self.start_goal_slack}"
        )

    # Grid build
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
        data = [[0 for _ in range(nx)] for _ in range(ny)]  # 0=free, 1=occ
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

    # validation
    def path_callback(self, msg: Path):
        self.get_logger().info(f"Received path with {len(msg.poses)} poses.")
        is_valid, reason = self.validate_path(msg)
        self.valid_pub.publish(Bool(data=is_valid))

        color = (0.0, 1.0, 0.0) if is_valid else (1.0, 0.0, 0.0)
        if is_valid:
            self.get_logger().info("Path is valid.")
            # clear any old debug markers
            self.publish_collision_debug(None, None, None, clear_only=True)
        else:
            self.get_logger().warn(f"Path is invalid: {reason}")
        self.viz_path_pub.publish(self.get_path_markers(msg, color=color))

    def validate_path(self, path: Path):
        # Basics
        if not path.poses or len(path.poses) < 2:
            return False, "Path has fewer than 2 poses."
        if path.header.frame_id and path.header.frame_id != self.frame_id:
            return False, f"Frame mismatch: got '{path.header.frame_id}', expected '{self.frame_id}'."

        # Start/Goal adherence (with slack)
        if self.enforce_start_goal:
            p0 = path.poses[0].pose.position
            pN = path.poses[-1].pose.position
            d_start = math.hypot(p0.x - self.start[0], p0.y - self.start[1])
            d_goal  = math.hypot(pN.x - self.goal[0],  pN.y - self.goal[1])
            allowed = self.goal_tolerance + self.start_goal_slack
            if d_start > allowed:
                return False, f"First pose not near start: d={d_start:.3f} m > allowed={allowed:.3f} m."
            if d_goal > allowed:
                return False, f"Last pose not near goal: d={d_goal:.3f} m > allowed={allowed:.3f} m."

        # Segment continuity + collision using the inflated grid
        prev = path.poses[0].pose.position
        for i in range(1, len(path.poses)):
            curr = path.poses[i].pose.position

            seg_len = math.hypot(curr.x - prev.x, curr.y - prev.y)
            if seg_len > self.max_step:
                return False, f"Discontinuous step between poses {i-1}->{i} ({seg_len:.2f} m > {self.max_step} m)."

            free, first_block = self._line_free_grid((prev.x, prev.y), (curr.x, curr.y))
            if not free:
                self.publish_collision_debug(first_block, (prev.x, prev.y), (curr.x, curr.y))
                wx, wy = first_block
                return False, f"Collision on inflated grid near ({wx:.3f}, {wy:.3f}) along segment {i-1}->{i}."
            prev = curr

        return True, "OK"

    # Ray-cast on the inflated grid (Bresenham). Returns (True, _) if clear; else (False, world_point)
    def _line_free_grid(self, p0, p1):
        (x0, y0), (x1, y1) = p0, p1
        gx0, gy0 = self._world_to_grid(x0, y0)
        gx1, gy1 = self._world_to_grid(x1, y1)

        dx = abs(gx1 - gx0)
        sx = 1 if gx0 < gx1 else -1
        dy = -abs(gy1 - gy0)
        sy = 1 if gy0 < gy1 else -1
        err = dx + dy
        x, y = gx0, gy0

        while True:
            if self._in_bounds(x, y) and self.grid["data"][y][x] != 0:
                return False, self._grid_to_world(x, y)
            if x == gx1 and y == gy1:
                break
            e2 = 2 * err
            if e2 >= dy:
                err += dy
                x += sx
            if e2 <= dx:
                err += dx
                y += sy
        return True, None

    # Visualization
    def get_path_markers(self, path: Path, color=(0.0, 1.0, 0.0)):
        ma = MarkerArray()

        line = Marker()
        line.header.frame_id = self.frame_id
        line.header.stamp = self.get_clock().now().to_msg()
        line.ns = "path"; line.id = 3001
        line.type = Marker.LINE_STRIP; line.action = Marker.ADD
        for ps in path.poses:
            line.points.append(Point(x=ps.pose.position.x, y=ps.pose.position.y, z=ps.pose.position.z))
       
        line.scale.x = max(0.01, self.robot_radius)
        line.color.a = 1.0
        line.color.r, line.color.g, line.color.b = color
        ma.markers.append(line)

        # Start / Goal spheres
        ma.markers.append(self._make_sphere(self.start, radius=max(0.03, self.robot_radius*0.6),
                                            mid=3101, ns="endpoints", rgba=(0.2, 0.6, 1.0, 0.9)))
        ma.markers.append(self._make_sphere(self.goal, radius=max(0.03, self.robot_radius*0.6),
                                            mid=3102, ns="endpoints", rgba=(1.0, 0.8, 0.2, 0.9)))
        return ma

    def publish_collision_debug(self, where, seg_p0, seg_p1, clear_only=False):
        ma = MarkerArray()
        if clear_only:
            m = Marker(); m.action = Marker.DELETEALL
            ma.markers.append(m); self.debug_pub.publish(ma); return

        if where is not None:
            sx, sy = where
            m = Marker()
            m.header.frame_id = self.frame_id
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns = "debug_collision"; m.id = 4001
            m.type = Marker.SPHERE; m.action = Marker.ADD
            m.pose.position.x = sx; m.pose.position.y = sy; m.pose.position.z = 0.02
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = m.scale.z = 0.15
            m.color.r, m.color.g, m.color.b, m.color.a = 1.0, 0.0, 0.0, 0.95
            ma.markers.append(m)

        if seg_p0 is not None and seg_p1 is not None:
            p0x, p0y = seg_p0; p1x, p1y = seg_p1
            m = Marker()
            m.header.frame_id = self.frame_id
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns = "debug_collision"; m.id = 4002
            m.type = Marker.LINE_LIST; m.action = Marker.ADD
            m.points = [Point(x=p0x, y=p0y, z=0.02), Point(x=p1x, y=p1y, z=0.02)]
            m.scale.x = 0.06
            m.color.r, m.color.g, m.color.b, m.color.a = 1.0, 1.0, 0.0, 0.95
            ma.markers.append(m)

        self.debug_pub.publish(ma)

    def _make_sphere(self, xy, radius=0.1, mid=0, ns="pts", rgba=(1, 1, 1, 1)):
        m = Marker()
        m.header.frame_id = self.frame_id
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = ns; m.id = mid
        m.type = Marker.SPHERE; m.action = Marker.ADD
        m.pose.position.x = float(xy[0]); m.pose.position.y = float(xy[1]); m.pose.position.z = 0.0
        m.pose.orientation.w = 1.0
        m.scale.x = m.scale.y = m.scale.z = radius * 2.0
        r, g, b, a = rgba
        m.color.r, m.color.g, m.color.b, m.color.a = float(r), float(g), float(b), float(a)
        return m


def main(args=None):
    rclpy.init(args=args)
    node = PathValidatorPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
