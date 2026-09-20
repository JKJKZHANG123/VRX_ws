#!/usr/bin/env python3
"""
Costmap -> 3D obstacle markers (visualization only).

The OccupancyGrid displays in RViz only draw a flat grid on the ground, which
reads poorly next to the dense PCL/Point-LIO point cloud. This node lifts the
occupied cells of the local costmap into 3D boxes (markers standing above the
water), so buoys / shoreline / floating obstacles are visible as solid blocks
that the green global plan clearly routes around.

  in : /local_costmap/costmap    (nav_msgs/OccupancyGrid, TRANSIENT_LOCAL)
  out: /obstacle_3d/markers      (visualization_msgs/MarkerArray, frame camera_init)

Pure visualization - no effect on planning/collision.
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import Marker, MarkerArray


class Costmap3dMarkers(Node):
    def __init__(self):
        super().__init__('costmap_3d_markers')

        self.declare_parameter('costmap_topic', '/local_costmap/costmap')
        self.declare_parameter('occ_threshold', 100)   # show lethal cells only; inflation is not geometry
        self.declare_parameter('block_height', 2.0)    # box height (m)
        self.declare_parameter('block_color_r', 1.0)   # red-ish
        self.declare_parameter('block_color_g', 0.3)
        self.declare_parameter('block_color_b', 0.2)
        self.declare_parameter('block_alpha', 0.7)
        self.declare_parameter('use_world_frame', 'camera_init')

        self.occ_thr = self.get_parameter('occ_threshold').value
        self.h = self.get_parameter('block_height').value
        self.cr = self.get_parameter('block_color_r').value
        self.cg = self.get_parameter('block_color_g').value
        self.cb = self.get_parameter('block_color_b').value
        self.ca = self.get_parameter('block_alpha').value
        self.frame = self.get_parameter('use_world_frame').value

        # costmap is TRANSIENT_LOCAL + RELIABLE
        q = QoSProfile(depth=1)
        q.reliability = ReliabilityPolicy.RELIABLE
        q.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self.sub = self.create_subscription(
            OccupancyGrid, self.get_parameter('costmap_topic').value,
            self._on_costmap, q)
        self.pub = self.create_publisher(
            MarkerArray, '/obstacle_3d/markers', 10)

        self.get_logger().info('costmap_3d_markers up (visualization only)')

    def _on_costmap(self, msg: OccupancyGrid):
        res = msg.info.resolution
        w, h_g = msg.info.width, msg.info.height
        ox, oy = msg.info.origin.position.x, msg.info.origin.position.y
        data = msg.data

        markers = []
        # group: obstacle blocks (id 0..N); delete: reuse ns='obst'
        # We rebuild the full array each frame for simplicity at 30x30 grids.
        marker = Marker()
        marker.header.frame_id = self.frame
        marker.header.stamp = msg.header.stamp
        marker.ns = 'obstacles'
        marker.action = Marker.ADD
        marker.type = Marker.CUBE_LIST      # one cube per occupied cell in one marker
        marker.scale.x = res * 0.85         # boxes slightly smaller than cell
        marker.scale.y = res * 0.85
        marker.scale.z = self.h
        marker.color.r, marker.color.g, marker.color.b = self.cr, self.cg, self.cb
        marker.color.a = self.ca
        # Do not let an old visualization survive if the costmap stops updating.
        # The marker is recreated on every costmap callback.
        marker.lifetime.sec = 1
        marker.pose.orientation.w = 1.0
        marker.id = 0

        # cube center: raised by half the block height above the water (z=0)
        zc = self.h / 2.0

        n = 0
        pts = []
        for j in range(h_g):
            for i in range(w):
                cell = data[j * w + i]
                if cell >= self.occ_thr:
                    pts.append((ox + (i + 0.5) * res,
                                oy + (j + 0.5) * res,
                                zc))
                    n += 1

        if n == 0:
            # publish a delete-all so stale blocks disappear
            m = Marker()
            m.header.frame_id = self.frame
            m.header.stamp = msg.header.stamp
            m.ns = 'obstacles'
            m.action = Marker.DELETEALL
            markers.append(m)
        else:
            from geometry_msgs.msg import Point
            marker.points = [Point(x=px, y=py, z=pz) for px, py, pz in pts]
            markers.append(marker)

        self.pub.publish(MarkerArray(markers=markers))
        self.get_logger().debug(f'published {n} obstacle cubes', throttle_duration_sec=3.0)


def main(args=None):
    rclpy.init(args=args)
    node = Costmap3dMarkers()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()