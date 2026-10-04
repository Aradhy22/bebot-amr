#!/usr/bin/env python3
"""Loop-closure backend layered on top of FAST-LIO.

FAST-LIO is pure lidar-inertial ODOMETRY: it has no mechanism to recognize
a previously-visited place and correct accumulated drift, so revisiting an
area (or a lot of in-place turning) lets small per-scan errors accumulate
until the same real geometry gets registered twice at a drifted pose -
visible as a duplicated, misaligned patch in the map.

This node is a read-only consumer of FAST-LIO's /Odometry and
/cloud_registered_body topics - it does not modify FAST-LIO itself. It
builds a sparse pose graph from keyframes, detects loop closures by
proximity + ICP verification (proximity alone risks false matches between
visually-similar aisles), and re-optimizes the trajectory (GTSAM) whenever
one is confirmed, publishing a corrected path/map and a map->camera_init
TF correction on top of FAST-LIO's raw (uncorrected) output.
"""

import math
import os

import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from geometry_msgs.msg import TransformStamped, PoseStamped
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_srvs.srv import Trigger
from tf2_ros import TransformBroadcaster

import gtsam

from bebot_slam.icp import icp
from bebot_slam.transform_utils import matrix_to_pose, voxel_downsample, write_pcd_ascii


def _pose3_from_xyzw(position, quaternion_xyzw):
    x, y, z, w = quaternion_xyzw
    return gtsam.Pose3(gtsam.Rot3.Quaternion(w, x, y, z), gtsam.Point3(*position))


class Keyframe:
    __slots__ = ('index', 'raw_position', 'raw_quaternion', 'points')

    def __init__(self, index, raw_position, raw_quaternion, points):
        self.index = index
        self.raw_position = raw_position      # (x,y,z), FAST-LIO's raw odometry
        self.raw_quaternion = raw_quaternion  # (x,y,z,w), FAST-LIO's raw odometry
        self.points = points                  # Nx3 numpy, body frame, voxel-downsampled


class LoopClosureNode(Node):

    def __init__(self):
        super().__init__('loop_closure_node')

        self.declare_parameter('keyframe_dist_thresh', 0.5)
        self.declare_parameter('keyframe_angle_thresh_deg', 15.0)
        self.declare_parameter('loop_search_radius', 3.0)
        self.declare_parameter('loop_min_keyframe_gap', 20)
        self.declare_parameter('keyframe_voxel_size', 0.2)
        self.declare_parameter('icp_max_correspondence_dist', 1.0)
        self.declare_parameter('icp_fitness_thresh', 0.3)
        self.declare_parameter('icp_min_inlier_ratio', 0.6)
        self.declare_parameter('map_voxel_size', 0.1)
        self.declare_parameter('map_publish_period_sec', 2.0)
        self.declare_parameter('correction_publish_hz', 10.0)
        self.declare_parameter('map_save_path', os.path.expanduser('~/bebot_maps/bebot_map_loop_closed.pcd'))
        self.declare_parameter('odom_frame_id', 'camera_init')
        self.declare_parameter('map_frame_id', 'map')

        self.keyframe_dist_thresh = self.get_parameter('keyframe_dist_thresh').value
        self.keyframe_angle_thresh = math.radians(self.get_parameter('keyframe_angle_thresh_deg').value)
        self.loop_search_radius = self.get_parameter('loop_search_radius').value
        self.loop_min_keyframe_gap = self.get_parameter('loop_min_keyframe_gap').value
        self.keyframe_voxel_size = self.get_parameter('keyframe_voxel_size').value
        self.icp_max_correspondence_dist = self.get_parameter('icp_max_correspondence_dist').value
        self.icp_fitness_thresh = self.get_parameter('icp_fitness_thresh').value
        self.icp_min_inlier_ratio = self.get_parameter('icp_min_inlier_ratio').value
        self.map_voxel_size = self.get_parameter('map_voxel_size').value
        self.map_publish_period_sec = self.get_parameter('map_publish_period_sec').value
        self.map_save_path = self.get_parameter('map_save_path').value
        self.odom_frame_id = self.get_parameter('odom_frame_id').value
        self.map_frame_id = self.get_parameter('map_frame_id').value

        os.makedirs(os.path.dirname(self.map_save_path), exist_ok=True)

        self.keyframes = []
        self.optimized_positions = np.zeros((0, 3))  # proximity search cache, synced with self.keyframes

        self.graph = gtsam.NonlinearFactorGraph()
        self.initial_values = gtsam.Values()
        self.optimized_values = None
        self._last_map_points = None

        self.latest_cloud_points = None
        self.last_keyframe_position = None
        self.last_keyframe_quaternion = None
        self._last_optimize_time = 0.0
        self._keyframes_since_optimize = 0

        self.loop_closure_count = 0
        self.correction_matrix = np.eye(4)  # map -> camera_init

        self.cloud_sub = self.create_subscription(
            PointCloud2, '/cloud_registered_body', self.cloud_callback, 10)
        self.odom_sub = self.create_subscription(
            Odometry, '/Odometry', self.odom_callback, 10)

        self.path_pub = self.create_publisher(Path, '/loop_closure/optimized_path', 10)
        self.map_pub = self.create_publisher(PointCloud2, '/loop_closure/optimized_map', 1)

        self.tf_broadcaster = TransformBroadcaster(self)
        correction_hz = self.get_parameter('correction_publish_hz').value
        self.correction_timer = self.create_timer(1.0 / correction_hz, self.publish_correction_tf)

        self.save_srv = self.create_service(Trigger, '/loop_closure/save_map', self.save_map_callback)

        self.get_logger().info(
            f'Loop closure node up. keyframe_dist={self.keyframe_dist_thresh}m, '
            f'search_radius={self.loop_search_radius}m, map save path={self.map_save_path}')

    # ------------------------------------------------------------------
    def cloud_callback(self, msg: PointCloud2):
        pts = point_cloud2.read_points_numpy(msg, field_names=('x', 'y', 'z'), skip_nans=True)
        if pts.size == 0:
            return
        pts = np.asarray(pts, dtype=np.float64).reshape(-1, 3)
        # skip_nans=True only drops NaN, not the Inf values a lidar reports
        # for "no return" beyond max range - either would crash the KD-tree
        # used in ICP later, so filter both here at ingestion.
        pts = pts[np.isfinite(pts).all(axis=1)]
        if pts.size == 0:
            return
        self.latest_cloud_points = pts

    def odom_callback(self, msg: Odometry):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        position = (p.x, p.y, p.z)
        quaternion = (q.x, q.y, q.z, q.w)

        if self.last_keyframe_position is None:
            self._add_keyframe(position, quaternion)
            return

        dx = position[0] - self.last_keyframe_position[0]
        dy = position[1] - self.last_keyframe_position[1]
        dz = position[2] - self.last_keyframe_position[2]
        dist = math.sqrt(dx * dx + dy * dy + dz * dz)
        angle = self._quat_angle_diff(self.last_keyframe_quaternion, quaternion)

        if dist >= self.keyframe_dist_thresh or angle >= self.keyframe_angle_thresh:
            self._add_keyframe(position, quaternion)

    @staticmethod
    def _quat_angle_diff(q1, q2):
        dot = abs(q1[0] * q2[0] + q1[1] * q2[1] + q1[2] * q2[2] + q1[3] * q2[3])
        dot = min(1.0, dot)
        return 2.0 * math.acos(dot)

    # ------------------------------------------------------------------
    def _add_keyframe(self, position, quaternion):
        if self.latest_cloud_points is None:
            return
        points = voxel_downsample(self.latest_cloud_points, self.keyframe_voxel_size)
        if len(points) < 50:
            return  # not enough geometry here to ever loop-close against reliably

        idx = len(self.keyframes)
        kf = Keyframe(idx, position, quaternion, points)
        self.keyframes.append(kf)
        self.last_keyframe_position = position
        self.last_keyframe_quaternion = quaternion

        pose = _pose3_from_xyzw(position, quaternion)
        self.initial_values.insert(idx, pose)

        if idx == 0:
            prior_noise = gtsam.noiseModel.Diagonal.Sigmas(np.array([1e-6] * 6))
            self.graph.add(gtsam.PriorFactorPose3(0, pose, prior_noise))
        else:
            prev_kf = self.keyframes[idx - 1]
            prev_pose = _pose3_from_xyzw(prev_kf.raw_position, prev_kf.raw_quaternion)
            relative = prev_pose.between(pose)
            odom_noise = gtsam.noiseModel.Diagonal.Sigmas(np.array([0.05, 0.05, 0.05, 0.1, 0.1, 0.1]))
            self.graph.add(gtsam.BetweenFactorPose3(idx - 1, idx, relative, odom_noise))

        self.optimized_positions = np.vstack([self.optimized_positions, np.array(position)])

        closed = self._try_loop_closure(idx)
        self._keyframes_since_optimize += 1

        # Time-based, not keyframe-count-based: gives smooth, progressive
        # "the map is building" feedback regardless of how fast/slow the
        # robot is driven, rather than going dark for however many meters
        # it takes to rack up a fixed keyframe count. Always optimize
        # immediately on the very first keyframe (instant initial feedback)
        # and on any accepted loop closure (don't sit on a correction).
        now = self.get_clock().now().nanoseconds / 1e9
        due = (now - self._last_optimize_time) >= self.map_publish_period_sec
        if closed or idx == 0 or (due and self._keyframes_since_optimize > 0):
            self._optimize()
            self._last_optimize_time = now
            self._keyframes_since_optimize = 0

    # ------------------------------------------------------------------
    def _try_loop_closure(self, new_idx) -> bool:
        if new_idx < self.loop_min_keyframe_gap:
            return False

        candidate_positions = self.optimized_positions[:new_idx - self.loop_min_keyframe_gap + 1]
        if len(candidate_positions) == 0:
            return False

        new_pos = self.optimized_positions[new_idx]
        dists = np.linalg.norm(candidate_positions - new_pos, axis=1)
        best_idx = int(np.argmin(dists))
        if dists[best_idx] > self.loop_search_radius:
            return False

        source_kf = self.keyframes[new_idx]
        target_kf = self.keyframes[best_idx]

        source_pose = _pose3_from_xyzw(source_kf.raw_position, source_kf.raw_quaternion)
        target_pose = _pose3_from_xyzw(target_kf.raw_position, target_kf.raw_quaternion)
        init_relative = target_pose.between(source_pose)

        R, t, fitness, inlier_ratio = icp(
            source_kf.points, target_kf.points,
            init_relative.rotation().matrix(), np.array(init_relative.translation()),
            max_correspondence_dist=self.icp_max_correspondence_dist)

        if fitness > self.icp_fitness_thresh or inlier_ratio < self.icp_min_inlier_ratio:
            return False

        refined_relative = gtsam.Pose3(gtsam.Rot3(R), gtsam.Point3(*t))
        loop_noise = gtsam.noiseModel.Diagonal.Sigmas(np.array([0.01, 0.01, 0.01, 0.02, 0.02, 0.02]))
        self.graph.add(gtsam.BetweenFactorPose3(best_idx, new_idx, refined_relative, loop_noise))

        self.loop_closure_count += 1
        self.get_logger().info(
            f'Loop closure #{self.loop_closure_count}: keyframe {new_idx} <-> {best_idx} '
            f'(dist={dists[best_idx]:.2f}m, fitness={fitness:.3f}, inliers={inlier_ratio:.2f})')
        return True

    # ------------------------------------------------------------------
    def _optimize(self):
        if len(self.keyframes) == 0:
            return
        optimizer = gtsam.LevenbergMarquardtOptimizer(self.graph, self.initial_values)
        result = optimizer.optimize()
        self.optimized_values = result

        positions = np.zeros((len(self.keyframes), 3))
        for i in range(len(self.keyframes)):
            positions[i] = result.atPose3(i).translation()
        self.optimized_positions = positions

        # map -> camera_init correction: newest keyframe's optimized pose
        # vs. its raw FAST-LIO odometry pose.
        last_idx = len(self.keyframes) - 1
        optimized_last = result.atPose3(last_idx)
        raw_last_kf = self.keyframes[last_idx]
        raw_last = _pose3_from_xyzw(raw_last_kf.raw_position, raw_last_kf.raw_quaternion)
        correction_pose = optimized_last.compose(raw_last.inverse())
        self.correction_matrix = correction_pose.matrix()

        self._publish_path(result)
        self._publish_map(result)

    # ------------------------------------------------------------------
    def _publish_path(self, result):
        # Published in map (not camera_init) frame: this is the globally
        # corrected estimate. RViz/consumers get FAST-LIO's raw camera_init-
        # frame output correctly overlaid automatically via the published
        # map->camera_init correction TF, same convention as map->odom in a
        # standard SLAM stack.
        path = Path()
        path.header = self._header(self.map_frame_id)
        for i in range(len(self.keyframes)):
            pose = result.atPose3(i)
            ps = PoseStamped()
            ps.header = path.header
            t = pose.translation()
            ps.pose.position.x, ps.pose.position.y, ps.pose.position.z = float(t[0]), float(t[1]), float(t[2])
            q = pose.rotation().toQuaternion()
            ps.pose.orientation.x, ps.pose.orientation.y = q.x(), q.y()
            ps.pose.orientation.z, ps.pose.orientation.w = q.z(), q.w()
            path.poses.append(ps)
        self.path_pub.publish(path)

    def _publish_map(self, result):
        all_points = []
        for i, kf in enumerate(self.keyframes):
            T = result.atPose3(i).matrix()
            pts_h = np.hstack([kf.points, np.ones((len(kf.points), 1))])
            world_pts = (T @ pts_h.T).T[:, :3]
            all_points.append(world_pts)
        if not all_points:
            return
        merged = voxel_downsample(np.vstack(all_points), self.map_voxel_size)
        self._last_map_points = merged

        msg = point_cloud2.create_cloud_xyz32(self._header(self.map_frame_id), merged.tolist())
        self.map_pub.publish(msg)

    def _header(self, frame_id):
        h = Header()
        h.frame_id = frame_id
        h.stamp = self.get_clock().now().to_msg()
        return h

    # ------------------------------------------------------------------
    def publish_correction_tf(self):
        if len(self.keyframes) == 0:
            return
        position, quaternion = matrix_to_pose(self.correction_matrix)
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.map_frame_id
        t.child_frame_id = self.odom_frame_id
        t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = position
        t.transform.rotation.x, t.transform.rotation.y = quaternion[0], quaternion[1]
        t.transform.rotation.z, t.transform.rotation.w = quaternion[2], quaternion[3]
        self.tf_broadcaster.sendTransform(t)

    # ------------------------------------------------------------------
    def save_map_callback(self, request, response):
        if self._last_map_points is None and self.optimized_values is not None:
            self._publish_map(self.optimized_values)
        if self._last_map_points is None:
            response.success = False
            response.message = 'No map accumulated yet.'
            return response
        write_pcd_ascii(self.map_save_path, self._last_map_points)
        response.success = True
        response.message = (
            f'Saved {len(self._last_map_points)} points to {self.map_save_path} '
            f'({self.loop_closure_count} loop closures applied)')
        self.get_logger().info(response.message)
        return response


def main(args=None):
    rclpy.init(args=args)
    node = LoopClosureNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
