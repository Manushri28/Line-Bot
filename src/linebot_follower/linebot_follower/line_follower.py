#!/usr/bin/env python3
import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import Image


def to_gray(msg: Image) -> np.ndarray:
    """sensor_msgs/Image (rgb8/bgr8/mono8) -> grayscale uint8, no cv_bridge needed."""
    h, w = msg.height, msg.width
    ch = 3 if msg.encoding in ('rgb8', 'bgr8') else 1
    buf = np.frombuffer(msg.data, dtype=np.uint8).reshape(h, msg.step)
    img = buf[:, :w * ch].reshape(h, w, ch)
    if ch == 3:
        code = cv2.COLOR_RGB2GRAY if msg.encoding == 'rgb8' else cv2.COLOR_BGR2GRAY
        return cv2.cvtColor(np.ascontiguousarray(img), code)
    return np.ascontiguousarray(img[:, :, 0])


class LineFollower(Node):
    def __init__(self):
        super().__init__('line_follower')
        defaults = dict(kp=1.6, ki=0.0, kd=0.25, v_max=0.18, v_min=0.05,
                        slow_gain=0.8, w_max=2.0, threshold=80, alpha=0.6,
                        min_pixels=30, search_w=0.8, near_weight=0.6)
        for k, v in defaults.items():
            self.declare_parameter(k, v)

        # Camera is the ONLY input
        self.create_subscription(Image, '/camera/image_raw', self.on_image, 1)
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.dbg = self.create_publisher(Image, '/line_debug', 1)

        self.e_f = 0.0        # filtered error
        self.last_e = 0.0     # remembered for line-lost recovery
        self.integral = 0.0
        self.last_t = None
        self.get_logger().info('line_follower node started')

    def p(self, name):
        return self.get_parameter(name).value

    def band_centroid(self, mask, r0, r1):
        m = cv2.moments(mask[r0:r1, :], binaryImage=True)
        if m['m00'] < self.p('min_pixels'):
            return None
        return m['m10'] / m['m00']

    def on_image(self, msg: Image):
        gray = to_gray(msg)
        h, w = gray.shape
        mask = (gray < self.p('threshold')).astype(np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))

        near = self.band_centroid(mask, int(0.70 * h), h)
        far = self.band_centroid(mask, int(0.40 * h), int(0.70 * h))

        cmd = Twist()
        if near is None and far is None:
            # Line lost: rotate toward the side it was last seen
            sign = 1.0 if self.last_e >= 0 else -1.0
            cmd.linear.x = 0.02
            cmd.angular.z = -sign * self.p('search_w')
            self.pub.publish(cmd)
            self.publish_debug(gray, mask, None)
            return

        nw = self.p('near_weight')
        if near is None:
            cx = far
        elif far is None:
            cx = near
        else:
            cx = nw * near + (1.0 - nw) * far

        # Error in -1 (line left) .. +1 (line right)
        e = (cx - w / 2.0) / (w / 2.0)
        a = self.p('alpha')
        e_prev = self.e_f
        self.e_f = a * e + (1.0 - a) * self.e_f
        self.last_e = self.e_f

        # PID
        t = self.get_clock().now().nanoseconds * 1e-9
        dt = 0.033 if self.last_t is None else max(t - self.last_t, 1e-3)
        self.last_t = t
        self.integral = float(np.clip(self.integral + self.e_f * dt, -1.0, 1.0))
        d = (self.e_f - e_prev) / dt
        u = self.p('kp') * self.e_f + self.p('ki') * self.integral + self.p('kd') * d

        wmax = self.p('w_max')
        # line on the right (e > 0) -> turn right -> negative angular z
        cmd.angular.z = float(np.clip(-u, -wmax, wmax))
        v = self.p('v_max') * (1.0 - self.p('slow_gain') * min(abs(self.e_f), 1.0))
        cmd.linear.x = float(np.clip(v, self.p('v_min'), self.p('v_max')))
        self.pub.publish(cmd)
        self.publish_debug(gray, mask, cx)

    def publish_debug(self, gray, mask, cx):
        vis = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
        vis[mask > 0] = (0, 255, 0)
        h, w = gray.shape
        cv2.line(vis, (w // 2, 0), (w // 2, h), (255, 0, 0), 1)
        if cx is not None:
            cv2.circle(vis, (int(cx), int(0.8 * h)), 5, (0, 0, 255), -1)
        out = Image()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = 'camera_link'
        out.height, out.width = vis.shape[:2]
        out.encoding = 'bgr8'
        out.step = out.width * 3
        out.data = vis.tobytes()
        self.dbg.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = LineFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()