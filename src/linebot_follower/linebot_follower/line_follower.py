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
        defaults = dict(
            kp=1.6, ki=0.0, kd=0.25, v_max=0.18, v_min=0.05,
            slow_gain=0.8, w_max=2.0, threshold=80, alpha=0.6,
            min_pixels=30, near_weight=0.6,
            # image bands as fractions of image height (0 = top, 1 = bottom)
            near_top=0.70, near_bot=1.00,
            far_top=0.40, far_bot=0.70,
            # line-lost behaviour
            lost_stop_frames=10,   # frames of creeping on before stopping (~0.3 s at 30 Hz)
            end_slow=0.5,          # speed factor when only the near band sees the line
            search_w=0.0)          # >0 = rotate to search after the stop; 0 = just stop
        for k, v in defaults.items():
            self.declare_parameter(k, v)
 
        # Camera is the ONLY input
        self.create_subscription(Image, '/camera/image_raw', self.on_image, 1)
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.dbg = self.create_publisher(Image, '/line_debug', 1)
 
        self.e_f = 0.0        # filtered error
        self.last_side = 1.0  # +1 line was last seen on the right, -1 on the left
        self.last_w = 0.0     # last angular command (held briefly when line drops out)
        self.lost = 0         # consecutive frames without a line
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
 
        near = self.band_centroid(mask, int(self.p('near_top') * h), int(self.p('near_bot') * h))
        far = self.band_centroid(mask, int(self.p('far_top') * h), int(self.p('far_bot') * h))
 
        if near is None and far is None:
            self.handle_lost(gray, mask)
            return
        self.lost = 0
 
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
        if abs(self.e_f) > 0.1:
            self.last_side = 1.0 if self.e_f > 0 else -1.0
 
        # dt from the image stamp (sim time), so it is right even if the sim
        # runs slower than real time or use_sim_time is not set
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if t <= 0.0:
            t = self.get_clock().now().nanoseconds * 1e-9
        dt = 0.033 if self.last_t is None else max(t - self.last_t, 0.01)
        self.last_t = t
 
        # PID
        self.integral = float(np.clip(self.integral + self.e_f * dt, -1.0, 1.0))
        d = (self.e_f - e_prev) / dt
        u = self.p('kp') * self.e_f + self.p('ki') * self.integral + self.p('kd') * d
 
        cmd = Twist()
        wmax = self.p('w_max')
        # line on the right (e > 0) -> turn right -> negative angular z
        cmd.angular.z = float(np.clip(-u, -wmax, wmax))
        v = self.p('v_max') * (1.0 - self.p('slow_gain') * min(abs(self.e_f), 1.0))
        if far is None:
            # far band is empty: the line is ending (or leaving the view) -> slow down
            v *= self.p('end_slow')
        cmd.linear.x = float(np.clip(v, self.p('v_min'), self.p('v_max')))
        self.last_w = cmd.angular.z
        self.pub.publish(cmd)
        self.publish_debug(gray, mask, cx)
 
    def handle_lost(self, gray, mask):
        """No line in either band. Do NOT spin: hold heading briefly, then stop."""
        self.lost += 1
        self.integral = 0.0
        cmd = Twist()
        if self.lost <= self.p('lost_stop_frames'):
            # short dropout or last few cm of the line: creep on, let the turn rate fade
            cmd.linear.x = self.p('v_min')
            cmd.angular.z = 0.5 * self.last_w
        elif self.p('search_w') > 0.0:
            # optional: rotate toward the side the line was last seen on
            cmd.angular.z = -self.last_side * self.p('search_w')
        # else: zero velocity -> bot stops at the end of the line
        if self.lost == self.p('lost_stop_frames') + 1:
            self.get_logger().info('Line lost - stopping')
        self.last_w = cmd.angular.z
        self.pub.publish(cmd)
        self.publish_debug(gray, mask, None)
 
    def publish_debug(self, gray, mask, cx):
        vis = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
        vis[mask > 0] = (0, 255, 0)
        h, w = gray.shape
        cv2.line(vis, (w // 2, 0), (w // 2, h), (255, 0, 0), 1)
        # band boundaries (yellow): near = lower pair, far = upper pair
        for key in ('near_top', 'near_bot', 'far_top', 'far_bot'):
            y = min(int(self.p(key) * h), h - 1)
            cv2.line(vis, (0, y), (w // 4, y), (0, 255, 255), 1)
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