import os
import time
import traceback
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Bool
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO

# Subscription topics
RGB_IMAGE_TOPIC = '/rgb/image_raw'
RGB_CAMERA_INFO_TOPIC = '/rgb/camera_info'

# Publication topics
LEFT_HUMAN_TOPIC  = '/human_detection/left'
RIGHT_HUMAN_TOPIC = '/human_detection/right'

# Debug
DEBUG_LOG = False
DEBUG_SAVE_IMAGES = False

# Maximum processing rate (Hz; None -> unlimited)
MAX_PROCESS_HZ = 2

# Class ID for 'person'
PERSON_CLASS_ID = 0

# Lower exclusion line (% of image height): Only persons with lower-right corner above this line are marked red
LOWER_LINE_LEFT_Y  = 0.64
LOWER_LINE_RIGHT_Y = 0.85

# Left exclusion line (top to bottom): Persons with lower-right corner left of this line are NEVER marked red
LEFT_LINE_TOP_X    = 0.68
LEFT_LINE_BOTTOM_X = 0.21

# Classification line (top to bottom): left (near sink) or right (between tables)? Classification based on which side of this line has more % of frame area.
CLASS_LINE_TOP_X    = 0.73
CLASS_LINE_BOTTOM_X = 0.59


def lower_line_y_at(x: int, w: int, h: int) -> float:
    """Return the Y coordinate of the lower exclusion line at pixel column x."""
    return h * LOWER_LINE_LEFT_Y + (h * LOWER_LINE_RIGHT_Y - h * LOWER_LINE_LEFT_Y) * (x / (w - 1))


def class_line_x_at(y: np.ndarray, w: int, h: int) -> np.ndarray:
    """Return the X coordinate of the classification line at pixel row(s) y."""
    return w * CLASS_LINE_TOP_X + (w * CLASS_LINE_BOTTOM_X - w * CLASS_LINE_TOP_X) * (y / (h - 1))


def left_line_x_at(y: float, w: int, h: int) -> float:
    """Return the X coordinate of the left exclusion line at pixel row y."""
    return w * LEFT_LINE_TOP_X + (w * LEFT_LINE_BOTTOM_X - w * LEFT_LINE_TOP_X) * (y / (h - 1))


def left_of_left_line(x2: int, y2: int, w: int, h: int) -> bool:
    """True if the lower-right corner of the bounding box is left of the left exclusion line."""
    return x2 < left_line_x_at(y2, w, h)


def side_of_class_line(x1: int, y1: int, x2: int, y2: int, w: int, h: int):
    """Return (side, left_pct, right_pct) based on classification line split of box area."""
    ys = np.arange(y1, y2 + 1, dtype=np.float64)
    bxs = class_line_x_at(ys, w, h)
    left_area  = float(np.sum(np.clip(bxs, x1, x2) - x1))
    right_area = float(np.sum(x2 - np.clip(bxs, x1, x2)))
    total = left_area + right_area
    left_pct  = 100.0 * left_area  / total if total > 0 else 0.0
    right_pct = 100.0 * right_area / total if total > 0 else 0.0
    side = 'left' if left_pct >= right_pct else 'right'
    return side, left_pct, right_pct



class HumanDetectionNode(Node):
    def __init__(self):
        super().__init__('human_detection_node')

        self.bridge = CvBridge()
        self.model = YOLO('yolo26m.pt')

        # Only keep the latest frame
        qos = QoSProfile(history=QoSHistoryPolicy.KEEP_LAST, depth=1)
        self.subscription = self.create_subscription(
            Image,
            RGB_IMAGE_TOPIC,
            self.image_callback,
            qos
        )

        self.left_pub  = self.create_publisher(Bool, LEFT_HUMAN_TOPIC,  10)
        self.right_pub = self.create_publisher(Bool, RIGHT_HUMAN_TOPIC, 10)

        # One-shot subscription to check camera distortion
        self._camera_info_sub = self.create_subscription(
            CameraInfo,
            RGB_CAMERA_INFO_TOPIC,
            self._camera_info_callback,
            1
        )

        self.save_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'human_detection_images')
        if DEBUG_SAVE_IMAGES:
            os.makedirs(self.save_dir, exist_ok=True)

        self._camera_matrix = None
        self._dist_coeffs = None
        self._last_process_time = 0.0
        self._min_interval = 1.0 / MAX_PROCESS_HZ if MAX_PROCESS_HZ else 0.0

        self.get_logger().info(f'Human Detection Node started, listening on {RGB_IMAGE_TOPIC}')

    def _camera_info_callback(self, msg: CameraInfo):
        self._camera_matrix = np.array(msg.k, dtype=np.float64).reshape(3, 3)
        self._dist_coeffs   = np.array(msg.d, dtype=np.float64)

        d = list(msg.d)
        model = msg.distortion_model
        has_distortion = any(abs(v) > 1e-6 for v in d)
        self.get_logger().info(
            f'Camera distortion model : {model}\n'
            f'Distortion coefficients : {d}\n'
            f'Has distortion          : {has_distortion}'
        )
        # Unsubscribe after the first message
        self.destroy_subscription(self._camera_info_sub)

    def image_callback(self, msg):
        try:
            # Rate-limit processing
            now = time.monotonic()
            if now - self._last_process_time < self._min_interval:
                return
            self._last_process_time = now

            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # Undistort if camera info has been received
            if self._camera_matrix is not None and self._dist_coeffs is not None:
                cv_image = cv2.undistort(cv_image, self._camera_matrix, self._dist_coeffs)

            results = self.model(cv_image, classes=[PERSON_CLASS_ID], verbose=False)

            human_left  = False
            human_right = False

            h, w = cv_image.shape[:2]

            # Draw lines for debugging
            if DEBUG_SAVE_IMAGES:
                cv2.line(cv_image, (0, int(h * LOWER_LINE_LEFT_Y)), (w - 1, int(h * LOWER_LINE_RIGHT_Y)), (0, 0, 255), 2)
                cv2.line(cv_image,
                         (int(w * LEFT_LINE_TOP_X),    0),
                         (int(w * LEFT_LINE_BOTTOM_X), h - 1),
                         (0, 0, 255), 2)
                cv2.line(cv_image,
                         (int(w * CLASS_LINE_TOP_X),    0),
                         (int(w * CLASS_LINE_BOTTOM_X), h - 1),
                         (255, 0, 0), 2)

            for result in results:
                for box in result.boxes:
                    x1, y1, x2, y2 = box.xyxy[0].cpu().numpy().astype(int)
                    conf = float(box.conf[0].cpu().numpy())

                    # Red if the lower-right corner is above the lower exclusion line, green otherwise
                    # Exception: never mark red if lower-right corner is left of the left exclusion line
                    above_lower_line = y2 < lower_line_y_at(x2, w, h)
                    left_excluded    = left_of_left_line(x2, y2, w, h)
                    color = (0, 0, 255) if (above_lower_line and not left_excluded) else (0, 255, 0)

                    cv2.rectangle(cv_image, (x1, y1), (x2, y2), color, 2)

                    if above_lower_line and not left_excluded:
                        side, left_pct, right_pct = side_of_class_line(x1, y1, x2, y2, w, h)
                        label = f'person {side} L:{left_pct:.0f}% R:{right_pct:.0f}% ({conf:.2f})'
                        if side == 'left':
                            human_left = True
                        else:
                            human_right = True
                    else:
                        side = 'n/a'
                        label = f'person: {conf:.2f}'

                    cv2.putText(cv_image, label, (x1, y1 - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)

                    if DEBUG_LOG:
                        self.get_logger().info(f'Human detected at [{x1},{y1},{x2},{y2}] conf={conf:.2f} above_lower_line={above_lower_line} left_excluded={left_excluded} side={side}')

            # Publish left/right human presence
            self.left_pub.publish(Bool(data=human_left))
            self.right_pub.publish(Bool(data=human_right))

            if DEBUG_SAVE_IMAGES:
                timestamp = self.get_clock().now().nanoseconds
                filename = os.path.join(self.save_dir, f'detection_{timestamp}.jpg')
                cv2.imwrite(filename, cv_image)

        except Exception as e:
            self.get_logger().error(f'Error in image_callback: {e}\n{traceback.format_exc()}')


def main(args=None):
    rclpy.init(args=args)
    node = HumanDetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
