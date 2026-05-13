import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time


def calculate_fps(prev_time):
    current_time = time.time()
    fps = 1 / (current_time - prev_time)
    return fps, current_time


def gstreamer_pipeline(
    capture_width=640,
    capture_height=480,
    display_width=640,
    display_height=480,
    framerate=60,
    flip_method=0,
):
    return (
        "nvarguscamerasrc ! "
        "video/x-raw(memory:NVMM), "
        "width=(int)%d, height=(int)%d, "
        "format=(string)NV12, framerate=(fraction)%d/1 ! "
        "nvvidconv flip-method=%d ! "
        "video/x-raw, width=(int)%d, height=(int)%d, format=(string)BGRx ! "
        "videoconvert ! "
        "video/x-raw, format=(string)BGR ! appsink"
        % (
            capture_width,
            capture_height,
            framerate,
            flip_method,
            display_width,
            display_height,
        )
    )


class ColorDetector(Node):

    COLOR_RANGES = {
        'yellow': (
            np.array([20, 100, 100]),
            np.array([35, 255, 255])
        ),
        'green': (
            np.array([35, 100, 100]),
            np.array([85, 255, 255])
        )
    }

    def __init__(self, camera_id=1):

        super().__init__('color_follow_node')

        self.publisher = self.create_publisher(
            Twist,
            'cmd_vel',
            10
        )

        self.twist = Twist()

        self.width = 640
        self.height = 480

        self.move_flag = self.width // 2

        

       
        self.enter_distance = 100

        
        self.exit_distance = 40

     
        self.car_speed = 0.04

   
        self.side_speed = 0.04


        self.stop_width = 170


        self.filtered_cx = None


        self.filter_alpha = 0.3

        self.current_motion = "stop"

        self.switching = False

        self.switch_time = 0

        self.switch_pause = 0.15

        self.target_motion = "stop"

        self.drawing = False

        self.ix, self.iy = -1, -1
        self.fx, self.fy = -1, -1

        self.roi_selected = False

        self.target_color = None

        self.current_frame = None


        self.cap = cv2.VideoCapture(
            gstreamer_pipeline(flip_method=0),
            cv2.CAP_GSTREAMER
        )

        if not self.cap.isOpened():
            print("错误：无法打开相机")
            exit(0)

        self.window_name = 'Color Follow'

        cv2.namedWindow(self.window_name)

        cv2.setMouseCallback(
            self.window_name,
            self._mouse_callback
        )


    def _mouse_callback(self, event, x, y, flags, param):

        if event == cv2.EVENT_LBUTTONDOWN:

            self.drawing = True

            self.roi_selected = False

            self.ix, self.iy = x, y

        elif event == cv2.EVENT_MOUSEMOVE:

            self.fx, self.fy = x, y

        elif event == cv2.EVENT_LBUTTONUP:

            self.drawing = False

            self.roi_selected = True

            self._detect_roi_color()

    def _detect_roi_color(self):

        x1, y1 = min(self.ix, self.fx), min(self.iy, self.fy)
        x2, y2 = max(self.ix, self.fx), max(self.iy, self.fy)

        if x2 - x1 <= 20 or y2 - y1 <= 20:

            print("框选区域太小")

            return

        roi = self.current_frame[y1:y2, x1:x2].copy()

        hsv_roi = cv2.cvtColor(
            roi,
            cv2.COLOR_BGR2HSV
        )

        mask_yellow = cv2.inRange(
            hsv_roi,
            *self.COLOR_RANGES['yellow']
        )

        mask_green = cv2.inRange(
            hsv_roi,
            *self.COLOR_RANGES['green']
        )

        yellow_pixels = cv2.countNonZero(mask_yellow)

        green_pixels = cv2.countNonZero(mask_green)

        if yellow_pixels > green_pixels and yellow_pixels > 50:

            self.target_color = 'yellow'

            print("已选择黄色木块")

        elif green_pixels > yellow_pixels and green_pixels > 50:

            self.target_color = 'green'

            print("已选择绿色木块")

        else:

            self.target_color = None

            print("未检测到有效颜色")


    def _detect_color(self, frame, color_name):

        hsv = cv2.cvtColor(
            frame,
            cv2.COLOR_BGR2HSV
        )

        lower, upper = self.COLOR_RANGES[color_name]

        mask = cv2.inRange(
            hsv,
            lower,
            upper
        )

        # 去噪
        kernel = np.ones((5, 5), np.uint8)

        mask = cv2.morphologyEx(
            mask,
            cv2.MORPH_OPEN,
            kernel
        )

        mask = cv2.morphologyEx(
            mask,
            cv2.MORPH_CLOSE,
            kernel
        )

        contours, _ = cv2.findContours(
            mask,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE
        )

        centers = []

        for contour in contours:

            area = cv2.contourArea(contour)

            if area > 500:

                M = cv2.moments(contour)

                if M['m00'] != 0:

                    cx = int(M['m10'] / M['m00'])
                    cy = int(M['m01'] / M['m00'])

                    x, y, w, h = cv2.boundingRect(contour)

                    centers.append(
                        (cx, cy, x, y, w, h)
                    )

        if len(centers) > 0:

            centers = sorted(
                centers,
                key=lambda c: c[4] * c[5],
                reverse=True
            )

            return centers[0], mask

        return None, mask



    def _draw_selection_box(self, frame):

        if self.drawing:

            display_frame = frame.copy()

            x1, y1 = min(self.ix, self.fx), min(self.iy, self.fy)
            x2, y2 = max(self.ix, self.fx), max(self.iy, self.fy)

            cv2.rectangle(
                display_frame,
                (x1, y1),
                (x2, y2),
                (255, 255, 0),
                2
            )

            return display_frame

        return frame



    def smooth_cx(self, cx):

        if self.filtered_cx is None:

            self.filtered_cx = cx

        self.filtered_cx = (
            (1 - self.filter_alpha) * self.filtered_cx
            + self.filter_alpha * cx
        )

        return int(self.filtered_cx)

   

    def set_motion(self, motion):

        
        if motion != self.current_motion:

            # 
            if not self.switching:

                print(
                    f"方向切换: "
                    f"{self.current_motion} -> {motion}"
                )


                self.twist.linear.x = 0.0
                self.twist.linear.y = 0.0

                self.publisher.publish(self.twist)

                self.switching = True

                self.switch_time = time.time()

                self.target_motion = motion

                return

            else:

                if (
                    time.time() - self.switch_time
                    >= self.switch_pause
                ):

                    self.switching = False

                    self.current_motion = self.target_motion

                else:

                    self.twist.linear.x = 0.0
                    self.twist.linear.y = 0.0

                    return

        motion = self.current_motion

        if motion == "left":

            self.twist.linear.x = 0.0
            self.twist.linear.y = self.side_speed

        elif motion == "right":

            self.twist.linear.x = 0.0
            self.twist.linear.y = -self.side_speed

        elif motion == "go":

            self.twist.linear.x = self.car_speed
            self.twist.linear.y = 0.0

        elif motion == "stop":

            self.twist.linear.x = 0.0
            self.twist.linear.y = 0.0


    def run(self):

        prev_time = time.time()

        while True:

            ret, frame = self.cap.read()

            if not ret:

                print("无法读取相机")

                break

            frame = cv2.flip(frame, -1)

            self.current_frame = frame



            if self.roi_selected and self.target_color:

                result, mask = self._detect_color(
                    frame,
                    self.target_color
                )

                if result is not None:

                    cx, cy, x, y, w, h = result

                    cx = self.smooth_cx(cx)

                    cv2.rectangle(
                        frame,
                        (x, y),
                        (x + w, y + h),
                        (0, 255, 255),
                        2
                    )

                    cv2.circle(
                        frame,
                        (cx, cy),
                        5,
                        (0, 0, 255),
                        -1
                    )

                    # cv2.line(
                    #     frame,
                    #     (self.move_flag, 0),
                    #     (self.move_flag, self.height),
                    #     (255, 0, 0),
                    #     2
                    # )


                    if w >= self.stop_width:

                        print("stop")

                        self.set_motion("stop")

                    else:


                        error = self.move_flag - cx

                        abs_error = abs(error)

                        print("cx =", cx)
                        print("error =", abs_error)
                        print("motion =", self.current_motion)


                        if self.current_motion == "left":

                          
                            if abs_error < self.exit_distance:

                                print("left -> go")

                                self.set_motion("go")

                            else:

                                self.set_motion("left")

                   
                        elif self.current_motion == "right":

                            if abs_error < self.exit_distance:

                                print("right -> go")

                                self.set_motion("go")

                            else:

                                self.set_motion("right")

                        
                        else:

                            
                            if error > self.enter_distance:

                                print("enter left")

                                self.set_motion("left")

                            elif error < -self.enter_distance:

                                print("enter right")

                                self.set_motion("right")

                            else:

                                self.set_motion("go")

                else:

                    print("no detect")

                    self.filtered_cx = None

                    self.set_motion("stop")

           

            self.publisher.publish(self.twist)

           

            fps, prev_time = calculate_fps(prev_time)

            # cv2.putText(
            #     frame,
            #     f'FPS: {int(fps)}',
            #     (10, 30),
            #     cv2.FONT_HERSHEY_SIMPLEX,
            #     1,
            #     (0, 255, 0),
            #     2
            # )

            display_frame = self._draw_selection_box(frame)

            cv2.imshow(
                self.window_name,
                display_frame
            )

            key = cv2.waitKey(1) & 0xFF

            if key == ord('q'):

                break

            elif key == ord('r'):

                self.set_motion("stop")

                self.roi_selected = False

                self.target_color = None

                self.filtered_cx = None

                self.current_frame = None

                print("已重置")

  

    def release(self):

        self.twist.linear.x = 0.0
        self.twist.linear.y = 0.0

        self.publisher.publish(self.twist)

        self.cap.release()

        cv2.destroyAllWindows()

        self.destroy_node()

        rclpy.shutdown()

        print("程序退出")


if __name__ == '__main__':

    rclpy.init()

    detector = ColorDetector(camera_id=1)

    try:

        if detector.cap.isOpened():

            detector.run()

    except KeyboardInterrupt:

        detector.release()