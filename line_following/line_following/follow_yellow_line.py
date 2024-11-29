import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist

class LineFollower(Node):

    def __init__(self) -> None:
        super().__init__('line_follower')
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(Image, '/front_camera_topic', self.camera_callback, 10)
        self.pub_vel = self.create_publisher(Twist, 'cmd_vel', 10)

    def camera_callback(self, msg: Image) -> None:
        try:
            # Convert ROS Image message to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().info(f"Error converting image: {e}")
            return

        height, width, _ = cv_image.shape
        rows_to_watch = 20
        # Crop image 
        crop_img = cv_image[height*3//4:height*3//4 + rows_to_watch][1:width]

        # Convert BGR image to HSV
        hsv = cv2.cvtColor(crop_img, cv2.COLOR_BGR2HSV)

        # Define lower and upper bounds for yellow color in HSV
        lower_yellow = np.array([28, 144, 89])
        upper_yellow = np.array([54, 255, 106])

        # Create a binary mask for yellow color
        mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

        # Calculate centroid of the mask
        moment = cv2.moments(mask, False)
        try:
            # x and y coordinates of centroid
            cx, cy = moment['m10'] / moment['m00'], moment['m01'] / moment['m00']
        except ZeroDivisionError:
            cy, cx = height/2, width/2

        res = cv2.bitwise_and(crop_img, crop_img, mask=mask)

        # Draw circle to visualize the detected centroid
        cv2.circle(res, (int(cx), int(cy)), 10, (0, 0, 255), -1)

        # Display
        scale_factorx = 2
        scale_factory = 10
        resized_res = cv2.resize(res, (0, 0), fx=scale_factorx, fy=scale_factory)

        # Display resized image
        cv2.imshow("RES", resized_res)
        cv2.waitKey(1)

        # Calculate horizontal error between the centroid and the center of the image
        error_x = cx - width / 2
        
        # Publish velocities
        self.pub_velocities(error_x)


    def pub_velocities(self, error: float) -> None:
        twist_object = Twist()
        twist_object.linear.x = 0.2
        twist_object.angular.z = error / 2000
        self.get_logger().info(f"Adjusting the angular velocity ---> {twist_object.angular.z:.2f}" )
        self.pub_vel.publish(twist_object)


def main(args=None) -> None:
    rclpy.init(args=args)
    line_follower = LineFollower()
    try:
        rclpy.spin(line_follower)
    except KeyboardInterrupt:
        pass
    line_follower.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()