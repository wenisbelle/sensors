import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, Twist

class BlobTracker(Node):
    def __init__(self) -> None:
        super().__init__('blob_tracker')
        self.subscription = self.create_subscription(
            Point,
            '/point_blob',
            self.blob_callback,
            10)
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        self.angular_gain = 0.05  
        self.no_blob_detected = True
        self.timer = self.create_timer(1.0, self.rotate_continuous)  

    def blob_callback(self, msg: Point) -> None:
        # Extract blob coordinates
        blob_x = msg.x
        blob_y = msg.y

        # Define linear velocity
        linear_x = 0.3  

        if blob_x != 0.0 or blob_y != 0.0:
            # Adjust angular velocity based on blob's position
            angular_vel = self.angular_gain * blob_x
            # Clip angular velocity to [-1, 1] range
            angular_vel = max(min(angular_vel, 0.1), -0.1)

            print("y blob", blob_y)

            # Adjust linear velocity based on blob's position
            if blob_y > -0.9:
                linear_vel = linear_x
            else:
                linear_vel = 0.0  # Stop the robot
            self.no_blob_detected = False
        else:
            return

        self.pub_velocities(linear_vel, angular_vel)

    def rotate_continuous(self) -> None:
        # Rotate continuously
        if self.no_blob_detected:
            twist_msg = Twist()
            twist_msg.angular.z = 0.05  # Adjust angular velocity as needed
            self.publisher.publish(twist_msg)
            self.get_logger().warn("No blobs detected yet ... Rotating continuously")

    def pub_velocities(self, linear: float, angular: float) -> None:
        # Create Twist message with linear and angular velocities
        twist_msg = Twist()
        twist_msg.linear.x = linear
        twist_msg.angular.z = angular

        # Publish the Twist message
        self.publisher.publish(twist_msg)
        self.get_logger().info(f"Linear vel: {linear:.2f}, Angular vel: {angular:.2f}")


def main(args=None) -> None:
    rclpy.init(args=args)
    blob_tracker = BlobTracker()
    rclpy.spin(blob_tracker)
    blob_tracker.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()