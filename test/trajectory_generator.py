import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, PoseWithCovariance, Twist, TwistWithCovariance
from tf_transformations import quaternion_from_euler
from dynamic_slam_interfaces.msg import ObjectOdometry
from std_msgs.msg import ColorRGBA
import random
import math


def hsv_to_rgb(h: float, s: float, v: float) -> tuple[float, float, float]:
    """Converts HSV color to RGB color.

    Args:
        h (float): Hue (0.0 to 1.0).
        s (float): Saturation (0.0 to 1.0).
        v (float): Value (brightness) (0.0 to 1.0).

    Returns:
        tuple[float, float, float]: The RGB color (r, g, b), each value between 0.0 and 1.0.
    """
    if s == 0.0:  # achromatic (grey)
        return v, v, v
    i = math.floor(h * 6)
    f = h * 6 - i
    p = v * (1 - s)
    q = v * (1 - f * s)
    t = v * (1 - (1 - f) * s)

    if i % 6 == 0:
        r, g, b = v, t, p
    elif i % 6 == 1:
        r, g, b = q, v, p
    elif i % 6 == 2:
        r, g, b = p, v, t
    elif i % 6 == 3:
        r, g, b = p, q, v
    elif i % 6 == 4:
        r, g, b = t, p, v
    elif i % 6 == 5:
        r, g, b = v, p, q
    return r, g, b


def unique_id_rgb(id: int, saturation: float = 0.5, value: float = 0.95) -> tuple[float, float, float]:
    """Maps an integer ID to a unique RGB color using the golden ratio for hue.

    Args:
        id (int): The unique identifier.
        saturation (float, optional): Saturation of the color (0.0 to 1.0). Defaults to 1.0.
        value (float, optional): Value (brightness) of the color (0.0 to 1.0). Defaults to 1.0.

    Returns:
        tuple[float, float, float]: The RGB color (r, g, b), each value between 0.0 and 1.0.
    """
    phi = (1 + math.sqrt(5)) / 2
    n = (id * phi) % 1  # More efficient modulo
    hue = (n * 256) % 256 / 255.0 # Ensure hue is in range 0-1

    r, g, b = hsv_to_rgb(hue, saturation, value)
    return r, g, b

class TrajectoryGenerator:
    def __init__(self, object_id, frame_id='odom', child_frame_id='base_link', max_speed=1.0, max_angular_speed=0.5):
        self.object_id = object_id
        self.frame_id = frame_id
        self.child_frame_id = child_frame_id
        self.max_speed = max_speed
        self.max_angular_speed = max_angular_speed
        self.position = [0.0, 0.0]  # x, y
        self.orientation = 0.0      # yaw (in radians)
        self.velocity = [0.0, 0.0]  # linear (x), angular (z)

    def generate_random_velocity(self):
        """Generate random linear and angular velocities."""
        linear_speed = random.uniform(-self.max_speed, self.max_speed)
        angular_speed = random.uniform(-self.max_angular_speed, self.max_angular_speed)
        return linear_speed, angular_speed

    def update_position(self, dt):
        """Update the position and orientation based on current velocities."""
        linear_speed = self.velocity[0]
        angular_speed = self.velocity[1]

        # Update orientation
        self.orientation += angular_speed * dt
        self.orientation = math.fmod(self.orientation, 2 * math.pi)  # Keep within [-pi, pi]

        # Update position (2D kinematic equations)
        self.position[0] += linear_speed * math.cos(self.orientation) * dt
        self.position[1] += linear_speed * math.sin(self.orientation) * dt

    def generate_odometry(self, timestamp):
        """Generate an Odometry message for the current state."""
        self.velocity = self.generate_random_velocity()
        odom_msg = Odometry()
        odom_msg.header.stamp = timestamp
        odom_msg.header.frame_id = self.frame_id
        odom_msg.child_frame_id = self.child_frame_id

        # Set position and orientation
        odom_msg.pose.pose.position.x = self.position[0]
        odom_msg.pose.pose.position.y = self.position[1]
        odom_msg.pose.pose.position.z = 0.0

        quat = quaternion_from_euler(0.0, 0.0, self.orientation)  # Roll, pitch, yaw
        odom_msg.pose.pose.orientation.x = quat[0]
        odom_msg.pose.pose.orientation.y = quat[1]
        odom_msg.pose.pose.orientation.z = quat[2]
        odom_msg.pose.pose.orientation.w = quat[3]

        # Set twist (linear and angular velocity)
        odom_msg.twist.twist.linear.x = self.velocity[0]
        odom_msg.twist.twist.angular.z = self.velocity[1]

        object_odom = ObjectOdometry()
        object_odom.odom = odom_msg
        object_odom.object_id = self.object_id

        colour_msg = ColorRGBA()
        random_colour = unique_id_rgb(self.object_id+1)
        colour_msg.r = random_colour[0]
        colour_msg.g = random_colour[1]
        colour_msg.b = random_colour[2]

    
        object_odom.colour = colour_msg
        print(object_odom.colour)

        return object_odom

class RandomTrajectoryNode(Node):
    def __init__(self):
        super().__init__('random_trajectory_node')

        # Parameters
        self.publish_rate = self.declare_parameter('publish_rate', 10.0).value  # Hz
        self.num_trajectories = self.declare_parameter('num_trajectories', 3).value
        self.max_speed = self.declare_parameter('max_speed', 1.0).value         # m/s
        self.max_angular_speed = self.declare_parameter('max_angular_speed', 0.5).value  # rad/s

        # Publisher
        self.odom_publisher = self.create_publisher(ObjectOdometry, 'object_odom', 10)

        # Timer
        self.timer = self.create_timer(1.0 / self.publish_rate, self.publish_odometries)

        # Trajectory Generators
        self.trajectories = [
            TrajectoryGenerator(
                object_id=int(i),
                frame_id=f'odom',
                child_frame_id=f'base_link',
                max_speed=self.max_speed,
                max_angular_speed=self.max_angular_speed
            ) for i in range(self.num_trajectories)
        ]

        self.current_time = self.get_clock().now()

        self.get_logger().info('Random Trajectory Node has been started.')

    def publish_odometries(self):
        """Publish odometry messages for all trajectories."""
        now = self.get_clock().now()
        dt = (now - self.current_time).nanoseconds * 1e-9
        self.current_time = now

        for trajectory in self.trajectories:
            trajectory.update_position(dt)
            odom_msg = trajectory.generate_odometry(now.to_msg())
            self.odom_publisher.publish(odom_msg)
            self.get_logger().info(f'Published odometry for {trajectory.child_frame_id}: position={trajectory.position}, orientation={trajectory.orientation}')


def main(args=None):
    rclpy.init(args=args)
    node = RandomTrajectoryNode()
    rclpy.spin(node)
    # try:
    #     rclpy.spin(node)
    # except KeyboardInterrupt:
    #     node.get_logger().info('Node terminated by user. Shutting down.')
    # finally:
    #     node.destroy_node()
    #     rclpy.shutdown()

if __name__ == '__main__':
    main()
