import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from std_msgs.msg import String
from sensor_msgs.msg import NavSatFix
import pandas as pd 
class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.subscription = self.create_subscription(
            NavSatFix,
            '/gps/fix',
            self.listener_callback, qos_profile=qos_profile)
        self.subscription  # prevent unused variable warning

        self.data_log = []

    def listener_callback(self, msg):
        self.data_log.append([msg.latitude, msg.longitude, msg.altitude])
        self.get_logger().info(f"Recorded [{len(self.data_log)}]")
    
    def destroy_node(self):
        super()
        df:pd.DataFrame = pd.DataFrame(self.data_log, columns=['latitude','longitude','altitude'])

        df.to_csv("./gps_3.csv")

def main(args=None):
    try:
        rclpy.init(args=args)

        minimal_subscriber = MinimalSubscriber()

        rclpy.spin(minimal_subscriber)
    finally:
        minimal_subscriber.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
