import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from std_msgs.msg import String
from sensor_msgs.msg import NavSatFix
import pandas as pd 
import datetime
from pathlib import Path

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


        self.prev_time = datetime.datetime.now()
        self.max_trials = 10
        self.trials_count = 0 
        self.trial_interval= datetime.timedelta(minutes=90) # 90 minutes
        # self.trial_interval= datetime.timedelta(seconds=5) # 90 minutes


        self.output_file_path = Path("./rtk_accuracy_test.txt")
        

    def listener_callback(self, msg:NavSatFix):
        if self.trials_count < self.max_trials:
            time_now = datetime.datetime.now() 
            time_diff = time_now-self.prev_time
            if time_diff > self.trial_interval:
                self.prev_time = time_now
                self.trials_count += 1
                self.write_msg_to_file(msg)
                self.get_logger().info(f"Message {msg} written")
        else:
            self.get_logger().info(f"Max trial [{self.max_trials}] reached")

    def write_msg_to_file(self, msg:NavSatFix):
        output_file = self.output_file_path.open('a')
        output_file.write(f"{datetime.datetime.now()},{msg.latitude},{msg.longitude},{msg.altitude}\n")
        output_file.close()
    
    def destroy_node(self):
        super()


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
