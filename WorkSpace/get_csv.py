import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import csv

class ScanSubscriber(Node):
    def __init__(self):
        super().__init__('scan_subscriber')
        self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )
        self.csv_file = open('scan_data1.csv', 'w')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(['angle_min', 'angle_max', 'angle_increment', 'ranges'])

    def scan_callback(self, msg):
        self.csv_writer.writerow([msg.angle_min, msg.angle_max, msg.angle_increment ,msg.ranges])

def main(args=None):
    rclpy.init(args=args)
    scan_subscriber = ScanSubscriber()
    rclpy.spin(scan_subscriber)
    scan_subscriber.csv_file.close()
    scan_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()