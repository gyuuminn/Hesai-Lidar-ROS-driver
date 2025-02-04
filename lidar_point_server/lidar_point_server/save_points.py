#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
import csv
import os

class LidarPointSaver(Node):
    def __init__(self):
        super().__init__('lidar_point_saver')

        # 1) 토픽 구독
        self.subscription = self.create_subscription(
            PointCloud2,
            '/lidar_points',
            self.listener_callback,
            10
        )

        # 2) CSV 파일 준비
        self.file_path = '/home/eric/0203_bag/810_bag/CSV/no/no_sus/3.csv'
        if not os.path.isfile(self.file_path):
            with open(self.file_path, 'w', newline='') as file:
                writer = csv.writer(file)
                # 헤더: sec, nanosec, x, y, z, intensity, ring, timestamp, azimuth_angle, distance
                writer.writerow([
                    'sec',
                    'nanosec',
                    'x',
                    'y',
                    'z',
                    'intensity',
                    'ring',
                    'timestamp',
                    'azimuth_angle',
                    'distance'
                ])

        # 3) 36초 후 자동 종료 타이머
        self.timer = self.create_timer(48.0, self.timer_callback)

    def listener_callback(self, msg):
        sec = msg.header.stamp.sec
        nanosec = msg.header.stamp.nanosec

        # 필드 이름이 실제 메시지와 일치하는지 확인 필요
        points = pc2.read_points(
            msg,
            field_names=(
                'x',
                'y',
                'z',
                'intensity',
                'ring',
                'timestamp',
                'azimuth_angle',
                'distance'
            ),
            skip_nans=True
        )

        with open(self.file_path, 'a', newline='') as file:
            writer = csv.writer(file)
            for p in points:
                x = p[0]
                y = p[1]
                z = p[2]
                intensity = p[3]
                ring = p[4]
                ts = p[5]
                azimuth_angle = p[6]
                distance = p[7]

                # CSV 한 행: [sec, nanosec, x, y, z, intensity, ring, timestamp, azimuth_angle, distance]
                writer.writerow([sec, nanosec, x, y, z, intensity, ring, ts, azimuth_angle, distance])

        self.get_logger().info('Saved pointcloud data to CSV')

    def timer_callback(self):
        # 36초 뒤 자동 종료
        self.get_logger().info('48초가 지났습니다! 노드를 종료합니다.')
        self.timer.cancel()
        self.destroy_node()
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = LidarPointSaver()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('KeyboardInterrupt 발생! 종료를 진행합니다...')
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()
