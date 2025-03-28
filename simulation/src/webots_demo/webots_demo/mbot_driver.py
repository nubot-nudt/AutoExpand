import rclpy
import os
import numpy as np
from geometry_msgs.msg import Twist
from sensor_msgs.msg import PointField,PointCloud2
HALF_DISTANCE_BETWEEN_WHEELS = 0.045
WHEEL_RADIUS = 0.025
# MbotDriver 是机器人的控制器程序，他创建了一个 ros2 节点 mbot_driver，
# 订阅 cmd_vel 话题，接收 Twist 消息，然后根据 Twist 消息的线速度和角速度控制机器人的左右轮速度，从而实现机器人的运动控制。
class MbotDriver:
    def init(self, webots_node, properties):
        # 获取 webots 里的 robot 对象，在本样例就是 my_world.wbt 中的 mbot_car
        self._robot = webots_node.robot
        self.time_step = 10000                                         
        # 获取左右轮的电机对象，并设置电机的目标位置为极大（一直旋转）和速度为 0
        # self._left_motor = self._robot.getDevice('left wheel motor')
        # self._right_motor = self._robot.getDevice('right wheel motor')

        # self._left_motor.setPosition(float('inf'))
        # self._left_motor.setVelocity(0)

        # self._right_motor.setPosition(float('inf'))
        # self._right_motor.setVelocity(0)

        self._ladar = self._robot.getDevice('Velodyne VLP-16')
        self._ladar.enable(self.time_step)
        self._ladar.enablePointCloud()

        self._target_twist = Twist()

        # 创建一个 ros2 节点 mbot_driver，订阅 cmd_vel 话题，用来驱动 mbot_car
        # 接收到的 Twist 消息存入 self._target_twist 中，等待 step 函数处理
        rclpy.init(args=None)
        self._node = rclpy.create_node('mbot_driver')
        self._node.create_subscription(Twist, 'cmd_vel', self._cmd_vel_callback, 1)
        self._point_cloud_publisher =self._node.create_publisher(PointCloud2, 'point_cloud', 10)
        self._obstacle_point_cloud_publisher = self._node.create_publisher(PointCloud2, 'obstacle_points', 10)

    def _cmd_vel_callback(self, twist):
        self._target_twist = twist

    # step 由 webots_ros2_driver.webots_controller.WebotsController 调用，称之为 simulation step
    # 这里可以理解为是机器人控制器的主循环函数，周期调用
    def step(self):
        # 使用 spin_once 来处理 mbot_driver 的一次事件，这里是一次 cmd_vel 订阅
        # 如果没有这个函数，_cmd_vel_callback 是不会被执行的
        # 如果事件没来，会立即返回，不会阻塞，确保实时性
        rclpy.spin_once(self._node, timeout_sec=0)

        # 这里讲解了 Twist 理解 和 右手定则：https://blog.csdn.net/cy1641395022/article/details/131236155
        # 获取机器人的前进速度和旋转速度，根据右手定则：
        # 如果机器人向左转（顺时针），angular_speed为负；
        # 如果机器人向右转（逆时针），angular_speed为正；
        forward_speed = self._target_twist.linear.x
        angular_speed = self._target_twist.angular.z

        # 机器人的前进和旋转速度需要转换为左右轮转速，由于机器人是差速驱动，所以需要根据机器人的轮距和轮径来计算左右轮转速
        # HALF_DISTANCE_BETWEEN_WHEELS 是机器人的轮距的一半，乘以 angular_speed 就是机器人内外轮线速度的补偿值
        # 得到内外轮线速度后，再除以 WHEEL_RADIUS 就是内外轮的转速（角速度乘以旋转半径为线速度）
        # 简单的三个场景，可以帮助理解这个公式：
        # 第一，控制机器人直线前进（forward_speed 为正，angular_speed 为0），左右轮转速必须相同，且转向相同
        # 第二，控制机器人原地顺时针旋转（forward_speed 为0，angular_speed 为负），左右轮转速必须相同，且转向相反
        # 第三，控制机器人原地逆时针旋转（forward_speed 为0，angular_speed 为正），左右轮转速必须相同，且转向相反
        command_motor_left = (forward_speed - angular_speed * HALF_DISTANCE_BETWEEN_WHEELS) / WHEEL_RADIUS
        command_motor_right = (forward_speed + angular_speed * HALF_DISTANCE_BETWEEN_WHEELS) / WHEEL_RADIUS
        self.save_directory = "/home/sjf/colcon_ws/src/webots_demo/resource"  # 指定保存点云文件的目录
        os.makedirs(self.save_directory, exist_ok=True)  # 确保目录存在
        self.point_cloud_filename = f"{self.save_directory}/sense6-1.txt"  # 指定点云文件名
        # 设置左右轮的目标转速
        # self._left_motor.setVelocity(command_motor_left)
        # self._right_motor.setVelocity(command_motor_right)
        point_cloud = self._ladar.getPointCloud()
        if point_cloud:
            self.publish_point_cloud(point_cloud)
            self.publish_obstacle_point_cloud(point_cloud)
            # self.save_point_cloud(point_cloud)



    def publish_point_cloud(self, point_cloud):
            # 提取点云数据中的x, y, z坐标
            points = [(p.x, p.y, p.z) for p in point_cloud]

            # 创建PointCloud2消息的header
            header = self.create_header()

            # 定义点云字段
            fields = [
                PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
                PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
                PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1)
            ]

            # 创建PointCloud2消息
            point_cloud_msg = PointCloud2()
            point_cloud_msg.header = header
            point_cloud_msg.height = 1  # 假设点云是无序的
            point_cloud_msg.width = len(points)
            point_cloud_msg.fields = fields
            point_cloud_msg.is_bigendian = False
            point_cloud_msg.point_step = 12  # 每个点占用的字节数，3个float32，每个4字节
            point_cloud_msg.row_step = point_cloud_msg.point_step * point_cloud_msg.width
            point_cloud_msg.data = np.array(points, dtype=np.float32).tobytes()

            # 发布点云消息
            self._point_cloud_publisher.publish(point_cloud_msg)
    def publish_obstacle_point_cloud(self, point_cloud):
            # 过滤掉地面点云，假设地面点云的z值小于0.1
            obstacle_points = [(p.x, p.y, p.z) for p in point_cloud if p.z >-0.35]
            if not obstacle_points:
                return

            header = self.create_header()
            fields = [
                PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
                PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
                PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1)
            ]
            obstacle_point_cloud_msg = PointCloud2()
            obstacle_point_cloud_msg.header = header
            obstacle_point_cloud_msg.height = 1
            obstacle_point_cloud_msg.width = len(obstacle_points)
            obstacle_point_cloud_msg.fields = fields
            obstacle_point_cloud_msg.is_bigendian = False
            obstacle_point_cloud_msg.point_step = 12
            obstacle_point_cloud_msg.row_step = obstacle_point_cloud_msg.point_step * obstacle_point_cloud_msg.width
            obstacle_point_cloud_msg.data = np.array(obstacle_points, dtype=np.float32).tobytes()
            self._obstacle_point_cloud_publisher.publish(obstacle_point_cloud_msg)

            # 保存障碍物点云
            self.save_point_cloud(obstacle_points)



    def save_point_cloud(self, point_cloud):
    
        with open(self.point_cloud_filename, 'w') as file:
            for point in point_cloud:
                file.write(f"{point[0]} {point[1]} {point[2]}\n")
        print(f"Point cloud saved to {self.point_cloud_filename}")



    def create_header(self):
            header = PointCloud2().header
            header.frame_id = 'velodyne'
            header.stamp = self._node.get_clock().now().to_msg()
            return header