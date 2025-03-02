import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Int32  # 订阅的条件数据

class NavigationClient(Node):
    def __init__(self):
        super().__init__('navigation_client')

        # 创建一个 Action 客户端，用于发送导航目标点
        self.client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # 订阅一个话题，用于接收条件
        self.subscription = self.create_subscription(
            Int32,  # 订阅的是整数类型的数据
            '/nav_condition',  # 你可以改成实际使用的话题
            self.condition_callback,
            10
        )

    def send_goal(self, x, y, yaw):
        """发送导航目标点"""
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = "map"
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.orientation.z = yaw  # 这里简单处理，建议用四元数

        self.client.wait_for_server()
        future = self.client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future)  # 等待目标完成
        self.get_logger().info(f"到达目标点: x={x}, y={y}, yaw={yaw}")

    def condition_callback(self, msg):
        """当接收到条件数据时，决定导航到哪个点"""
        condition = msg.data  # 取出接收到的整数数据
        self.get_logger().info(f"收到条件: {condition}")

        if condition == 1:
            self.send_goal(2.0, 3.0, 0.0)
        elif condition == 2:
            self.send_goal(4.0, 1.0, 1.57)
        elif condition == 3:
            self.send_goal(1.0, -2.0, -1.57)
        elif condition == 4:
            self.send_goal(-3.0, 4.0, 0.0)
        elif condition == 5:
            self.send_goal(-2.0, -3.0, 1.57)
        elif condition == 6:
            self.send_goal(5.0, 0.0, -1.57)
        elif condition == 7:
            self.send_goal(-5.0, 5.0, 3.14)
        elif condition == 8:
            self.send_goal(0.0, -5.0, 0.0)
        elif condition == 9:
            self.send_goal(3.0, 3.0, 1.0)
        elif condition == 10:
            self.send_goal(-3.0, -3.0, -1.0)
        else:
            self.get_logger().info("未定义的条件值")

def main(args=None):
    rclpy.init(args=args)
    nav_client = NavigationClient()
    rclpy.spin(nav_client)  # 让节点一直运行，等待消息
    nav_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
