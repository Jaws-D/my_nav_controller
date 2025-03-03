import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from rclpy.qos import QoSProfile
from referee_msg.msg import Referee  # 自定义消息类型
from action_msgs.msg import GoalStatus  # 用于取消导航
from geometry_msgs.msg import Quaternion
from std_msgs.msg import Int8  # 状态消息类型
import math

class NavigationClient(Node):
    def __init__(self):
        super().__init__('navigation_client')

        # 创建 Action 客户端
        self.client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # 订阅 STM32 数据
        self.subscription = self.create_subscription(
            Referee,
            'stm32_ros2_data',
            self.condition_callback,
            10
        )

        # 发布导航状态（1=导航中，0=未导航）
        self.status_publisher = self.create_publisher(Int8, 'nav2_status', 10)

        # 每秒发布一次状态，防止通信错误导致误判
        self.timer = self.create_timer(1.0, self.publish_status)

        # 目标点集合
        self.target_points = {
            1: (2.0, 3.0, 0.0),
            2: (4.0, 1.0, 1.57),
            3: (1.0, -2.0, -1.57),
            4: (-3.0, 4.0, 0.0),
            5: (-2.0, -3.0, 1.57),
            6: (5.0, 0.0, -1.57),
            7: (-5.0, 5.0, 3.14),
            8: (0.0, -5.0, 0.0),
            9: (3.0, 3.0, 1.0),
            10: (-3.0, -3.0, -1.0)
        }

        self.current_condition = 0  # 记录当前目标编号
        self.current_goal_handle = None  # 记录当前导航状态
        self.is_navigating = False  # 当前导航状态

    def send_goal(self, x, y, yaw):
        """发送导航目标点"""
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = "map"
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.orientation = self.yaw_to_quaternion(yaw)  # 转换成四元数

        self.get_logger().info(f"发送目标点: x={x}, y={y}, yaw={yaw}")

        self.client.wait_for_server(timeout_sec=5.0)
        future = self.client.send_goal_async(goal_msg)
        future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        """处理导航目标的接受情况"""
        goal_handle = future.result()
        if not goal_handle or not goal_handle.accepted:
            self.get_logger().warn("导航目标未被接受")
            self.is_navigating = False  # 未导航
            return
        
        self.current_goal_handle = goal_handle
        self.get_logger().info("导航目标已被接受，开始导航")
        self.is_navigating = True  # 进入导航状态
        
        # 监听导航任务完成情况
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.goal_result_callback)

    def goal_result_callback(self, future):
        """处理导航完成或失败"""
        status = future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info("导航成功到达目标点")
        else:
            self.get_logger().warn(f"导航失败，状态码: {status}")

        self.is_navigating = False  # 任务结束
        self.current_goal_handle = None  # 清除当前导航任务

    def cancel_goal(self):
        """取消当前导航"""
        if self.current_goal_handle is None:
            return  # 没有目标，不需要取消

        cancel_future = self.current_goal_handle.cancel_goal_async()
        cancel_future.add_done_callback(self.cancel_done_callback)

    def cancel_done_callback(self, future):
        """取消任务完成后的回调"""
        self.get_logger().info("导航目标已取消")
        self.is_navigating = False  # 取消后进入未导航状态
        self.current_goal_handle = None  # 清除当前任务

    def condition_callback(self, msg):
        """接收 STM32 传来的数据，判断是否需要导航"""
        new_condition = msg.game_progress  # 这里用 game_progress 作为条件
        self.get_logger().info(f"收到条件: {new_condition}")

        if new_condition == 0:
            self.get_logger().info("无目标，保持当前状态")
            return

        if new_condition not in self.target_points:
            self.get_logger().warn(f"条件 {new_condition} 不在定义的目标范围内")
            return

        if new_condition == self.current_condition:
            self.get_logger().info("目标点未变化，不重新发送")
            return

        # 目标发生变化，取消当前导航
        self.cancel_goal()
        self.current_condition = new_condition  # 更新当前目标编号
        x, y, yaw = self.target_points[new_condition]
        self.send_goal(x, y, yaw)

    def publish_status(self):
        """每秒发布导航状态 (1=导航中, 0=未导航)"""
        msg = Int8()
        msg.data = 1 if self.is_navigating else 0
        self.status_publisher.publish(msg)
        self.get_logger().info(f"定期发布导航状态: {msg.data}")

    def yaw_to_quaternion(self, yaw):
        """将 yaw 角转换为四元数"""
        q = Quaternion()
        q.z = math.sin(yaw / 2.0)
        q.w = math.cos(yaw / 2.0)
        return q

def main(args=None):
    rclpy.init(args=args)
    nav_client = NavigationClient()
    rclpy.spin(nav_client)  # 让节点一直运行，等待消息
    nav_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
