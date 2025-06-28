import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json

class BatteryNode(Node):
    def __init__(self, queue, device_id):
        self.queue = queue
        self.device_id = device_id
        super().__init__('Battery_Logo_node_' + device_id)
        self.subscription = self.create_subscription(String, 'gv_battery_' + device_id, self.battery_callback, 10)
        self.get_logger().info("Battery 订阅者已启动")

    def battery_callback(self, msg):
        try:
            bat = json.loads(msg.data)
            self.queue.put(bat)
            status = "🔌充电中" if bat["current"] > 0 else "🔋放电中"
            self.get_logger().info(
                f"[{status}] 电量: {bat['capacity_percentage']}%, 电压: {bat['voltage']:.1f}V, 电流: {bat['current']:.2f}A"
            )
        except Exception as e:
            self.get_logger().error(f"电池数据解析失败: {e}\n原始数据: {msg.data}")

