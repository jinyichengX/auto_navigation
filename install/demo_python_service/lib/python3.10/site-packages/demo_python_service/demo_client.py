import rclpy
from rclpy.node import Node
from sum_srv_interface.srv import Sum

class Client(Node):
    def __init__(self, name):
        super().__init__(name)
        self.client = self.create_client(Sum, 'sum')

    def send_request(self):
        #判断服务是否可用
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Service not available, waiting...")
        request = Sum.Request()
        request.a = 1
        request.b = 2
        future = self.client.call_async(request)
        future.add_done_callback(self.handle_response)

    def handle_response(self, future):
        response = future.result()
        self.get_logger().info(f"Sum: {response.sum}")

def main():
    rclpy.init()
    client = Client('req_sum')
    client.send_request()
    rclpy.spin(client)
    rclpy.shutdown()