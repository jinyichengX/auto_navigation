import rclpy
from rclpy.node import Node
from sum_srv_interface.srv import Sum

class SumCalc(Node):
    def __init__(self, name):
        super().__init__(name)
        self.service = self.create_service(Sum, 'sum', self.calc_sum)

    def calc_sum(self, request, response):
        response.sum = request.a + request.b
        return response

def main():
    rclpy.init()
    sum_calc = SumCalc('sum_calc')
    rclpy.spin(sum_calc)
    rclpy.shutdown()