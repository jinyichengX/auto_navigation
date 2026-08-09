#下载小说并发布

import rclpy
from rclpy.node import Node
import requests
from example_interfaces.msg import String
from queue import Queue

class novel_pub_node(Node):
    def __init__(self, node_name):
        super().__init__(node_name)
        self.get_logger().info(f'{node_name},start!')
        self.novel_queue = Queue()
        self.novel_publisher = self.create_publisher(String, 'publish_novel', 10)
        self.create_timer(5, self.timer_callback) # period : 5s

    def timer_callback(self):
        if self.novel_queue.qsize() > 0: #if/while都可以
            line = self.novel_queue.get()# 这个队列操作是在定时器线程执行的，有线程安全问题
            msg = String()
            msg.data = line
            self.novel_publisher.publish(msg)  
            self.get_logger().info(f'发布了{msg}')

    def download_novel(self, url):
        response = requests.get(url) #一次性下载完才会执行下一行
        response.encoding = 'utf-8'
        text = response.text #response.text是内存缓冲区
        self.get_logger().info(f'下载{url},{len(text)}')
        for line in text.splitlines():
            self.novel_queue.put(line) # 这个队列操作是在main主线程执行的，有线程安全问题

def main():
    rclpy.init()
    node = novel_pub_node('novel_pub_node1')
    node.download_novel("http://localhost:8000/core/node1/node1/novel/chapter1.txt")
    rclpy.spin(node)
    rclpy.shutdown()