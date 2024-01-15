import socket

import rclpy
from custom_interfaces.srv import CheckNode
from rclpy.node import Node


class NodeCheckerService(Node):
    def __init__(self):
        super().__init__("NodeChecker_service_node")
        self.srv = self.create_service(
            CheckNode, "check_node", self.node_checker_callback
        )

    async def node_checker_callback(self, request, response):
        self.get_logger().info(f"Node to check: {request.name}")

        node_names = Node.get_node_names(self)

        if request.name in node_names:
            response.success = True
        else:
            self.get_logger().error(f"Node {request.name} Failed! ")
            response.success = False

        return response


def main(args=None):
    rclpy.init(args=args)

    SimChecker_service = NodeCheckerService()

    rclpy.spin(SimChecker_service)

    rclpy.shutdown()


if __name__ == "__main__":
    main()
