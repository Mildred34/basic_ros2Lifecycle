# import sys
from threading import Thread

import rclpy
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor

from .manager_pkg.manager_2 import Manager as manager

print("Program started")


def spin_node(executor):
    try:
        executor.spin
    except SystemExit as e:
        rclpy.logging.get_logger("Quitting_Node").error(
            "Error happened of type: {}\n msg: {} \nEnd of testing grasp!".format(
                type(e), e
            )
        )
    except KeyboardInterrupt:
        rclpy.logging.get_logger("Quitting_Node").info("End of testing grasp!")
    except ExternalShutdownException:
        rclpy.logging.get_logger("Quitting_Node").info("External shutdown exception!")
    finally:
        executor.shutdown()


def main(args=None):
    rclpy.init(args=args)

    manager_node = manager()
    executor = MultiThreadedExecutor()
    executor.add_node(manager_node)

    # Start the ROS2 node on a separate thread
    # thread = Thread(target=spin_node,args=(executor,)) # communinication not working if doing like that
    thread = Thread(target=executor.spin)

    # Let the app running on the main thread
    try:
        thread.start()
        manager_node.get_logger().info("Spinned ROS2 manager 2 Node. . .")
        manager_node.exec()

    except SystemExit as e:
        rclpy.logging.get_logger("Quitting").error(
            "Error happened of type: {}\n msg: {} \nEnding program".format(type(e), e)
        )
    except KeyboardInterrupt:
        rclpy.logging.get_logger("Quitting").info("Ending program!")
    finally:
        manager_node.get_logger().info("Shutting down ROS2 Node . . .")
        manager_node.destroy_node()
        executor.shutdown()

    try:
        thread.join()
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
