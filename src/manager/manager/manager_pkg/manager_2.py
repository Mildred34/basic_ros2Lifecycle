import collections
import re
import time
from transitions.extensions import HierarchicalGraphMachine
from transitions.extensions.states import Timeout, add_state_features
import sys
import rclpy
from rclpy.task import Future
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import (
    ExternalShutdownException,
    MultiThreadedExecutor,
)
from functools import partial
from lifecycle_msgs.srv import ChangeState, GetState
from lifecycle_msgs.msg import Transition
from std_srvs.srv import Trigger
from custom_interfaces.srv import CheckNode
from time import sleep
from threading import Thread, Event


@add_state_features(Timeout)
class CustomStateMachine(HierarchicalGraphMachine):
    pass


class Manager(Node, object):
    # Define some states.
    states = [
        "init",
        "configure",
        {
            "name": "activate",
        },
        {
            "name": "working",
        },
        {
            "name": "shutdown",
        },
        {
            "name": "error",
        },
    ]

    # Transitions
    transitions = [
        # init
        {
            "trigger": "gotoconfigure",
            "source": "init",
            "dest": "configure",
        },
        {
            "trigger": "activate",
            "source": "configure",
            "dest": "activate",
            "conditions": "isconfigured",
        },
        {
            "trigger": "gotowork",
            "source": "activate",
            "dest": "working",
            "conditions": "isactivated",
        },
        {
            "trigger": "end",
            "source": "working",
            "dest": "shutdown",
        },
        {
            "trigger": "gotanerror",
            "source": "*",
            "dest": "error",
            "before": "log_error",
        },
        {
            "trigger": "shutdown",
            "source": "error",
            "dest": "shutdown",
        },
    ]

    def __init__(self):
        super().__init__("manager")

        self.node_list = ["lc_talker2"]

        # Callback groups
        self.all_cb_group = ReentrantCallbackGroup()

        # Initialize the change_state_client that will update the talker state
        self.change_talking_state_cli = self.create_client(
            ChangeState, "/lc_talker2/change_state"
        )

        # Initialize the get_state_client that will get the talker state
        self.get_talking_state_cli = self.create_client(GetState, "/lc_talker2/get_state")

        # Call the on_configure state
        while not self.change_talking_state_cli.wait_for_service(
            timeout_sec=1.0
        ) or not self.get_talking_state_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(
                "Get and Set States Services not available, waiting again..."
            )

        # Client to check if node is done
        self.done_checker = []
        for node in self.node_list:
            srv_name = "/" + node + "/isdone"
            self.done_checker.append(
                self.create_client(
                    Trigger,
                    srv_name,
                    callback_group=self.all_cb_group,
                )
            )

            while not self.done_checker[-1].wait_for_service(timeout_sec=1.0):
                self.get_logger().info(
                    f"Endind node Services named {srv_name} not available, waiting again..."
                )

        # client for node checker
        self.checker_node_cli = self.create_client(
            CheckNode,
            "/check_node",
            callback_group=self.all_cb_group,
        )

        while not self.checker_node_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(
                "Nodes checker services not available, waiting again..."
            )

        self.call_timer = self.create_timer(
            1.0, self.cb_checker_nodestillactive, callback_group=self.all_cb_group
        )

        # Enregistrement des fonctions d'état
        # History of the state machine
        self.state_history = collections.deque(maxlen=2)

        # Initialize the state machine
        self.machine = CustomStateMachine(
            model=self,
            states=Manager.states,
            transitions=Manager.transitions,
            initial="init",
        )

    @property
    def state(self):
        return self.state_history[-1]

    @state.setter
    def state(self, value):
        self.state_history.append(value)

    def log_error(self):
        print(f"Error at {self.state} going shut down nodes!")

    # Monitoring nodes
    async def cb_checker_nodestillactive(self):
        """
        Monitoring node every 5s
        """
        self.get_logger().warn("Checker service called!")

        res = True

        # Check if nodes are still alive
        for node in self.node_list:
            req = CheckNode.Request()
            req.name = node

            future = self.checker_node_cli.call_async(req)

            try:
                response = await future
            except Exception as e:
                self.get_logger().info("Service call failed %r" % (e,))
            else:
                self.get_logger().info("Result : %s" % (response.success))

                if not response.success:
                    res = response.success
                    break

        if not res:
            self.gotanerror()
            return

        # Check if node's action are done
        for i, node in enumerate(self.node_list):
            req = Trigger.Request()

            future = self.done_checker[i].call_async(req)

            try:
                response = await future
            except Exception as e:
                self.get_logger().info("Service call failed %r" % (e,))
            else:
                self.get_logger().debug("Result : %s" % (response.success))
                res = response.success

            if res:
                self.get_logger().warn(f"Node {node} Done! ")
                self.node_list.remove(node)

        if len(self.node_list) == 0:
            self.call_timer.destroy()
            self.end()

    # State getter / setter
    def get_node_state(self, nodename: str) -> str:
        """_summary_

        Args:
            nodename (str): _description_

        Returns:
            str: _description_
        """
        event = Event()

        def _get_node_state_result(nodename: str, future: Future):
            if future.result() is not None:
                self.get_logger().debug(
                    f"{nodename} in current state: {future.result().current_state.label}!"
                )
            nonlocal event
            event.set()

        self.get_logger().debug(f"Get {nodename} state!")

        # request
        self.req = GetState.Request()

        if nodename == "lc_talker2":
            future = self.get_talking_state_cli.call_async(self.req)
            future.add_done_callback(partial(_get_node_state_result, nodename))
            event.wait()

        if future.result() is not None:
            self.get_logger().debug(
                f"{nodename} in current state: {future.result().current_state.label}!"
            )
            return future.result().current_state.label
        else:
            return None

    def set_node_state(self, nodename: str, state: str) -> bool:
        """_summary_

        Args:
            nodename (str): _description_
            state (str): _description_

        Returns:
            bool: _description_
        """
        event1 = Event()

        def _set_node_state_result(future: Future):
            nonlocal event1
            event1.set()

        self.get_logger().info(f"Change {nodename} to state {state}")

        # Transition request
        self.req = ChangeState.Request()
        self.req.transition = Transition(label=state)

        if nodename == "lc_talker2":
            future = self.change_talking_state_cli.call_async(self.req)
            future.add_done_callback(partial(_set_node_state_result))
            event1.wait()

        if not future.result().success:
            self.get_logger().error(f"Error while setting {nodename} to state {state}!")

        return future.result().success

    # Special function exit/start state machine
    def on_enter_configure(self) -> None:
        self.get_logger().info("Manager is going to configure talker..")

        self.set_node_state("lc_talker2", "configure")

    def on_enter_activate(self) -> None:
        self.get_logger().info("Manager is going to activate talker...")

        self.set_node_state("lc_talker2", "activate")

    def on_enter_working(self) -> None:
        self.get_logger().info("Nodes are now working !!!")

    # Transition checking
    def isconfigured(self) -> bool:
        """Check if all the nodes are configured

        Returns:
            bool: true if every node are configured, false otherwise
        """

        state1 = self.get_node_state("lc_talker2")

        return state1 != "unconfigured"

    def isactivated(self) -> bool:
        """Check if all the nodes are active

        Returns:
            bool: true if every node are active, false otherwise
        """

        state1 = self.get_node_state("lc_talker2")

        return state1 == "active"

    def stateMachine(self) -> None:
        self.get_logger().debug("State:{}".format(self.state))

        # INIT STATE
        if re.search("^init", self.state) is not None:
            self.STATE_INIT()

        # CONFIGURE STATE
        if re.search("^configure", self.state) is not None:
            self.STATE_CONFIGURE()

        # ACTIVATE STATE
        if re.search("^activate", self.state) is not None:
            self.STATE_ACTIVATE()

        # WORKING STATE
        if re.search("^working", self.state) is not None:
            self.STATE_WORKING()

        # SHUTDOWN STATE
        if re.search("^shutdown", self.state) is not None:
            self.STATE_SHUTDOWN()

        # ERROR STATE
        if re.search("^error", self.state) is not None:
            self.STATE_ERROR()

    def STATE_INIT(self):
        self.gotoconfigure()

    def STATE_CONFIGURE(self):
        self.activate()

    def STATE_ACTIVATE(self):
        self.gotowork()

    def STATE_WORKING(self):
        pass

    def STATE_SHUTDOWN(self):
        self.set_node_state("lc_talker2", "shutdown")
        sleep(1.0)
        sys.exit()

    def STATE_ERROR(self):
        self.shutdown()

    def exec(self):
        while True:
            try:
                self.stateMachine()
            except KeyboardInterrupt:
                print("You pressed Ctrl+C!")
                break

        return 0


def main(args=None):
    rclpy.init(args=args)

    # Create the node
    manager_node = Manager()
    executor = MultiThreadedExecutor()
    executor.add_node(manager_node)

    # Start the ROS2 node on a separate thread
    # thread = Thread(target=spin_node,args=(executor,)) # communinication not working if doing like that
    thread1 = Thread(target=executor.spin)

    try:
        # Spin the node so the callback function is called.
        thread1.start()

        manager_node.get_logger().info("Spinned ROS2 Manager 2 Node . . .")
        manager_node.exec()

    except (KeyboardInterrupt, ExternalShutdownException):
        # If the node receive a KeyboardInterrupt command
        pass

    finally:
        # Destroy the node explicitly
        # (optional - Done automatically when node is garbage collected)
        manager_node.destroy_node()
        executor.shutdown()

    try:
        thread1.join()
    except KeyboardInterrupt:
        pass

    # rclpy.shutdown()


if __name__ == "__main__":
    main()
