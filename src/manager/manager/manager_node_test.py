import collections
from threading import Thread, Lock
from transitions.extensions import HierarchicalGraphMachine
from transitions.extensions.states import Timeout, add_state_features
import rclpy
from rclpy.executors import (
    ExternalShutdownException,
    MultiThreadedExecutor,
    SingleThreadedExecutor,
)
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from std_srvs.srv import Trigger
import re
import sys
from lifecycle_msgs.srv import ChangeState, GetState
from lifecycle_msgs.msg import Transition
from custom_interfaces.srv import CheckNode
from time import sleep
from functools import partial
from rclpy.task import Future

print("Program started")


@add_state_features(Timeout)
class CustomStateMachine(HierarchicalGraphMachine):
    pass


class Manager_System(Node):
    def __init__(self):
        super().__init__("manager")
        self.node_list = ["lc_talker", "lc_listener"]
        self.all_cb_group = ReentrantCallbackGroup()
        self.timer_cb_group = MutuallyExclusiveCallbackGroup()
        self.get_cb_group = MutuallyExclusiveCallbackGroup()

        # Initialize the change_state_client that will update the talker state
        self.change_talking_state_cli = self.create_client(
            ChangeState,
            "/lc_talker/change_state",
            callback_group=self.all_cb_group,
        )

        # Initialize the change_state_client that will update the listener state
        self.change_listening_state_cli = self.create_client(
            ChangeState,
            "/lc_listener/change_state",
            callback_group=self.all_cb_group,
        )

        # Initialize the get_state_client that will get the talker state
        self.get_talking_state_cli = self.create_client(
            GetState,
            "/lc_talker/get_state",
            callback_group=self.all_cb_group,
        )

        # Initialize the get_state_client that will get the listener state
        self.get_listening_state_cli = self.create_client(
            GetState,
            "/lc_listener/get_state",
            callback_group=self.all_cb_group,
        )

        # Call the on_configure state
        while (
            not self.change_talking_state_cli.wait_for_service(timeout_sec=1.0)
            or not self.change_listening_state_cli.wait_for_service(timeout_sec=1.0)
            or not self.get_talking_state_cli.wait_for_service(timeout_sec=1.0)
            or not self.get_listening_state_cli.wait_for_service(timeout_sec=1.0)
        ):
            self.get_logger().info(
                "Get and Set States Services not available, waiting again..."
            )

        # Configure talking and listening nodes
        res1 = self.set_state("lc_talker", "configure")
        res2 = self.set_state("lc_listener", "configure")

        if not (res1 and res2):
            self.set_state("lc_talker", "shutdown")
            self.set_state("lc_listener", "shutdown")

            sys.exit()

        self.call_timer = self.create_timer(
            2.0,
            partial(self.cb_checker_nodestillactive),
            callback_group=self.all_cb_group,
        )

    # Monitoring nodes
    async def cb_checker_nodestillactive(self):
        """
        Monitoring node every 5s
        """
        self.get_logger().warn("Checker service called!")

        res = True


class Manager(Manager_System, object):
    # Define some states.
    states = [
        "configure1",
        {
            "name": "activate1",
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
            "trigger": "gotoactivate",
            "source": "configure1",
            "dest": "activate1",
            "conditions": "isconfigured",
        },
        {
            "trigger": "gotowork",
            "source": "activate1",
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
        super().__init__()

        # Enregistrement des fonctions d'état
        # History of the state machine
        self.state_history = collections.deque(maxlen=2)

        # Initialize the state machine
        self.machine = CustomStateMachine(
            model=self,
            states=Manager.states,
            transitions=Manager.transitions,
            initial="configure1",
        )

        self._count = 0

    @property
    def state(self):
        return self.state_history[-1]

    @state.setter
    def state(self, value):
        self.state_history.append(value)

    def log_error(self):
        print(f"Error at {self.state} going shut down nodes!")

    # State getter / setter
    def get_state(self, nodename: str) -> str:
        """_summary_

        Args:
            nodename (str): _description_

        Returns:
            str: _description_
        """

        self.get_logger().debug(f"Get {nodename} state!")

        # request
        self.req = GetState.Request()

        if nodename == "lc_talker":
            future = self.get_talking_state_cli.call_async(self.req)
        elif nodename == "lc_listener":
            future = self.get_listening_state_cli.call_async(self.req)

        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

        if future.result() is not None:
            self.get_logger().debug(
                f"{nodename} in current state: {future.result().current_state}!"
            )
            return future.result().current_state
        else:
            return None

    def set_state(self, nodename: str, state: str) -> bool:
        """_summary_

        Args:
            nodename (str): _description_
            state (str): _description_

        Returns:
            bool: _description_
        """

        self.get_logger().info(f"Change {nodename} to state {state}")

        # Transition request
        self.req = ChangeState.Request()
        self.req.transition = Transition(label=state)

        if nodename == "lc_talker":
            future = self.change_talking_state_cli.call_async(self.req)
        elif nodename == "lc_listener":
            future = self.change_listening_state_cli.call_async(self.req)

        rclpy.spin_until_future_complete(self, future)

        if not future.result().success:
            self.get_logger().error(f"Error while setting {nodename} to state {state}!")

        return future.result().success

    # Transition checking
    def isconfigured(self) -> bool:
        """Check if all the nodes are configured

        Returns:
            bool: true if every node are configured, false otherwise
        """
        state1 = self.get_state("lc_talker")
        state2 = self.get_state("lc_listener")

        # self.get_logger().error(f"State talker {state1} and listener: {state2}")

        # return (state1 != "unconfigured") and (state2 != "unconfigured")

        return True

    def isactivated(self) -> bool:
        """Check if all the nodes are active

        Returns:
            bool: true if every node are active, false otherwise
        """

        # state1 = self.get_state("lc_talker")
        # state2 = self.get_state("lc_listener")

        # return (state1 != "active") and (state2 != "active")
        return True

    def stateMachine(self) -> None:
        self.get_logger().debug("State:{}".format(self.state))

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

    def STATE_CONFIGURE(self):
        self.gotoactivate()

    def STATE_ACTIVATE(self):
        if self._count < 1:
            # self.set_state("lc_talker", "activate")
            # self.set_state("lc_listener", "activate")
            self._count += 1

        self.gotowork()

    def STATE_WORKING(self):
        pass

    def STATE_SHUTDOWN(self):
        self.set_state("lc_talker", "shutdown")
        self.set_state("lc_listener", "shutdown")

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

    manager_node = Manager()
    executor = MultiThreadedExecutor()
    executor.add_node(manager_node)

    # Start the ROS2 node on a separate thread
    # thread = Thread(target=spin_node,args=(executor,)) # communinication not working if doing like that
    thread = Thread(target=executor.spin)

    # Let the app running on the main thread
    try:
        thread.start()
        manager_node.get_logger().info("Spinned ROS2 Node. . .")
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
