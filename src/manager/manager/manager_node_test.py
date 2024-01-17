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
from threading import Event

print("Program started")


@add_state_features(Timeout)
class CustomStateMachine(HierarchicalGraphMachine):
    pass


class Manager_System(Node):
    def __init__(self):
        super().__init__("manager")
        self.node_list = ["lc_talker"]
        self.all_cb_group = ReentrantCallbackGroup()

        # Initialize the change_state_client that will update the talker state
        self.change_talking_state_cli = self.create_client(
            ChangeState, "/lc_talker/change_state", callback_group=self.all_cb_group
        )

        # Initialize the get_state_client that will get the talker state
        self.get_talking_state_cli = self.create_client(GetState, "/lc_talker/get_state",callback_group=self.all_cb_group)

        # Call the on_configure state
        while not self.change_talking_state_cli.wait_for_service(
            timeout_sec=1.0
        ) or not self.get_talking_state_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(
                "Get and Set States Services not available, waiting again..."
            )

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
            "trigger": "gotoactivate",
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
        super().__init__()
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

    # State getter / setter
    def get_node_state(self, nodename: str) -> str:
        """_summary_

        Args:
            nodename (str): _description_

        Returns:
            str: _description_
        """
        event=Event()
        
        def _get_node_state_result(nodename: str, future: Future):
            if future.result() is not None:
                self.get_logger().info(
                    f"{nodename} in current state: {future.result().current_state.label}!"
                )
            nonlocal event
            event.set()

        
        self.get_logger().info(f"Get {nodename} state!")

        # request
        self.req = GetState.Request()

        if nodename == "lc_talker":
            future: Future = self.get_talking_state_cli.call_async(self.req)
            future.add_done_callback(partial(_get_node_state_result,nodename))
            event.wait()
            
        self.get_logger().info(f"{nodename} state is: {future.result().current_state.label}!")
            
        return future.result().current_state.label
            
            
    def set_node_state(self, nodename: str, state: str) -> bool:
        """_summary_

        Args:
            nodename (str): _description_
            state (str): _description_

        Returns:
            bool: _description_
        """
        event1=Event()
        
        def _set_node_state_result(future: Future):
            
            # if not future.result().success:
            #     self.get_logger().error(f"Error while setting {nodename} to state {state}!")
            # else:
            #     self.get_logger().info(f"Success while setting {nodename} to state {state}!")
                
            nonlocal event1
            event1.set()
    
        self.get_logger().info(f"Change {nodename} to state {state}")

        # Transition request
        self.req = ChangeState.Request()
        self.req.transition = Transition(label=state)

        if nodename == "lc_talker":
            future: Future = self.change_talking_state_cli.call_async(self.req)
            future.add_done_callback(partial(_set_node_state_result))
            event1.wait()
            
        return future.result().success
    

    # Special function exit/start state machine
    def on_enter_configure(self) -> None:
        self.get_logger().info("Manager is going to configure talker and listener...")

        self.set_node_state("lc_talker", "configure")
        
    def on_enter_activate(self) -> None:
        self.get_logger().info("Manager is going to activate talker and listener...")

        self.set_node_state("lc_talker", "activate")
        
        
    # Transition checking
    def isconfigured(self) -> bool:
        """Check if all the nodes are configured

        Returns:
            bool: true if every node are configured, false otherwise
        """
        state1 = self.get_node_state("lc_talker")

        return state1 != "unconfigured"

    def isactivated(self) -> bool:
        """Check if all the nodes are active

        Returns:
            bool: true if every node are active, false otherwise
        """

        state1 = self.get_node_state("lc_talker")

        return state1 != "active"

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
        if self.isconfigured():
            self.gotoactivate()

    def STATE_ACTIVATE(self):
        sleep(0.5)

        if self.isactivated():
            self.gotowork()

    def STATE_WORKING(self):
        pass

    def STATE_SHUTDOWN(self):
        self.set_node_state("lc_talker", "shutdown")

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
