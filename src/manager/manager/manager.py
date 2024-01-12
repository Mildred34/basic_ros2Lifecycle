import collections
import re
from transitions.extensions import HierarchicalGraphMachine
from transitions.extensions.states import Timeout, add_state_features
import sys
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor, SingleThreadedExecutor

from lifecycle_msgs.srv import ChangeState, GetState
from lifecycle_msgs.msg import Transition
from custom_interfaces.srv import CheckNode

from threading import Thread

@add_state_features(Timeout)
class CustomStateMachine(HierarchicalGraphMachine):
    pass
  
class Manager(Node, object):
# Define some states.
    states = [
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
            "conditions": "workisdone",
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
        super().__init__('manager')
        
        self.node_list = ["lc_talker","lc_listener"]

        # Initialize the change_state_client that will update the talker state
        self.change_talking_state_cli = self.create_client(ChangeState, '/lc_talker/change_state')
        
        # Initialize the change_state_client that will update the listener state
        self.change_listening_state_cli = self.create_client(ChangeState, '/lc_listener/change_state')
        
        
        # Initialize the get_state_client that will get the talker state
        self.get_talking_state_cli = self.create_client(GetState, '/lc_talker/get_state')
        
        # Initialize the get_state_client that will get the listener state
        self.get_listening_state_cli = self.create_client(GetState, '/lc_listener/get_state')
        

        # Call the on_configure state
        while   not self.change_talking_state_cli.wait_for_service(timeout_sec=1.0) or \
                not self.change_listening_state_cli.wait_for_service(timeout_sec=1.0) or \
                not self.get_talking_state_cli.wait_for_service(timeout_sec=1.0) or \
                not self.get_listening_state_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Services not available, waiting again...")
        
        # Configure talking and listening nodes
        res1 = self.set_state("lc_talker","configure")
        res2 = self.set_state("lc_listener","configure")
        
        if not (res1 and res2):
            
            self.set_state("lc_talker","shutdown")
            self.set_state("lc_listener","shutdown")
            
            sys.exit()
            
        self.client_cb_group = MutuallyExclusiveCallbackGroup()
        self.timer_cb_group = MutuallyExclusiveCallbackGroup()
        
        # client for node checker
        self.checker_node_cli = self.create_client(
            CheckNode,
            "/check_node",
            callback_group=self.client_cb_group,
        )
        
        while not self.checker_node_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Nodes checker services not available, waiting again...")
        
        self.call_timer = self.create_timer(
            5.0, self.cb_checker_nodestillactive,
            callback_group=self.timer_cb_group
        )
    
        # Enregistrement des fonctions d'état
        # History of the state machine
        self.state_history = collections.deque(maxlen=2)

        # Initialize the state machine
        self.machine = CustomStateMachine(
            model=self,
            states=Manager.states,
            transitions=Manager.transitions,
            initial="configure",
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
        self.get_logger().info("Checker service called!")
        
        res = False
        
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

            if future.done():
                res = future.result().success
                
                if not res:
                    break
                
        if not res:
            self.gotanerror()
        
    # State getter / setter
    def get_state(self, nodename : str) -> str:
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
            self.get_logger().debug(f"{nodename} in current state: {future.result().current_state}!")     
            return future.result().current_state
        else:
            return None
    
    def set_state(self, nodename : str, state : str) -> bool:
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
    
    # Special function exit/start state machine
    def on_enter_activate(self) -> None:
        self.get_logger().info(
            "Manager is going to activate talker and listener..."
        )
        
        self.set_state("lc_talker","activate")
        self.set_state("lc_listener","activate")
        
    def on_enter_working(self) -> None:
        self.get_logger().info(
            "Nodes are now working !!!"
        )
        
    # Transition checking
    def isconfigured(self) -> bool:
        """Check if all the nodes are configured

        Returns:
            bool: true if every node are configured, false otherwise
        """
        
        state1 = self.get_state("lc_talker")
        state2 = self.get_state("lc_listener")
        
        return (state1 != "unconfigured") and (state2 != "unconfigured")
 
    def isactivated(self) -> bool:
        """Check if all the nodes are active

        Returns:
            bool: true if every node are active, false otherwise
        """
        
        state1 = self.get_state("lc_talker")
        state2 = self.get_state("lc_listener")
        
        return (state1 != "active") and (state2 != "active")

    def workisdone(self) -> bool:
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
      self.activate()
    
    def STATE_ACTIVATE(self):
      self.gotowork()
    
    def STATE_WORKING(self):
        pass
    
    def STATE_SHUTDOWN(self):
        
        self.set_state("lc_talker","shutdown")
        self.set_state("lc_listener","shutdown")
        
        sys.exit()
    
    def STATE_ERROR(self):
        self.shutdown()
  
    # async def gui_callback(self):
    #     self.event, self.values = self.window.read(timeout=10)

    #     if self.event in ('Exit', 'None'):
    #         exit()

    #     elif self.event == 'Talk':
    #         ######  Here start the talker like it would with `ros2 run py_pubsub talker`

    #         # Update the transition according to current button state
    #         next_talking_transition = 'activate' if not self.is_talking else 'deactivate'

    #         # Create the change state object
    #         self.req.transition = Transition(label=next_talking_transition)

    #         # Call to the service
    #         future_lifecycle = self.change_talking_state_cli.call_async(self.req)
    #         result_change_state = await future_lifecycle

    #         # If the request was successful, update the streaming status
    #         if result_change_state.success:
    #             self.is_talking = not self.is_talking
    #             button_color = sg.theme_button_color() if not self.is_talking else 'red'
    #             self.window['Talk'].update(button_color=button_color)
    
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
    thread = Thread(target=executor.spin)
    
    try:
        # Spin the node so the callback function is called.
        thread.start()
        
        manager_node.get_logger().info("Spinned ROS2 Node . . .")
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
        thread.join()
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()
    
    
if __name__ == '__main__':
    main()