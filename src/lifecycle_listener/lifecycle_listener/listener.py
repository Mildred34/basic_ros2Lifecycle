# Copyright 2021 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from typing import Optional

import rclpy

# Node, State and Publisher are aliases for LifecycleNode, LifecycleState and LifecyclePublisher
# respectively.
# In case of ambiguity, the more explicit names can be imported.

from rclpy.lifecycle import Node
from rclpy.lifecycle import State
from rclpy.lifecycle import TransitionCallbackReturn
from rclpy.timer import Timer
from std_srvs.srv import Trigger

import std_msgs.msg


class LifecycleListener(Node):
    """Our lifecycle listener node."""

    def __init__(self, node_name, **kwargs):
        """Construct the node."""
        super().__init__(node_name, **kwargs)
        self._count: int = 0
        self._qos_policy = rclpy.qos.QoSProfile(  # reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
            # history=rclpy.qos.HistoryPolicy.KEEP_LAST,
            depth=1,
            # durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL
        )

        self._sub = None

        # Service for checking if node is done
        self._done = False
        self.srv = self.create_service(
            Trigger, "/" + node_name + "/isdone", self.isdone_callback
        )

    async def isdone_callback(self, request, response):
        """Check if node is done

        Args:
            request (Trigger.Request): Empty
            response (Trigger.Response): Boolean + String

        Returns:
            Trigger.Response: Boolean + string
        """
        if self._done:
            response.success = True
        else:
            response.success = False
        return response

    def listener_cb(self, msg):
        """Listen a new message when enabled."""
        self._count += 1

        # We independently from the current state call publish on the lifecycle
        # publisher.
        # Only if the listener is in an active state, the message transfer is
        # enabled and the message actually logged.
        if self._sub is None:
            self.get_logger().info(
                "Lifecycle listener is currently inactive. Messages are not listened."
            )
        else:
            self.get_logger().debug(
                f"Lifecycle listener is active. Listening: [{msg.data}]"
            )

        if self._count > 1:
            self.done_ = True

    def on_configure(self, state: State) -> TransitionCallbackReturn:
        """
        Configure the node, after a configuring transition is requested.

        on_configure callback is being called when the lifecycle node
        enters the "configuring" state.

        :return: The state machine either invokes a transition to the "inactive" state or stays
        in "unconfigured" depending on the return value.
          TransitionCallbackReturn.SUCCESS transitions to "inactive".
          TransitionCallbackReturn.FAILURE transitions to "unconfigured".
          TransitionCallbackReturn.ERROR or any uncaught exceptions to "errorprocessing"
        """
        self._sub = self.create_subscription(
            std_msgs.msg.String,
            "lifecycle_chatter",
            self.listener_cb,
            self._qos_policy,
        )

        self.get_logger().info("on_configure() is called.")
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        # Differently to rclcpp, a lifecycle publisher transitions automatically between the inactive and
        # enabled state and viceversa.
        # For that reason, we only need to write an on_configure() and on_cleanup() callbacks, and we don't
        # need to write on_activate()/on_deactivate() callbacks.

        # Log, only for demo purposes
        self.get_logger().info("on_activate() is called.")

        # The default LifecycleNode callback is the one transitioning
        # LifecyclePublisher entities from inactive to enabled.
        # If you override on_activate(), don't forget to call the parent class method as well!!
        return super().on_activate(state)

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        self._sub = None
        # Log, only for demo purposes
        self.get_logger().info("on_deactivate() is called.")
        # Same reasong here that for on_activate().
        # These are the two only cases where you need to call the parent method.
        return super().on_deactivate(state)

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        """
        Cleanup the node, after a cleaning-up transition is requested.

        on_cleanup callback is being called when the lifecycle node
        enters the "cleaning up" state.

        :return: The state machine either invokes a transition to the "unconfigured" state or stays
        in "inactive" depending on the return value.
          TransitionCallbackReturn.SUCCESS transitions to "unconfigured".
          TransitionCallbackReturn.FAILURE transitions to "inactive".
          TransitionCallbackReturn.ERROR or any uncaught exceptions to "errorprocessing"
        """
        self.get_logger().info("on_cleanup() is called.")

        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        """
        Shutdown the node, after a shutting-down transition is requested.

        on_shutdown callback is being called when the lifecycle node
        enters the "shutting down" state.

        :return: The state machine either invokes a transition to the "finalized" state or stays
        in the current state depending on the return value.
          TransitionCallbackReturn.SUCCESS transitions to "unconfigured".
          TransitionCallbackReturn.FAILURE transitions to "inactive".
          TransitionCallbackReturn.ERROR or any uncaught exceptions to "errorprocessing"
        """
        self.get_logger().info("on_shutdown() is called.")

        return TransitionCallbackReturn.SUCCESS


# A lifecycle node has the same node API
# as a regular node. This means we can spawn a
# node, give it a name and add it to the executor.


def main():
    rclpy.init()

    executor = rclpy.executors.SingleThreadedExecutor()
    lc_node = LifecycleListener("lc_listener")
    executor.add_node(lc_node)

    try:
        executor.spin()
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        lc_node.destroy_node()
        executor.shutdown()


if __name__ == "__main__":
    main()
