#!/usr/bin/env python3
"""Stop the ROS controller when the real-time plugin reports trial completion."""

from __future__ import annotations

import threading

import rospy
from controller_manager_msgs.srv import SwitchController, SwitchControllerRequest
from std_msgs.msg import Bool


class TrialSupervisor:
    """Observe the finished topic and stop the active effort controller once."""

    def __init__(self) -> None:
        """Load the controller name, connect the switch service, and subscribe."""
        self.controller_name = rospy.get_param("~controller_name", "ecbf_controller")
        self.finished_event = threading.Event()
        rospy.wait_for_service("/controller_manager/switch_controller")
        self.switch_controller = rospy.ServiceProxy("/controller_manager/switch_controller", SwitchController)
        self.subscriber = rospy.Subscriber("finished", Bool, self.finished_callback, queue_size=1)

    def finished_callback(self, message: Bool) -> None:
        """Set the local completion event when the controller publishes true."""
        if message.data:
            self.finished_event.set()

    def stop_controller(self) -> None:
        """Request a strict controller stop before allowing roslaunch to terminate."""
        request = SwitchControllerRequest()
        request.stop_controllers = [self.controller_name]
        request.strictness = SwitchControllerRequest.STRICT
        request.start_asap = False
        request.timeout = rospy.Duration(2.0)
        try:
            response = self.switch_controller(request)
            if not response.ok:
                rospy.logwarn("Controller manager did not confirm the ECBF controller stop.")
        except rospy.ServiceException as error:
            rospy.logwarn("Controller stop service failed: %s", error)

    def spin(self) -> None:
        """Wait without polling ROS aggressively, stop the controller, and exit."""
        rate = rospy.Rate(20.0)
        while not rospy.is_shutdown() and not self.finished_event.is_set():
            rate.sleep()
        if self.finished_event.is_set():
            self.stop_controller()
            rospy.loginfo("ECBF trial complete; supervisor is terminating roslaunch.")


def main() -> None:
    """Initialize the ROS node and run the completion supervisor."""
    rospy.init_node("ecbf_trial_supervisor")
    TrialSupervisor().spin()


if __name__ == "__main__":
    main()
