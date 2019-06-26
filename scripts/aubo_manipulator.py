#!/usr/bin/env python
import copy
import sys
from std_msgs.msg import String
import geometry_msgs.msg
import moveit_commander
import message_filters
import moveit_msgs.msg
import rospy
import tf
from bronkhorst.srv import SetIO
import numpy as np
from bronkhorst.msg import LfeCoordinate

DEFAULT_HEIGHT = 0.111  # Change to better value

class MoveGroupPythonInteface(object):
    def __init__(self):
        super(MoveGroupPythonInteface, self).__init__()

        moveit_commander.roscpp_initialize(sys.argv)

        rospy.init_node('robot_manipulator', anonymous=True)
        service_name = '/aubo_driver/set_io'
        rospy.wait_for_service(service_name)

        self.set_io = rospy.ServiceProxy(service_name, SetIO)

        self.last_message = None

        # Globar variables
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.group = moveit_commander.MoveGroupCommander('manipulator_i5')
        self.display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                            moveit_msgs.msg.DisplayTrajectory, queue_size=20)
        self.planning_frame = self.group.get_planning_frame()
        self.group_names = self.robot.get_group_names()

        self.orientation = tf.transformations.quaternion_from_euler(3.106, 0.0, -1.570) #3.12 0.015 0.269

        # Robot configuration
        self.group.set_goal_position_tolerance(0.01)
        self.group.set_goal_orientation_tolerance(0.1)
        self.group.set_planning_time(1)
        self.group.set_max_acceleration_scaling_factor(0.01)
        self.group.set_max_velocity_scaling_factor(0.001)

        lfe_sub = message_filters.Subscriber('lfe_coordinate', LfeCoordinate)
        lfe_sub.registerCallback(self.handle_lfe_position)

    def set_arm_height(self, z_axe):
        waypoints = []

        wgoal = self.group.get_current_pose().pose

        wgoal.position.z = z_axe

        wgoal.orientation.x = self.orientation[0]
        wgoal.orientation.y = self.orientation[1]
        wgoal.orientation.z = self.orientation[2]
        wgoal.orientation.w = self.orientation[3]
        waypoints.append(copy.deepcopy(wgoal))

        self.plan_and_execute(waypoints)

    def move_to(self, x, y, z):
        waypoints = []
        wgoal = geometry_msgs.msg.Pose()

        wgoal.position.x = x
        wgoal.position.y = y
        wgoal.position.z = z

        wgoal.orientation.x = self.orientation[0]
        wgoal.orientation.y = self.orientation[1]
        wgoal.orientation.z = self.orientation[2]
        wgoal.orientation.w = self.orientation[3]
        waypoints.append(copy.deepcopy(wgoal))
        self.plan_and_execute(waypoints)

    def move_to_pillar(self, upside):
        if upside:
            self.move_to(0.4457,-0.2755,0.115)  # pillar for upside lfe
        if not upside:
            self.move_to(0.4645,-0.2791,0.115)  # pillar for downside lfe

    def move_to_home(self):
        self.move_to(0.2276,-0.2791,0.115)  #  home coordinates or joint states.

    def plan_and_execute(self, goal):
        self.group.set_start_state_to_current_state()
        plan, fraction = self.group.compute_cartesian_path(goal, 0.1, 0.0, True)
        self.group.execute(plan, wait=True)

    def set_vacuum_state(self, state):  # ON = 2, OFF = 0
        try:
            return self.set_io(1, 3, state)
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e

    def axe_is_same(self, new_pose, last_pose):
        if abs(new_pose - last_pose) < 0.05:
            return True

    def pos_is_same(self, msg):
        if self.last_message is None:
            print "Last message is not yet set"
            return False
        if self.axe_is_same(msg.x_axe, self.last_message.x_axe) and self.axe_is_same(msg.y_axe, self.last_message.y_axe):
            print("Position is the same as last message")
            return True
        return False


    def handle_lfe_position(self, msg):
        if not self.pos_is_same(msg):
            self.move_to(msg.x_axe, msg.y_axe, DEFAULT_HEIGHT)
            rospy.sleep(0.2)
            self.set_vacuum_state(2)
            rospy.sleep(0.2)
            self.set_arm_height(0.111)  #should be the z pos that does not destroy the LFE, or read vacuum meter.
            rospy.sleep(0.4)
            self.move_to_pillar(msg.upside)
            rospy.sleep(0.4)
            self.set_vacuum_state(0)
            rospy.sleep(0.2)
            self.move_to_home()
            self.last_message = msg


def main():
    try:
        MoveGroupPythonInteface()
        rospy.spin()
    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return


if __name__ == '__main__':
    main()