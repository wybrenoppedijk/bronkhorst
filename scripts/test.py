#!/usr/bin/env python
import copy
import sys
from std_msgs.msg import String
import geometry_msgs.msg
import moveit_commander
import moveit_msgs.msg
import rospy
import tf
from bronkhorst.srv import SetIO
import numpy as np


class MoveGroupPythonInteface(object):
    def __init__(self):
        super(MoveGroupPythonInteface, self).__init__()

        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('robot_manipulator', anonymous=True)
        service_name = '/aubo_driver/set_io'
        rospy.wait_for_service(service_name)

        self.set_io = rospy.ServiceProxy(service_name, SetIO)


        # List subscribers below...
        self.gripper_server_publisher = rospy.Publisher('robotToGripper', String, queue_size=10)

        # Globar variables
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.group = moveit_commander.MoveGroupCommander('manipulator_i5')
        self.display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                            moveit_msgs.msg.DisplayTrajectory, queue_size=20)
        self.planning_frame = self.group.get_planning_frame()
        self.group_names = self.robot.get_group_names()

        # Allow replanning to increase the odds of a solution

        # Robot configuration
        self.group.set_goal_position_tolerance(0.01)
        self.group.set_goal_orientation_tolerance(0.1)
        self.group.set_planning_time(0.1)
        self.group.set_max_acceleration_scaling_factor(0.1)
        self.group.set_max_velocity_scaling_factor(0.1)

    def set_arm_height(self, z_axe):
        waypoints = []

        wgoal = self.group.get_current_pose().pose
        q = tf.transformations.quaternion_from_euler(3.106, 0.0, -2.759) #3.12 0.015 0.269

        wgoal.position.z = z_axe

        wgoal.orientation.x = q[0]
        wgoal.orientation.y = q[1]
        wgoal.orientation.z = q[2]
        wgoal.orientation.w = q[3]
        waypoints.append(copy.deepcopy(wgoal))

        self.plan_and_execute(waypoints)

    def move_to(self, coordinates):
        waypoints = []
        wgoal = geometry_msgs.msg.Pose()
        q = tf.transformations.quaternion_from_euler(3.106, 0.0, -2.759) #3.12 0.015 0.269

        wgoal.position.x = coordinates[0]
        wgoal.position.y = coordinates[1]
        wgoal.position.z = coordinates[2]

        wgoal.orientation.x = q[0]
        wgoal.orientation.y = q[1]
        wgoal.orientation.z = q[2]
        wgoal.orientation.w = q[3]
        waypoints.append(copy.deepcopy(wgoal))


        self.plan_and_execute(waypoints)

    def plan_and_execute(self, goal):
        self.group.set_start_state_to_current_state()
        plan, fraction = self.group.compute_cartesian_path(goal, 0.1, 0.0, True)
        self.group.execute(plan, wait=True)

    def set_vacuum_state(self, state):
        try:
            return self.set_io(1, 3, state)
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e

    def get_state(self, counter):
        if counter % 2 == 0:
            return 2
        else:
            return 0

    def test_run(self):
        print "**********STARTING TEST RUN**********"
        counter = 0
        while True:
            self.move_to((0.175, 0.785, 0.136))
            rospy.sleep(0.2)
            self.set_arm_height(-0.0127)
            rospy.sleep(0.2)
            self.set_vacuum_state(self.get_state(counter))
            rospy.sleep(0.2)
            self.set_arm_height(0.136)
            rospy.sleep(0.2)
            self.move_to((-0.263, 0.656, 0.136))
            rospy.sleep(0.2)
            self.set_arm_height(-0.013)
            self.set_vacuum_state(self.get_state(counter + 1))
            rospy.sleep(0.2)
            self.set_arm_height(0.136)
            rospy.sleep(0.2)
            self.move_to((-0.087, 0.861, 0.136))
            rospy.sleep(0.2)
            self.set_arm_height(-0.01098)
            rospy.sleep(0.2)
            self.set_vacuum_state(self.get_state(counter))
            rospy.sleep(0.2)
            self.set_arm_height(0.136)
            rospy.sleep(0.2)
            counter += 1
        print self.group.get_current_pose()
        # self.move_to_coordinates()
        print "**********TEST RUN END**********"



def main():
    try:
        mgpi = MoveGroupPythonInteface()
        mgpi.test_run()
    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return


if __name__ == '__main__':
    main()