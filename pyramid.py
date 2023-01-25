#!/usr/bin/env python

# Software License Agreement (BSD License)

# Copyright (c) 2013, SRI International
# All rights reserved.

# Author: Acorn Pooley, Mike Lautman

import sys
import copy
import rospy
# moveit_commander namespace provides MoveGroupCommander class, PlanningSceneInterface class, and RobotCommander class
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
from tf.transformations import quaternion_from_euler
import itertools

# Global Variables:
# Table = height: 0.7112m width: 0.60960m length: 1.8288m
# Table coordinates: xpos=0.5, ypos=0.0, zpos=-0.3556
# Distance from table to center of robot is about  7 11/16 in
# Cubes are 0.0587375m in length
box_length = 0.0587375
pyramid_coordinates = [
    (0.5, -0.25, (box_length/2) + 0.115),
    (0.5, -0.31, (box_length/2) + 0.115),
    (0.5, -0.37, (box_length/2) + 0.115),
    (0.5, -0.25 - box_length/2, (box_length * 1.5) + 0.115),
    (0.5, -0.31 - box_length/2, (box_length * 1.5) + 0.115),
    (0.5, -0.31, (box_length * 2.5) + 0.115)
]

# Box 1 foot from center of table and 1 foot from edge. Each box 6 inches from each other
boxes = [
    (0.5, 0.3048, box_length/2, 'box 0'),
    (0.5, 0.4572, box_length/2, 'box 1'),
    (0.5, 0.6096, box_length/2, 'box 2'),
    (0.3476, 0.3048, box_length/2, 'box 3'),
    (0.3476, 0.4572, box_length/2, 'box 4'),
    (0.3476, 0.6096, box_length/2, 'box 5')
]


def all_close(goal, actual, tolerance):
    """
    Convenience method for testing if a list of values are within a tolerance of their counterparts in another list
    @param: goal       A list of floats, a Pose or a PoseStamped
    @param: actual     A list of floats, a Pose or a PoseStamped
    @param: tolerance  A float
    @returns: bool
    """
    all_equal = True
    if type(goal) is list:
        for index in range(len(goal)):
            if abs(actual[index] - goal[index]) > tolerance:
                return False

    elif type(goal) is geometry_msgs.msg.PoseStamped:
        return all_close(goal.pose, actual.pose, tolerance)

    elif type(goal) is geometry_msgs.msg.Pose:
        return all_close(pose_to_list(goal), pose_to_list(actual), tolerance)

    return True


class MoveGroupPythonIntefaceTutorial(object):
    """MoveGroupPythonIntefaceTutorial"""

    def __init__(self):

        super(MoveGroupPythonIntefaceTutorial, self).__init__()

        # Initilializing moveit_commander and a rospy node
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('move_group_python_interface_tutorial', anonymous=True)

        # Instantiate a `RobotCommander`_ object. Provides information such as the robot's
        # kinematic model and the robot's current joint states
        robot = moveit_commander.RobotCommander()

        # Instantiate a PlanningSceneInterface object.  This provides a remote interface
        # for getting, setting, and updating the robot's internal understanding of the surrounding world
        scene = moveit_commander.PlanningSceneInterface()

        # Instantiate a MoveGroupCommander  object. This object is an interface
        # to a planning group (group of joints). The group is the primary
        # arm joints in the Panda robot, so we set the group's name to "panda_arm".
        # This interface can be used to plan and execute motions
        group_name = "panda_arm"
        move_group = moveit_commander.MoveGroupCommander(group_name)

        # Create a DisplayTrajectory ROS publisher which is used to display trajectories in Rviz
        display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                       moveit_msgs.msg.DisplayTrajectory,
                                                       queue_size=20)

        # Get the name of the reference frame for this robot
        planning_frame = move_group.get_planning_frame()
        print("\nPlanning frame: %s" % planning_frame)

        # Get the name of the link that is considered to be an end-effector. Will be empty string if no end effector
        eef_link = move_group.get_end_effector_link()
        print("End effector link: %s\n" % eef_link)

        # We can get a list of all the groups in the robot
        group_names = robot.get_group_names()
        print("Available Planning Groups:", robot.get_group_names())

        # Sometimes for debugging it is useful to print the entire state of the robot
        print("\nPrinting robot state: \n")
        print(robot.get_current_state())

        # Misc variables
        self.box_name = ''
        self.robot = robot
        self.scene = scene
        self.move_group = move_group
        self.display_trajectory_publisher = display_trajectory_publisher
        self.planning_frame = planning_frame
        self.eef_link = eef_link
        self.group_names = group_names

    def go_to_pose_euler(self,
                         x_pos=0.4,
                         y_pos=0.1,
                         z_pos=0.4,
                         roll=0.0,
                         pitch=0.0,
                         yaw=0.0):

        # We can plan a motion for this group to a desired pose for the end-effector:
        quaternion = quaternion_from_euler(roll, pitch, yaw)

        pose_goal = geometry_msgs.msg.Pose()

        pose_goal.orientation.x = quaternion[0]
        pose_goal.orientation.y = quaternion[1]
        pose_goal.orientation.z = quaternion[2]
        pose_goal.orientation.w = quaternion[3]

        pose_goal.position.x = x_pos
        pose_goal.position.y = y_pos
        pose_goal.position.z = z_pos

        self.move_group.set_pose_target(pose_goal)

        # Now, we call the planner to compute the plan and execute it.
        plan = self.move_group.go(wait=True)
        # Calling `stop()` ensures that there is no residual movement
        self.move_group.stop()
        # It is always good to clear your targets after planning with poses.
        # Note: there is no equivalent function for clear_joint_value_targets()
        self.move_group.clear_pose_targets()

        current_pose = self.move_group.get_current_pose().pose
        return all_close(pose_goal, current_pose, 0.01)

    def execute_plan(self, plan):
        # Executing a Plan
        # Use execute if you would like the robot to follow the plan that has already been computed
        self.move_group.execute(plan, wait=True)
        # **Note:** The robot's current joint state must be within some tolerance of the
        # First waypoint in the `RobotTrajectory`_ or ``execute()`` will fail

    def wait_for_state_update(self, box_is_known=False, box_is_attached=False, timeout=4):
        # Copy class variables to local variables to make the web tutorials more clear.
        box_name = self.box_name
        scene = self.scene
        # Ensuring Collision Updates Are Receieved
        # If the Python node dies before publishing a collision object update message, the message
        # could get lost and the box will not appear. To ensure that the updates are
        # made, we wait until we see the changes reflected in the
        # ``get_attached_objects()`` and ``get_known_object_names()`` lists.
        # For the purpose of this tutorial, we call this function after adding,
        # removing, attaching or detaching an object in the planning scene. We then wait
        # until the updates have been made or ``timeout`` seconds have passed
        start = rospy.get_time()
        seconds = rospy.get_time()
        while (seconds - start < timeout) and not rospy.is_shutdown():
            # Test if the box is in attached objects
            attached_objects = scene.get_attached_objects([box_name])
            is_attached = len(attached_objects.keys()) > 0

            # Test if the box is in the scene.
            # Note that attaching the box will remove it from known_objects
            is_known = box_name in scene.get_known_object_names()

            # Test if we are in the expected state
            if (box_is_attached == is_attached) and (box_is_known == is_known):
                return True

            # Sleep so that we give other threads time on the processor
            rospy.sleep(0.1)
            seconds = rospy.get_time()

        # If we exited the while loop without returning then we timed out
        return False

    def add_box(self, timeout=4, name='table', xpos=0.0, ypos=0.0, zpos=0.0, size=(0.0, 0.0, 0.0)):
        # Copy class variables to local variables
        box_name = self.box_name
        scene = self.scene

        box_pose = geometry_msgs.msg.PoseStamped()
        box_pose.header.frame_id = "world"
        box_pose.pose.orientation.w = 1.0
        box_pose.pose.position.x = xpos
        box_pose.pose.position.y = ypos
        box_pose.pose.position.z = zpos
        box_name = name
        scene.add_box(box_name, box_pose, size=size)

        # Copy local variables back to class variables.
        self.box_name = box_name
        return self.wait_for_state_update(box_is_known=True, timeout=timeout)

    # For opening and closing gripper

    def joint_goal_gripper(self, joint0=0, joint1=0):
        # Set local variable group_name to move to be "hand" not the class variable
        group_name = "panda_hand"
        move_group = moveit_commander.MoveGroupCommander(group_name)

        # Move panda fingers
        joint_goal = move_group.get_current_joint_values()
        joint_goal[0] = joint0
        joint_goal[1] = joint1

        # Move to commands
        move_group.go(joint_goal, wait=True)

        # Calling stop() ensures that there is no residual movement
        move_group.stop()

        # For testing:
        current_joints = move_group.get_current_joint_values()
        return all_close(joint_goal, current_joints, 0.01)

    def attach_box(self, timeout=4, box_name='box'):
        # Copy class variables to local variables
        robot = self.robot
        scene = self.scene
        eef_link = self.eef_link
        group_names = self.group_names

        # attach the box to the Panda gripper. Manipulating objects requires the
        # robot be able to touch them without the planning scene reporting the contact as a
        # collision. By adding link names to the ``touch_links`` array, we are telling the
        # planning scene to ignore collisions between those links and the box. For the Panda
        # robot, we set ``grasping_group = 'hand'`
        touch_links = robot.get_link_names(group='panda_hand')
        touch_links.append('panda_hand_sc')
        scene.attach_box(eef_link, box_name, touch_links=touch_links)

        # We wait for the planning scene to update.
        return self.wait_for_state_update(box_is_attached=True, box_is_known=False, timeout=timeout)

    def detach_box(self, timeout=4, box_name='box'):
        # Copy class variables to local variables to make the web tutorials more clear.
        scene = self.scene
        eef_link = self.eef_link

        # We can also detach and remove the object from the planning scene:
        scene.remove_attached_object(eef_link, name=box_name)
        # END_SUB_TUTORIAL

        # We wait for the planning scene to update.
        return self.wait_for_state_update(box_is_known=True, box_is_attached=False, timeout=timeout)

    def plan_cartesian_path(self, del_x=0.0, del_y=0.0, del_z=0.0):
        # Copy class variables to local variables
        move_group = self.move_group

        waypoints = []

        wpose = move_group.get_current_pose().pose
        wpose.position.x += del_x
        wpose.position.y += del_y
        wpose.position.z += del_z
        waypoints.append(copy.deepcopy(wpose))

        # Interpolate cartesian path at resolution of 1 cm (eef_step). Disable jump threshold. Avoid
        # collisions with object you are picking up.
        (plan, fraction) = move_group.compute_cartesian_path(
            waypoints,   # waypoints to follow
            0.01,        # eef_step
            0.0,         # jump_threshold
            False)       # avoid_collisions

        # Note: We are just planning, not asking move_group to actually move the robot yet:
        return plan, fraction

    def remove_box(self, timeout=4):
        # Copy class variables to local variables to make the web tutorials more clear.
        box_name = self.box_name
        scene = self.scene

        scene.remove_world_object(box_name)

        # We wait for the planning scene to update.
        return self.wait_for_state_update(box_is_attached=False, box_is_known=False, timeout=timeout)

    def place_pyramid(self, current_box, pyramid_position):
        print("Picking up " + current_box[3])
        self.go_to_pose_euler(
            x_pos=current_box[0],
            y_pos=current_box[1],
            z_pos=current_box[2]+0.225,
            roll=pi,
            pitch=0.0,
            yaw=pi/4
        )

        # Open gripper
        self.joint_goal_gripper(joint0=0.04, joint1=0.04)

        # plan and execute cartesian move without collisions
        cartesian_plan, fraction = self.plan_cartesian_path(
            del_z=-0.1)
        self.execute_plan(cartesian_plan)

        self.attach_box(box_name=current_box[3])

        # Close gripper
        self.joint_goal_gripper(
            joint0=(box_length/2), joint1=(box_length/2))

        print("Moving to pyramid position " + str(pyramid_position))
        # Pyramid positions are globally defined, only index is required to pick correct position
        self.go_to_pose_euler(
            x_pos=pyramid_coordinates[pyramid_position][0],
            y_pos=pyramid_coordinates[pyramid_position][1],
            z_pos=pyramid_coordinates[pyramid_position][2]+0.1,
            roll=pi,
            pitch=0.0,
            yaw=pi/4
        )

        cartesian_plan, fraction = self.plan_cartesian_path(
            del_z=-0.1)
        self.execute_plan(cartesian_plan)
        self.detach_box(box_name=current_box[3])
        # Open gripper
        self.joint_goal_gripper(joint0=0.04, joint1=0.04)


def main():

    try:
        print("\nPress Ctrl-D to exit at any time\n")

        # Set up the moveit_commander
        environment = MoveGroupPythonIntefaceTutorial()

        print("\nAdding table to scene\n")
        environment.add_box(name="table", xpos=0.5, ypos=0.0,
                            zpos=-0.3556, size=(0.6096, 1.8288, 0.7112))

        # Adding all boxes to scene
        for b in boxes:
            print("Adding " + b[3]+" to scene")
            environment.add_box(name=b[3], xpos=b[0], ypos=b[1],
                                zpos=b[2], size=(box_length, box_length, box_length))

        position = 0
        pyramid_position = 0

        while len(boxes) > 0:
            b = boxes[position]

            environment.go_to_pose_euler(
                x_pos=b[0],
                y_pos=b[1],
                z_pos=b[2]+0.25,
                roll=pi,
                pitch=0.0,
                yaw=pi/4
            )
            while True:
                print(
                    "\nSelect Box: \n \t (c) Current \n \t (n) Next \n \t (p) Previous \n \t (a) Place all")
                input = raw_input("Input: ").strip().lower()

                # Place current box
                if input == "c":
                   # pick up box and place in pyramid
                    environment.place_pyramid(b, pyramid_position)
                    pyramid_position += 1
                    boxes.pop(position)  # Remove box from list
                    if len(boxes) == 0:
                        break
                    position %= len(boxes)  # Stay in indices of list
                    break
                # Go to next box
                elif input == 'n':
                    position += 1
                    position %= len(boxes)
                    break
                # Go to previous box
                elif input == "p":
                    position -= 1
                    position %= len(boxes)
                    break
                # Place all boxes
                elif input == "a":
                    while len(boxes) > 0:
                        environment.place_pyramid(boxes[0], pyramid_position)
                        pyramid_position += 1
                        boxes.pop(0)
                    break
                else:
                    print("Invalid Input")

            # once all boxes are placed, move arm up
        cartesian_plan, fraction = environment.plan_cartesian_path(
            del_z=0.1)
        environment.execute_plan(cartesian_plan)

        print("\nPyramid Complete")

    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return


if __name__ == '__main__':
    main()
