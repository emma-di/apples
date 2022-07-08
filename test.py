#!/usr/bin/env python3

# How successful is applevision_motion? 
# Goal: run applevision_motion several times & determine success or failure

# Emma's Questions:
# - the move function -> finding coords for the "home" position
# - determining success vs failure: find dist btwn apple & hand and check rob vision
# - calling/importing things from other scripts (namely applevision_motion.py)

# Before starting: have rviz & simulated camera open
    # roslaunch applevision_moveit_config demo.launch
    # roslaunch applevision_rospkg fake_sensor.launch

# To (hopefully) run this:
    # src/applevision_rospkg/bin/test.py

# go_to_pose_goal() could be useful
# https://github.com/ros-planning/moveit_tutorials/blob/master/doc/move_group_python_interface/scripts/move_group_python_interface_tutorial.py

# import motion class/methods
import applevision_motion
import rospy
import tf
from tf.listener import TransformListener

bot = applevision_motion.MoveGroupInterface()

# initial position
joints = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
positions = [0, -.58, -2.26, -.44, 1.62, .74]
# !!!!! applepos = uhh
# apple: (-.51; -.16; 1.3)
# concerns: apple center? apple edge?

# if you want to get fancy, make a list of trials + results and log to a csv

num = int(input("Run how many times? "))

# loops through for desired amout of time
for x in range(num):
    # reset
    # go to home position
    bot.moveToJointPosition(joints, positions)
    result = "fail"
    print("move joints done")

    # motion runthrough
    applevision_motion.main()
    print("main done")

    # check final position
    # all the code initiates some sort of node ?
    # rospy.init_node(applevision_motion)
    now = rospy.Time.now()
    sbr = tf.TransformBroadcaster()
    R = rospy.Rate(150)
    while not rospy.is_shutdown():
        br.sendTransform((1, 1, 1), (0, 0, 0, 1), rospy.Time(), '/base','/palm')      
        R.sleep()
    
    listener = tf.TransformListener()
    now = rospy.Time.now()
    listener.waitForTransform('/base','/palm',rospy.Time(), rospy.Duration(4.0))
    (trans, rot) = listener.lookupTransform('/base', '/palm', rospy.Time(0))
    print (trans)
    print('listener done')
    
    #applevision_motion.getpose()

    # determine success
        # find distance between apple (find coords) and hand
    #if dist(applepos, final) < BLANK (units) and APPLEISINSIGHT:
      #  result = "success"

    x+=1
    # log results (see note above)
    print("Number " + str(x) + " was a " + result)