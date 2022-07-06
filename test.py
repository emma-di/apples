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

# python runs thigns when imported
# import motion class/methods
from applevision_motion import MotionPlanner, AppleApproach, main

# it noramlly has to be in proxy to work


# !!!!! initial = uh
bot = MotionPlanner()
bot.go_to_joint_state
# posible headjoint joint states (-.56, -.18, -1.99,-4.06,-2.12,-1.37)
# !!!!! applepos = uhh
# apple: (-.51; -.16; 1.3)
# concerns: apple center? apple edge?

# if you want to get fancy, make a list of trials + results and log to a csv

x=1
num = int(input("Run how many times? "))

# loops through for desired amout of time
for x in range(num):
    # reset
    # go to home position
    result = "fail"

    # motion runthrough
    main()

    # # check final position
    # final = self.get_current_state()

    # # determine success
    #     # find distance between apple (find coords) and hand
    # if dist(applepos, final) < BLANK (units) and APPLEISINSIGHT:
    #     result = "success"

    # log results (see note above)
    print("Number" + str(x) + "was a" + result)

    x+=1