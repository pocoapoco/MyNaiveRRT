#!/usr/bin/env python
# -*- coding: utf-8 -*-
#HW3 for RBE 595/CS 525 Motion Planning
import time
import openravepy

#### YOUR IMPORTS GO HERE ####

#### END OF YOUR IMPORTS ####

if not __openravepy_build_doc__:
    from openravepy import *
    from numpy import *

def waitrobot(robot):
    """busy wait for robot completion"""
    while not robot.GetController().IsDone():
        time.sleep(0.01)

def tuckarms(env,robot):
    with env:
        jointnames = ['l_shoulder_lift_joint','l_elbow_flex_joint','l_wrist_flex_joint','r_shoulder_lift_joint','r_elbow_flex_joint','r_wrist_flex_joint']
        robot.SetActiveDOFs([robot.GetJoint(name).GetDOFIndex() for name in jointnames])
        robot.SetActiveDOFValues([1.29023451,-2.32099996,-0.69800004,1.27843491,-2.32100002,-0.69799996]);        
        robot.GetController().SetDesired(robot.GetDOFValues());
    waitrobot(robot)

if __name__ == "__main__":

    env = Environment()
    env.SetViewer('qtcoin')
    env.Reset()        
    # load a scene from ProjectRoom environment XML file
    env.Load('hw3.env.xml')
    time.sleep(0.1)

    # 1) get the 1st robot that is inside the loaded scene
    # 2) assign it to the variable named 'robot'
    robot = env.GetRobots()[0]

    ### INITIALIZE YOUR PLUGIN HERE ###
    RaveInitialize()
    RaveLoadPlugin('build/MyRRT')
    myRRT = RaveCreateModule(env, 'MyRRT')
    
    ### END INITIALIZING YOUR PLUGIN ###
   

    # tuck in the PR2's arms for driving
    tuckarms(env,robot);
        
    # the active DOF are translation in X and Y and rotation about the Z axis of the base of the robot.
    jointnames =['l_shoulder_pan_joint','l_shoulder_lift_joint','l_elbow_flex_joint','l_upper_arm_roll_joint','l_forearm_roll_joint','l_wrist_flex_joint','l_wrist_roll_joint']
    robot.SetActiveDOFs([robot.GetJoint(name).GetDOFIndex() for name in jointnames])

    # set start config
    startconfig = [-0.15,0.075,-1.008,0,0,0,0]
    robot.SetActiveDOFValues(startconfig);        
    robot.GetController().SetDesired(robot.GetDOFValues());
    waitrobot(robot)

    with env:

        # startconfig = robot.GetActiveDOFValues().tolist()
        goalconfig = [0.449,-0.201,0.,0.,0.,0.,0.]

        ### YOUR CODE HERE ###
        ###call your plugin to plan, draw, and execute a path from the current configuration of the left arm to the goalconfig

        lmodel = databases.linkstatistics.LinkStatisticsModel(robot)
        if not lmodel.load():
            lmodel.autogenerate()

        lmodel.setRobotResolutions(0.01)
        lmodel.setRobotWeights()


        myRRT.SendCommand('setPlannerPar 0.16,15000,0.02')
        myRRT.SendCommand('setInitialConfig ' + str(startconfig).strip('[]').replace(' ', ''))
        myRRT.SendCommand('setGoalConfig ' + str(goalconfig).strip('[]').replace(' ', ''))
        myRRT.SendCommand('init')

        starttime = time.time()
        success = myRRT.SendCommand('run')
        plantime = time.time()-starttime

        print success
        print "Planning time: %fs"%plantime

        def isFloat(str):
            try:
                float(str)
                return True
            except ValueError:
                return False

        handles = []

        if success == 'success':

            # draw original path in red points
            strPath = myRRT.SendCommand('getFingerPath').split(';')[:-1]
            points = []

            for _ in strPath:

                point = []           
                for dimension in _.split(','):

                    if isFloat(dimension):
                        point.append(float(dimension))

                points.append((point[0], point[1], point[2]))

                handles.append(env.plot3(points =array((point[0], point[1], point[2])),
                                         pointsize = 0.008, colors = array(((1,0,0))), drawstyle = 1))

            # draw original path in red lines
            handles.append(env.drawlinestrip(points=array((points)),
                                 linewidth=3.0, colors=array(((1,0,0)))))

            # smooth original path
            smoothStartTime = time.time()
            myRRT.SendCommand('smoothPath')
            smoothTime = time.time()-smoothStartTime

            print "Smoothing time: %fs"%smoothTime

            # draw smoothed path in blue points
            strPath = myRRT.SendCommand('getFingerPath').split(';')[:-1]
            points = []

            for _ in strPath:

                point = []           
                for dimension in _.split(','):

                    if isFloat(dimension):
                        point.append(float(dimension))

                points.append((point[0], point[1], point[2]))

                handles.append(env.plot3(points =array((point[0], point[1], point[2])),
                                         pointsize = 0.008, colors = array(((0,0,1))), drawstyle = 1))

            
             # draw original path in blue lines
            handles.append(env.drawlinestrip(points=array((points)),
                                 linewidth=3.0, colors=array(((0,0,1)))))

            myRRT.SendCommand('executeTraj')


 
        ### END OF YOUR CODE ###
    waitrobot(robot)

    raw_input("Press enter to exit...")

