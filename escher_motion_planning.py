# Python interface for the C++ Escher motion planning module
import openravepy as rave

import time
import sys
import numpy as np

def load_robot(env, urdf_path=None, srdf_path=None):
    if(not urdf_path):
        urdf_path = urdf

    if(not srdf_path):
        srdf_path = srdf


    rave.RaveLoadPlugin('../or_urdf/build/devel/lib/openrave-0.9/or_urdf_plugin')
    module = rave.RaveCreateModule(env, 'urdf')
    robot_name = module.SendCommand('load {} {}'.format(urdf_path, srdf_path))
    robot = env.GetRobot(robot_name)

    robot.GetManipulator('l_arm').SetLocalToolDirection(np.array([1, 0, 0]))
    robot.GetManipulator('l_arm').SetLocalToolTransform(np.array([
        [0,  1, 0, 0.18],
        [ -1, 0, 0, -0.025],
        [ 0,  0, 1, 0],
        [ 0,  0, 0, 1]])
    )

    robot.GetManipulator('r_arm').SetLocalToolDirection(np.array([1, 0, 0]))
    robot.GetManipulator('r_arm').SetLocalToolTransform(np.array([
        [ 0,  -1, 0, 0.18],
        [ 1,  0, 0, 0.025],
        [ 0,  0, 1, 0],
        [ 0,  0, 0, 1]])
    )

    robot.GetManipulator('l_leg').SetLocalToolDirection(np.array([0, 0, -1]))
    robot.GetManipulator('r_leg').SetLocalToolDirection(np.array([0, 0, -1]))

    return robot

# OpenRave C++ plugin is called by sending string command. We can add parameters in this function to construct the command, and decode in C++ side.
# For example, I can add an option whether to turn on the parallelization or not
def SendStartPlanningCommand(Module,robotname=None,goal=None,parallelization=None):
    cmd = ['StartPlanning']

    cmd.append('robotname')
    cmd.append(robotname)

    cmd.append('goal')

    for g in goal:
        cmd.append(g)

    if(parallelization is not None):
        cmd.append('parallelization')

        if(parallelization):
            cmd.append(1)
        else:
            cmd.append(0)

    cmd_str = " ".join(str(item) for item in cmd)

    result_str = Module.SendCommand(cmd_str)

    print("Output message received in Python:")
    print(result_str)


    return

def main():
    env = rave.Environment()
    env.SetViewer('qtcoin')
    env.Reset()

    ## load the Escher robot
    urdf = 'file://escher_model/escher_cpp.urdf'
    srdf = 'file://escher_model/escher_cpp.srdf'

    robot = load_robot(env, urdf_path=urdf, srdf_path=srdf)

    ### INITIALIZE PLUGIN ###
    rave.RaveInitialize()
    rave.RaveLoadPlugin('build/escher_motion_planning')
    EscherMotionPlanning = rave.RaveCreateModule(env,'EscherMotionPlanning')
    ### END INITIALIZING PLUGIN ###

    SendStartPlanningCommand(EscherMotionPlanning,robotname=robot.GetName(),goal=[1.0,0.0,0.0],parallelization=True)

    raw_input("Press enter to exit...")
    # import IPython; IPython.embed();

    return



if __name__ == "__main__":
    main()
    
