import support.vrep as vrep
import time
import numpy as np
import math
from scipy.linalg import expm, logm
import matplotlib.pyplot as plt
from scripts.robot_motion import *
from scripts.robot_localization import *
from scripts.arm_motion import *
from scripts.robot_lidar import *
from scripts.vision import *
from scripts.dijkstra_hardcode import setup_dijkstras, get_local_heading
from pf_test import *
import sys
import threading as th

show_pf = False
show_dijkstra = False
manual_mode = False
for arg in sys.argv:
    if arg == "-show_pf":
        show_pf = True
    if arg == "-show_path_grid":
        show_dijkstra = True
    if arg == "-manual":
        manual_mode = True

# Close all open connections (Clear bad cache)
vrep.simxFinish(-1)
# Connect to V-REP (raise exception on failure)
clientID = vrep.simxStart('127.0.0.1', 19997, True, True, 5000, 5)
if clientID == -1:
	raise Exception('Failed connecting to remote API server')

# ======================================== Setup "handle"  ============================================= #

# Get a handle by name
def get_handle_blocking(name):
    result, obj = vrep.simxGetObjectHandle(clientID, name, vrep.simx_opmode_blocking)
    if result != vrep.simx_return_ok:
        raise Exception('could not get object: {}'.format(name))
    return obj

wheelJoints = [-1, -1, -1, -1]
wheelJoints[0]  = get_handle_blocking('rollingJoint_fl')
wheelJoints[1]  = get_handle_blocking('rollingJoint_rl')
wheelJoints[2]  = get_handle_blocking('rollingJoint_rr')
wheelJoints[3]  = get_handle_blocking('rollingJoint_fr')
youBot          = get_handle_blocking('youBot')
youBotRef       = get_handle_blocking('youBot_ref')
tip             = get_handle_blocking('youBot_positionTip')
prox_sensor     = get_handle_blocking('Proximity_sensor')
lidar_motor     = get_handle_blocking('Tower_Turning_Joint')
gripper 		= get_handle_blocking('youBotGripperJoint1')
gripper2		= get_handle_blocking('youBotGripperJoint2')
vision_sens     = get_handle_blocking('Vision_sensor')

armJoints = [-1] * 5
for i in range(5):
    armJoints[i] = get_handle_blocking('youBotArmJoint{}'.format(i))
# ======================================================================================================= #
# ======================================= Start Simulation ============================================== #
# ======================================================================================================= #

vrep.simxSynchronous(clientID, 1)
vrep.simxStartSimulation(clientID, vrep.simx_opmode_blocking)

# initialize motion classes
robot_motion = robot_motion(clientID, youBotRef, wheelJoints, armJoints[0])
arm_motion = arm_motion(clientID, youBotRef, armJoints, youBot, gripper, gripper2)

# Simulation dt is 50ms (0.05s)
dt = 0.05
lidar_v = 6

pf = particle_filter(400, KNOWN_MAP,perturb_pos_stdev=0.05, perturb_angle_stdev = 0.15,random_fraction=2)

# initialize sensor classes
vision_sensor = vision_sensor(clientID, vision_sens)
robot_lidar = robot_lidar(clientID, prox_sensor, lidar_motor)
robot_lidar.set_lidar_velocity(lidar_v)

# raise ValueError("poop")

vfb = 0
vlr = 0
vt = 0
i = 0
n_pf_updates = 0
readings = []
def update_pf():
    global i
    global n_pf_updates
    global readings
    lidar_result = robot_lidar.get_lidar_raw();
    if lidar_result:
        readings.append(lidar_result)
    else:
        readings.append(5)

    pf.update(robot_motion.velocities[0], robot_motion.velocities[1], robot_motion.velocities[2], robot_lidar.get_lidar_angle() + np.pi / 2, dt)
    i += 1
    if i == 30:
        i = 0
        n_pf_updates += 1
        pf.resample(readings)
        if show_pf:
            visualize_pf(pf)
            plt.ion()
            plt.show()
            plt.pause(0.001)
        readings = []
        print("Prediction:",pf.get_predicted_pose()[:2, -1])
        pos = robot_motion.get_global_position()
        print("Actual:[{} {}]".format(pos[0] + 2, pos[1] + 2))

if not manual_mode:
    print("Localizing and preparing dijkstras tables...")
    keep_going = True
    def dijkstras_run_thread():
        global keep_going
        setup_dijkstras(show_dijkstra)
        keep_going = False

    th.Thread(target=dijkstras_run_thread, args=(), name='dijkstras_run_thread', daemon=True).start()
    robot_motion.set_move(0, 0, 0)
    last_n_pf_updates = n_pf_updates
    while keep_going or n_pf_updates < 4:#12:
        #break
        vrep.simxSynchronousTrigger(clientID)
        vrep.simxGetPingTime(clientID)
        update_pf()
        # if n_pf_updates == last_n_pf_updates+4:
            # robot_motion.set_move(-robot_motion.velocities[0], 0, 0)
            # last_n_pf_updates = n_pf_updates
        robot_motion.motion_update()
        arm_motion.motion_update()

    if keep_going:
        print("Warning, Dijkstra's unfinished!")

    pf.resample_particles = 40

    target_ind = 0
    last_target_ind = -1
    target_points = [(1, 2), (0.5, 0.5), (1, 1), (3, 2)]
    print("Pathing mode: Visiting points {}".format(target_points))
    
    target_point = target_points[target_ind]
    print("Pathing mode: Going to {}".format(target_point))
    robot_motion.set_move_global_position2(target_point, get_local_heading, lambda: pf.get_predicted_pose(), 0.25)
    target_ind += 1
    keep_going = True
    
    still_positioning = True
    grab_can_state = 0
    while keep_going:
        #break
        vrep.simxSynchronousTrigger(clientID)
        vrep.simxGetPingTime(clientID)
        update_pf()
        avg_distance, avg_angle, any_red = vision_sensor.red_pixel_detection()
        if not still_positioning: # and grab_can_state == 1):
            print(arm_motion.state_machine)
            # TODO
            # grab trash, pick up, drop in bin
            # when done not done...? set's an angle loop
            # build set target arm angles, assign new update
            # that moves the gripper
            # then new update function to move the block
            still_positioning = arm_motion.motion_update()
            robot_motion.motion_update()
            # set this to true when done grabbing can
        elif any_red and arm_motion.state_machine == 0: #and grab_can_state == 0):
            still_positioning = True
            grab_can_state = 1
            # FIND A PIECE OF TRASH
            
            arm_motion.set_gripper(1)
            pickup_angles = np.array([0.0, -75, -95.4, 100.0, 0.0]) # degrees
            pickup_angles *= (np.pi/180) # radians
            arm_motion.set_target_arm_angles(pickup_angles) # correct set move function
            
            robot_motion.set_move_get_can(vision_sensor.red_pixel_detection, 0.1875)
            still_positioning = robot_motion.motion_update()
            still_positioning = arm_motion.motion_update() or still_positioning
                
            arm_motion.state_machine = 0
            # TODO, still_positioning not returning False --> done
            last_target_ind = target_ind
            # Hacky way to restore the old target point
            if not still_positioning:
                arm_motion.set_move_get_can()
        elif last_target_ind != -1:
            target_ind = last_target_ind - 1
            print(f"Resume path, index {target_ind}")
            last_target_ind = -1
            target_point = target_points[target_ind]
            print("Pathing mode: Going to {}".format(target_point))
            robot_motion.set_move_global_position2(target_point, get_local_heading, lambda: pf.get_predicted_pose(), 0.25)
            target_ind += 1
        else: #grab_can_state == 0:
            # CONTINUE THE SEARCH PATH
            keep_going = robot_motion.motion_update()
            arm_motion.motion_update()
            if (not keep_going) and target_ind < len(target_points):
                target_point = target_points[target_ind]
                print("Pathing mode: Going to {}".format(target_point))
                robot_motion.set_move_global_position2(target_point, get_local_heading, lambda: pf.get_predicted_pose(), 0.25)
                target_ind += 1
                keep_going = True
        # else:
        #     tmp1 = arm_motion.motion_update()
        #     tmp2 = robot_motion.motion_update()
        #     still_positioning = tmp2 or tmp1


print("Manual mode")

# Hack to do manual robot control
keep_going = True
if (pf.resample_particles == 400):
    pf = particle_filter(40, KNOWN_MAP,perturb_pos_stdev=0.05, perturb_angle_stdev = 0.15,random_fraction=2)

arm_angles = np.array([0.0, -87.4, -95.4, 62.0, 0.0]) # degrees
arm_angles *= (np.pi/180) # radians
# arm_angles = [0, -1.5708, -1.6, 1.2, 0]
grasp_pos = arm_motion.get_gripper()

def key_capture_thread():
    global keep_going
    global vfb
    global vlr
    global vt
    global arm_motion
    global avg_angle
    global avg_distance
    global vision_sensor
    while True:
        in_str = input().strip()
        for c in in_str:
            if c == "q":
                keep_going = False
                break
            elif c == "b":
                vfb = 0
                vlr = 0
                vt = 0
            elif c == "m":
                # arm_motion.set_target_arm_angles([0]*5)
                arm_motion.grab_red(avg_angle, avg_distance, vision_sensor)
            elif c == "w":
                vfb += 0.1
            elif c == "s":
                vfb -= 0.1
            elif c == "d":
                vlr += 0.1
            elif c == "a":
                vlr -= 0.1
            elif c == "x":
                vt -= 0.2
            elif c == "z":
                vt += 0.2
            elif c == "y":
                arm_angles[0] += 0.1
            elif c == "u":
                arm_angles[1] += 0.1
                arm_motion.set_target_arm_angles(arm_angles)
            elif c == "i":
                arm_angles[2] += 0.1
                arm_motion.set_target_arm_angles(arm_angles)
            elif c == "o":
                arm_angles[3] += 0.1
                arm_motion.set_target_arm_angles(arm_angles)
            elif c == "p":
                arm_angles[4] += 0.1
                arm_motion.set_target_arm_angles(arm_angles)
            elif c == "h":
                arm_angles[0] -= 0.1
                arm_motion.set_target_arm_angles(arm_angles)
            elif c == "j":
                arm_angles[1] -= 0.1
                arm_motion.set_target_arm_angles(arm_angles)
            elif c == "k":
                arm_angles[2] -= 0.1
                arm_motion.set_target_arm_angles(arm_angles)
            elif c == "l":
                arm_angles[3] -= 0.1
                arm_motion.set_target_arm_angles(arm_angles)
            elif c == ";":
                arm_angles[4] -= 0.1
                arm_motion.set_target_arm_angles(arm_angles)
            elif c == "[":
                print(arm_angles)
                print(arm_motion.get_gripper())
            elif c == "]":
                robot_motion.set_move_get_can(vision_sensor.red_pixel_detection)
            elif c == "g":
                arm_motion.set_gripper(-0.1)
            elif c == "t":
                arm_motion.set_gripper(0.1)
                # arm_motion.inv_kin(None)

th.Thread(target=key_capture_thread, args=(), name='key_capture_thread', daemon=True).start()
while keep_going:
    # arm_motion.set_move_get_can(vision_sensor)
    # Trigger a "tick"

    vrep.simxSynchronousTrigger(clientID)
    vrep.simxGetPingTime(clientID)
    update_pf()

    robot_motion.set_move(vfb, vlr, vt)
    robot_motion.motion_update()
    # arm_motion.set_gripper(1)
    arm_motion.motion_update()
    # if not arm_motion.motion_update():
    arm_motion.set_target_arm_angles(arm_angles)

# robot_motion.set_move(0,0,0)
# pos = robot_motion.get_global_position()
# print(pos)


# ======================================================================================================= #
# ======================================== End Simulation =============================================== #
# ======================================================================================================= #

vrep.simxStopSimulation(clientID, vrep.simx_opmode_oneshot)
# Before closing the connection to V-REP, make sure that the last command sent out had time to arrive. You can guarantee this with (for example):
vrep.simxGetPingTime(clientID)
# Close the connection to V-REP
vrep.simxFinish(clientID)
print("==================== ** Simulation Ended ** ====================")
