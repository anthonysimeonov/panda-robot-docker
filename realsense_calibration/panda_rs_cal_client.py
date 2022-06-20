import os, os.path as osp
import json
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import time
import cv2
import signal
import sys
from scipy import optimize  
import copy
import lcm
import argparse
import threading

import rospkg
import rospy
# from airobot import Robot
from franka_interface import ArmInterface

from airobot import log_info, log_warn, log_debug, log_critical, set_log_level
from utils import util
from utils.real_util import RealImageLCMSubscriber, RealCamInfoLCMSubscriber
from config.default_multi_realsense_cfg import get_default_multi_realsense_cfg
# from ndf_robot.utils.franka_ik import FrankaIK

rospack = rospkg.RosPack()
sys.path.append(osp.join(rospack.get_path('panda_package'), 'src'))
from robot_utils.panda_mg_wrapper import FrankaMoveIt
from robot_utils.config.default_real_demo_cfg import get_real_demo_cfg_defaults
from robot_utils import robot_util


def get_tool_position(pose, offset):
    ee_pose_world = util.list2pose_stamped(real_util.panda_full_pose(pose))
    offset_pose_ee = util.list2pose_stamped(offset.tolist() + [0, 0, 0, 1])
    offset_pose_world = util.convert_reference_frame(
        pose_source=offset_pose_ee,
        pose_frame_target=util.unit_pose(),
        pose_frame_source=ee_pose_world
    )
    return util.pose_stamped2np(offset_pose_world)[:3]


### !!! THIS ASSUMES THAT NOTHING IS ATTACHED TO THE ROBOT! MAKE SURE NOT TO SET THE TCP AS THE OFFSET POSE, OR ELSE IT WILL GET DOUBLED!!! ###
checkerboard_offset_from_tool = np.array([0.0, 0.0, 0.1])


cam012_home_joints = {
    'panda_joint1': -0.04935819447772545,
    'panda_joint2': -0.3469163031241724,
    'panda_joint3': 0.19999700257783215,
    'panda_joint4': -2.483996871786347,
    'panda_joint5': 0.08815259526835548,
    'panda_joint6': 2.305844914529558,
    'panda_joint7': -1.6618345036738449
}

cam0_workspace_limits = np.asarray([[400, 425], [-100, 100], [225, 250]]) # Cols: min max, Rows: x y z (define workspace limits in robot coordinates) # SET THIS
cam2_workspace_limits = np.asarray([[400, 425], [-100, 100], [225, 250]]) # Cols: min max, Rows: x y z (define workspace limits in robot coordinates) # SET THIS
# cam2_workspace_limits = np.asarray([[400, 425], [-100, 100], [225, 250]]) # Cols: min max, Rows: x y z (define workspace limits in robot coordinates) # SET THIS

cam1_workspace_limits = np.asarray([[400, 450], [-150, 150], [225, 275]]) # Cols: min max, Rows: x y z (define workspace limits in robot coordinates) # SET THIS
cam3_workspace_limits = np.asarray([[400, 450], [-150, 150], [225, 275]]) # Cols: min max, Rows: x y z (define workspace limits in robot coordinates) # SET THIS

full_stiffness = [550]*3 + [40]*3
half_stiffness = (np.asarray(full_stiffness) / 2).tolist()


# Estimate rigid transform with SVD (from Nghia Ho)
def get_rigid_transform(A, B):
    assert len(A) == len(B)
    N = A.shape[0]; # Total points
    centroid_A = np.mean(A, axis=0)
    centroid_B = np.mean(B, axis=0)
    AA = A - np.tile(centroid_A, (N, 1)) # Centre the points
    BB = B - np.tile(centroid_B, (N, 1))
    H = np.dot(np.transpose(AA), BB) # Dot is matrix multiplication for array
    U, S, Vt = np.linalg.svd(H)
    R = np.dot(Vt.T, U.T)
    if np.linalg.det(R) < 0: # Special reflection case
        Vt[2,:] *= -1
        R = np.dot(Vt.T, U.T)
    t = np.dot(-R, centroid_A.T) + centroid_B.T
    return R, t


def get_rigid_transform_error(z_scale):
	global measured_pts, observed_pts, observed_pix, world2camera, camera, cam_intrinsics

	# Apply z offset and compute new observed points using camera intrinsics
	observed_z = observed_pts[:,2:] * z_scale
	observed_x = np.multiply(observed_pix[:,[0]]-cam_intrinsics[0][2],observed_z/cam_intrinsics[0][0])
	observed_y = np.multiply(observed_pix[:,[1]]-cam_intrinsics[1][2],observed_z/cam_intrinsics[1][1])
	new_observed_pts = np.concatenate((observed_x, observed_y, observed_z), axis=1)
	# print("new_observed_pts", new_observed_pts)
	# print("measured_pts", measured_pts)
	
	# Estimate rigid transform between measured points and new observed points
	R, t = get_rigid_transform(np.asarray(measured_pts), np.asarray(new_observed_pts))
	t.shape = (3,1)
	world2camera = np.concatenate((np.concatenate((R, t), axis=1),np.array([[0, 0, 0, 1]])), axis=0)

	# Compute rigid transform error
	registered_pts = np.dot(R,np.transpose(measured_pts)) + np.tile(t,(1,measured_pts.shape[0]))
	error = np.transpose(registered_pts) - new_observed_pts
	error = np.sum(np.multiply(error,error))
	rmse = np.sqrt(error/measured_pts.shape[0])
	return rmse


def lcm_sub_thread(lc):
    while True:
        lc.handle_timeout(1)


def main(args):
    if args.debug:
        set_log_level('debug')
    else:
        set_log_level('info')

    global measured_pts, observed_pts, observed_pix, world2camera, camera, cam_intrinsics

    signal.signal(signal.SIGINT, util.signal_handler)
    rospy.init_node('calibration_realsense')

    tf_launch_file_dir = osp.join(os.environ['NDF_SOURCE_DIR'], '../../catkin_ws/src/panda_ndf/launch')
    assert osp.exists(tf_launch_file_dir), 'Launch file directory doesn"t exist!'
    # tf_launch_file_dir = osp.join(rospack.get_path('panda_ndf'), 'launch/rs_static_tf.launch')
    tf_launch_file_str = '<launch>\n'

    calib_package_path = rospack.get_path('hand_eye_calibration')
    assert osp.exists(calib_package_path), 'Calibration file destination doesn"t exist!'

    with_robot = args.robot

    # cam_list_all = ['cam_0', 'cam_1', 'cam_2']
    # cam_list_all = ['cam_0', 'cam_1', 'cam_2', 'cam_3']
    # cam_list = [cam_list_all[args.cam_index]]
    lc = lcm.LCM("udpm://239.255.76.67:7667?ttl=0")

    lc_th = threading.Thread(target=lcm_sub_thread, args=(lc,))
    lc_th.daemon = True
    lc_th.start()

    rs_cfg = get_default_multi_realsense_cfg()
    serials = rs_cfg.SERIAL_NUMBERS

    rgb_topic_name_suffix = rs_cfg.RGB_LCM_TOPIC_NAME_SUFFIX
    depth_topic_name_suffix = rs_cfg.DEPTH_LCM_TOPIC_NAME_SUFFIX
    info_topic_name_suffix = rs_cfg.INFO_LCM_TOPIC_NAME_SUFFIX
    pose_topic_name_suffix = rs_cfg.POSE_LCM_TOPIC_NAME_SUFFIX

    prefix = rs_cfg.CAMERA_NAME_PREFIX
    camera_names = [f'{prefix}{i}' for i in range(len(serials))]
    camera_names = [camera_names[args.cam_index]]

    cam_list_str = ', '.join(camera_names)
    log_info(f'Camera list: {cam_list_str}')

    # update the topic names based on each individual camera
    rgb_sub_names = [f'{cam_name}_{rgb_topic_name_suffix}' for cam_name in camera_names]
    depth_sub_names = [f'{cam_name}_{depth_topic_name_suffix}' for cam_name in camera_names]
    info_sub_names = [f'{cam_name}_{info_topic_name_suffix}' for cam_name in camera_names]
    pose_sub_names = [f'{cam_name}_{pose_topic_name_suffix}' for cam_name in camera_names]

    img_subscribers = []
    for i, name in enumerate(camera_names):
        img_sub = RealImageLCMSubscriber(lc, rgb_sub_names[i], depth_sub_names[i])
        info_sub = RealCamInfoLCMSubscriber(lc, pose_sub_names[i], info_sub_names[i])
        img_subscribers.append((name, img_sub, info_sub))
    
    done = False
    while not done and not rospy.is_shutdown():
        # for idx, cam in enumerate(cam_list):
        for idx, cam_and_subs in enumerate(img_subscribers):
            cam, img_sub, info_sub = cam_and_subs

            save_dir = osp.join(os.getcwd(), 'calibration', cam)
            if not osp.exists(save_dir):
                os.makedirs(save_dir)

            if args.test:
                i = 0
                try:
                    while True:
                        i += 1
                        image = img_sub.get_rgb_img()
                        depth_image = img_sub.get_rgb_img()
                        now = time.time()
                        cv2.imwrite(osp.join(save_dir, f'{i}_{now}_rgb.png'), image)
                        cv2.imwrite(osp.join(save_dir, f'{i}_{now}_depth.png'), depth_image)
                        log_info('Testing: got image')
                except KeyboardInterrupt:
                    return

            cam_intrinsics = info_sub.get_cam_intrinsics(block=True)
            # hang out here in a loop until we can get the intrinsics (means our publisher is up)
            if np.array_equal(np.eye(3), cam_intrinsics):
                while True:
                    time.sleep(1.0)
                    cam_intrinsics = info_sub.get_cam_intrinsics(block=True)
                    if not np.array_equal(np.eye(3), cam_intrinsics):
                        break
            log_info('Camera intrinsics: ')
            print(cam_intrinsics)

            # Set workspace
            if cam == 'cam_0':
                workspace_limits = cam0_workspace_limits
                home_joints = cam012_home_joints
            elif cam == 'cam_1':
                workspace_limits = cam1_workspace_limits
                home_joints = cam012_home_joints
            elif cam == 'cam_2':
                workspace_limits = cam2_workspace_limits
                home_joints = cam012_home_joints
            elif cam == 'cam_3':
                workspace_limits = cam3_workspace_limits
                home_joints = cam012_home_joints

            # Set grid resolution
            workspace_limits = workspace_limits / 1000.0 # change to m
            calib_grid_num = [2,5,2]


            # Construct 3D calibration grid across workspace
            gridspace_x = np.linspace(workspace_limits[0][0], workspace_limits[0][1], num=calib_grid_num[0])
            gridspace_y = np.linspace(workspace_limits[1][0], workspace_limits[1][1], num=calib_grid_num[1])
            gridspace_z = np.linspace(workspace_limits[2][0], workspace_limits[2][1], num=calib_grid_num[2])

            # calib_grid_x, calib_grid_y, calib_grid_z = np.meshgrid(gridspace_x, gridspace_y, gridspace_z)
            calib_grid_x, calib_grid_y, calib_grid_z = np.meshgrid(gridspace_x, gridspace_y[::-1], gridspace_z)
            num_calib_grid_pts = calib_grid_x.shape[0]*calib_grid_x.shape[1]*calib_grid_x.shape[2]
            calib_grid_x.shape = (num_calib_grid_pts,1)
            calib_grid_y.shape = (num_calib_grid_pts,1)
            calib_grid_z.shape = (num_calib_grid_pts,1)
            calib_grid_pts = np.concatenate((calib_grid_x, calib_grid_y, calib_grid_z), axis=1)

            measured_pts = []
            observed_pts = []
            observed_pix = []

            if with_robot: 
                log_info('Connecting to robot...')
                # get config
                cfg = get_real_demo_cfg_defaults()
                cfg.freeze()

                # get main panda arm interface and set params
                panda = ArmInterface()
                panda.set_EE_frame_to_link('panda_link8')
                time.sleep(1.0)

                # get IK and motion planning interfaces
                #ik_helper = FrankaIK(gui=False, base_pos=[0, 0, 0.01])
                panda_mp = FrankaMoveIt(panda, cfg=cfg, pb_gui=False)

                # panda.move_to_joint_positions(home_joints)
                panda.set_joint_position_speed(0.9)
                home_plan = panda_mp.plan_joint_target_mp(home_joints, current=True)
                if args.start_at_current_pose:
                    pass
                else:
                    panda.execute_position_path(home_plan)
                    current_pose = panda.endpoint_pose()
                    panda.set_cart_impedance_pose(current_pose, [0]*6)
                    time.sleep(2.0)

                    val = input('Please manually move the robot to a good initial configuration near the center of the workspace, and press "enter" to begin')

                panda.move_to_joint_positions(panda.joint_angles())		

                # Move robot to each calibration point in workspace
                log_info('Collecting data...')
                for calib_pt_idx in range(num_calib_grid_pts):
                    new_pose = copy.deepcopy(panda.endpoint_pose()) 
                    robot_position = calib_grid_pts[calib_pt_idx, :]
                    new_pose['position'] = np.asarray(robot_position)
                    pos_str = ', '.join([str(val) for val in new_pose['position'].tolist()])
                    # pos_str = ', '.join(new_pose['position'].tolist())
                    log_debug(f'Moving to new position: {pos_str}')

                    ## get joint values with IK and plan path to joint values
                    #start_jnt_pos = panda.joint_angles()
                    #target_jnt_pos = ik_helper.get_feasible_ik(new_pose)
                    #motion_plan = ik_helper.plan_joint_motion(start_jnt_pos, target_jnt_pos)

                    # use moveit interface to plan path to pose target
                    panda_mp.set_current_start_state()  # make sure motion planner starts from current config
                    new_pose_list = real_util.panda_full_pose(new_pose)
                    new_pose_mp = new_pose_list
                    # pose_str = ', '.join(new_pose_mp)
                    pose_str = ', '.join([str(val) for val in new_pose_mp])
                    log_debug(f'New pose to set as MP target: {pose_str}')
                    motion_plan = panda_mp.plan_pose_target_mp(new_pose_mp, current=False)

                    # execute position plan
                    panda.execute_position_path(motion_plan)

                    tool_position = get_tool_position(panda.endpoint_pose(), checkerboard_offset_from_tool)  
                    #################################################################################################################
                    
                    # Find checkerboard center
                    checkerboard_size = (3,3)
                    refine_criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
                    
                    # get rgb and depth image
                    rgb, depth = img_sub.get_rgb_and_depth(block=True)
                    depth = depth * 0.001
                    depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth, alpha=0.03), cv2.COLORMAP_JET)
                    log_info(f'Saving to {save_dir}')
                    cv2.imwrite(osp.join(save_dir, f'rgb_{calib_pt_idx}_{idx}_{time.time()}.png'), cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR))
                    cv2.imwrite(osp.join(save_dir, f'depth_{calib_pt_idx}_{idx}_{time.time()}.png'), depth_colormap)
                
                    grayscale = cv2.cvtColor(rgb, cv2.COLOR_BGR2GRAY)

                    # normalize the grayscale image to be brighter
                    max_grayscale = float(np.amax(grayscale))
                    norm_grayscale = np.asarray((grayscale/max_grayscale)*255, dtype=np.uint8)

                    camera_depth_img = depth
                    camera_color_img = grayscale # instead of rgb -  use grayscale
                    gray_data_2 = grayscale
                    
                    ###################################################
                    
                    # Find checkerboard center
                    checkerboard_found, corners = cv2.findChessboardCorners(gray_data_2, checkerboard_size, None, cv2.CALIB_CB_ADAPTIVE_THRESH)
                    log_info(f'Found Checkerboard: {checkerboard_found}')
                    if checkerboard_found:
                        corners_refined = cv2.cornerSubPix(gray_data_2, corners, (3,3), (-1,-1), refine_criteria)

                        # Get observed checkerboard center 3D point in camera space
                        checkerboard_pix = np.round(corners_refined[4,0,:]).astype(int)
                        checkerboard_z = camera_depth_img[checkerboard_pix[1]][checkerboard_pix[0]]
                        checkerboard_x = np.multiply(checkerboard_pix[0]-cam_intrinsics[0][2],checkerboard_z/cam_intrinsics[0][0])
                        checkerboard_y = np.multiply(checkerboard_pix[1]-cam_intrinsics[1][2],checkerboard_z/cam_intrinsics[1][1])
                        log_debug(f'Checkerboard [x, y, z]: {checkerboard_x:.3f}, {checkerboard_y:.3f}, {checkerboard_z:.3f}')
                        if np.abs(checkerboard_z) < 1e-4:
                            log_info('Depth value too low, skipping')
                            continue

                        # Save calibration point and observed checkerboard center
                        observed_pts.append([checkerboard_x,checkerboard_y,checkerboard_z])

                        measured_pts.append(tool_position)
                        observed_pix.append(checkerboard_pix)
                        
                        # Draw and display the corners
                        # vis = cv2.drawChessboardCorners(robot.camera.color_data, checkerboard_size, corners_refined, checkerboard_found)
                        vis = cv2.drawChessboardCorners(gray_data_2, (1,1), corners_refined[4,:,:], checkerboard_found)
                        cv2.imwrite(osp.join(save_dir, '%06dd.png' % len(measured_pts)), vis)
                        # cv2.imshow('Calibration',vis)
                        # cv2.waitKey(0)

                if with_robot: 
                    # Move robot back to home pose
                    # panda.set_joint_position_speed(0.3)
                    # panda.move_to_joint_positions(home_joints)
                    home_plan = panda_mp.plan_joint_target_mp(home_joints, current=True)
                    panda.execute_position_path(home_plan)

                    measured_pts = np.asarray(measured_pts)
                    observed_pts = np.asarray(observed_pts)
                    observed_pix = np.asarray(observed_pix)

                    try:
                        np.save(osp.join(save_dir, 'measured_pts'), measured_pts)
                        np.save(osp.join(save_dir, 'observed_pts'), observed_pts)
                        np.save(osp.join(save_dir, 'observed_pix'), observed_pix)
                    except IOError as e:
                        print(e)
                        from IPython import embed
                        embed()

                else:
                    measured_pts = np.load(osp.join(save_dir, 'measured_pts.npy'))
                    observed_pts = np.load(osp.join(save_dir, 'observed_pts.npy'))
                    observed_pix = np.load(osp.join(save_dir, 'observed_pix.npy') ) 

                world2camera = np.eye(4)

                # Optimize z scale w.r.t. rigid transform error
                log_info('Calibrating...')

                z_scale_init = 1
                optim_result = optimize.minimize(get_rigid_transform_error, np.asarray(z_scale_init), method='Nelder-Mead')
                camera_depth_offset = optim_result.x

                # Save camera optimized offset and camera pose
                log_info('Saving...')
                np.savetxt(osp.join(save_dir, 'camera_depth_scale.txt'), camera_depth_offset, delimiter=' ')
                get_rigid_transform_error(camera_depth_offset)
                camera_pose = np.linalg.inv(world2camera)
                np.savetxt(osp.join(save_dir, 'camera_pose.txt'), camera_pose, delimiter=' ')
                log_info('Done.')

                # DEBUG CODE -----------------------------------------------------------------------------------
                np.savetxt(osp.join(save_dir, 'measured_pts.txt'), np.asarray(measured_pts), delimiter=' ')
                np.savetxt(osp.join(save_dir, 'observed_pts.txt'), np.asarray(observed_pts), delimiter=' ')
                np.savetxt(osp.join(save_dir, 'observed_pix.txt'), np.asarray(observed_pix), delimiter=' ')
                measured_pts = np.loadtxt(osp.join(save_dir, 'measured_pts.txt'), delimiter=' ')
                observed_pts = np.loadtxt(osp.join(save_dir, 'observed_pts.txt'), delimiter=' ')
                observed_pix = np.loadtxt(osp.join(save_dir, 'observed_pix.txt'), delimiter=' ')

                fig = plt.figure()
                ax = fig.add_subplot(111, projection='3d')
                ax.scatter(measured_pts[:,0],measured_pts[:,1],measured_pts[:,2], c='blue')

                # print(camera_depth_offset)
                R, t = get_rigid_transform(np.asarray(measured_pts), np.asarray(observed_pts))
                t.shape = (3,1)
                camera_pose = np.concatenate((np.concatenate((R, t), axis=1),np.array([[0, 0, 0, 1]])), axis=0)
                camera2robot = np.linalg.inv(camera_pose)
                t_observed_pts = np.transpose(np.dot(camera2robot[0:3,0:3],np.transpose(observed_pts)) + np.tile(camera2robot[0:3,3:],(1,observed_pts.shape[0])))

                ax.scatter(t_observed_pts[:,0],t_observed_pts[:,1],t_observed_pts[:,2], c='red')

                new_observed_pts = observed_pts.copy()
                new_observed_pts[:,2] = new_observed_pts[:,2] * camera_depth_offset[0]
                R, t = get_rigid_transform(np.asarray(measured_pts), np.asarray(new_observed_pts))
                t.shape = (3,1)
                camera_pose = np.concatenate((np.concatenate((R, t), axis=1),np.array([[0, 0, 0, 1]])), axis=0)
                camera2robot = np.linalg.inv(camera_pose)
                t_new_observed_pts = np.transpose(np.dot(camera2robot[0:3,0:3],np.transpose(new_observed_pts)) + np.tile(camera2robot[0:3,3:],(1,new_observed_pts.shape[0])))

                ax.scatter(t_new_observed_pts[:,0],t_new_observed_pts[:,1],t_new_observed_pts[:,2], c='green')

                plt.show()

                from airobot.utils import common
                print(camera2robot[:-1, -1].tolist() + common.rot2quat(camera2robot[:-1, :-1]).tolist())

                trans = camera2robot[:-1, -1].tolist()
                quat = common.rot2quat(camera2robot[:-1, :-1]).tolist()

                ret = {
                    'b_c_transform': {
                        'position': trans,
                        'orientation': quat,
                        'T': camera2robot.tolist()
                    }
                }

                calib_file_dir = osp.join(calib_package_path, args.data_path)
                if not osp.exists(calib_file_dir):
                    os.makedirs(calib_file_dir)
                calib_file_path = osp.join(calib_file_dir, cam + '_calib_base_to_cam.json')
                # calib_file_path = osp.join(calib_file_dir, args.cam + '_calib_base_to_cam.json')
                print(json.dumps(ret, indent=2))
                with open(calib_file_path, 'w') as fp:
                    json.dump(ret, fp, indent=2)    

                tf_str_list = [str(val) for val in trans] + [str(val) for val in quat]
                tf_str = ', '.join(tf_str_list)
                new_str_line = '  <node pkg="tf" type="static_transform_publisher" name="%s_broadcaster" args="%s panda_link0 %s_color_optical_frame 100" />\n' % (cam, tf_str, cam)
                tf_launch_file_str += new_str_line
                open(osp.join(tf_launch_file_dir, 'rs_static_tf_%s.launch' % cam), 'w').write('<launch>\n%s</launch>' % new_str_line)
        tf_launch_file_str += '</launch>'
        open(osp.join(tf_launch_file_dir, 'rs_static_tf.launch'), 'w').write(tf_launch_file_str)
        done = True
        break


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--cam_index', type=int, default=0)
    parser.add_argument('--all_cams', action='store_true')
    parser.add_argument('--robot', action='store_true')
    parser.add_argument('--data_path', type=str, default='result/panda')  
    parser.add_argument('--test', action='store_true')
    parser.add_argument('--start_at_current_pose', action='store_true')
    parser.add_argument('--debug', action='store_true')
    args = parser.parse_args()

    main(args)
