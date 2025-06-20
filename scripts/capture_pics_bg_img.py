#!/usr/bin/env python
import argparse
import rospy
import baxter_interface
from baxter_interface import CHECK_VERSION
from baxter_interface.camera import CameraController
import numpy as np
import struct
import rosgraph
import socket
import tf
import cv2
import cv_bridge
import os
import time
import baxter_interface.digital_io as DIO
from sensor_msgs.msg import Image
from baxter_core_msgs.srv import ListCameras
from std_msgs.msg import Header
from baxter_core_msgs.srv import (
    SolvePositionIK,
    SolvePositionIKRequest,
)
from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)

right_base1 = [0.346689104635,  -0.554945461976,  -0.001543541374]
right_base2 = [-0.0179540327606, 0.689243036276, 0.00840373721407, 0.724258977752]
cameras = ['head_camera','right_hand_camera','left_hand_camera']
def send_image(host, port,img):
	s = socket.socket()
	s.connect((host,port))
	
	_, img_encoded = cv2.imencode('.jpg',img)
	img_bytes = img_encoded.tostring()
	s.sendall(struct.pack('>I',len(img_bytes)))
	s.sendall(img_bytes)
	time.sleep(0.1)
	s.close()

def get_pose(pos_list, quat_list):
    hdr = Header(stamp=rospy.Time.now(), frame_id='base')
    return PoseStamped(
        header = hdr,
        pose = Pose(
            position=Point(pos_list[0], pos_list[1], pos_list[2]),
            orientation = Quaternion(quat_list[0], quat_list[1], quat_list[2], quat_list[3]),
        )
    )

def ik_get(limb, pose):
    ns = "ExternalTools/" + limb + "/PositionKinematicsNode/IKService"
    iksvc = rospy.ServiceProxy(ns, SolvePositionIK)
    ikreq = SolvePositionIKRequest()
    ikreq.pose_stamp.append(pose)

    try:
        rospy.wait_for_service(ns, 5.0)
        resp = iksvc(ikreq)
    except (rospy.ServiceException, rospy.ROSException), e:
        rospy.logerr("Service call failed: %s" % (e,))
        return 1
    
    resp_seeds = struct.unpack('<%dB' % len(resp.result_type), resp.result_type)
    if (resp_seeds[0] != resp.RESULT_INVALID):
        limb_joints = dict(zip(resp.joints[0].name, resp.joints[0].position))
        print("SUCCESS - Valid Joint Solution Found")
        return limb_joints
    else:
        print("INVALID POSE - No Valid Joint Solution Found.")
        return 0


def open_cam(camera, res):
    """Open the given camera at the set resolution if valid."""

    if not any((res[0] == r[0] and res[1] == r[1]) for r in CameraController.MODES):
        rospy.logerr("Invalid resolution provided.")
    cam = CameraController(camera) 
    cam.resolution = res 
    cam.open()

def close_cam(camera):
    """Close the given camera."""

    cam = CameraController(camera)
    cam.close()

def list_cameras():
    """Get dictionary with camera names and whether they are active or not."""

    ls = rospy.ServiceProxy('cameras/list', ListCameras)
    rospy.wait_for_service('cameras/list', timeout = 10)
    resp = ls()
    if len(resp.cameras):
        master = rosgraph.Master('/rostopic')
        resp.cameras
        cam_topics = dict([(cam, "/cameras/%s/image" % cam) for cam in resp.cameras])
        open_cams = dict([(cam, False) for cam in resp.cameras])
        try:
            topics = master.getPublishedTopics('')
            for topic in topics:
                for cam in resp.cameras:
                    if topic[0] == cam_topics[cam]:
                            open_cams[cam] = True
        except socket.error:
            raise ROSTopicIOException("Cannot communicate with master.")
    return open_cams

class CameraLink():
    def __init__(self, camera_open, res, camera_close):
        """Establishes a new camera link to camera_open, closing camera_close if both other cameras are open."""
        self._cv_image = None
        print("Getting camera list.")
        cameras = list_cameras()
        camera_count = 0
        for key, value in cameras.items():
            if value:
                camera_count += 1
        
        if(not cameras.get(camera_open, False) and  camera_count > 1):
            print("Closing %s." % (camera_close))
            close_cam(camera_close)

        print("Opening %s." % (camera_open))
        open_cam(camera_open, res)

        print("Opening bridge.")
        self._bridge = cv_bridge.CvBridge()

        print("Opening subscriber to image.")
        rospy.Subscriber('/cameras/'+ camera_open + '/image', Image, self._image_callback)
        
    def _image_callback(self, ros_img):
        self._cv_image = self._bridge.imgmsg_to_cv2(ros_img, desired_encoding = "passthrough")  

    def get_image(self):
        return self._cv_image


def display_link(camera_link):
        """Display the image from camera_link on the running machine."""
        cv_image = camera_link.get_image()
        if(cv_image is not None):
            cv2.imshow('Image', cv_image)

def head_stream():
    """Continously stream the head camera to the host screen until the node is shutdown."""
    save_dir = "/home/rosbox/ros_ws/src/baxter-coffee_maker/scripts/images"
    head_cam_link = CameraLink(cameras[2], (640, 400), None)
    rate = rospy.Rate(60)
    limb_left = baxter_interface.Limb('left')
    limb_right = baxter_interface.Limb('right')
    print("Hit ^C in terminal or q in window to exit stream.")
    while not rospy.is_shutdown():
        display_link(head_cam_link)
	key = cv2.waitKey(1)
	#if key == ord('l'):
	#	image_count = len(os.listdir(save_dir))
	#	image_path = os.path.join(save_dir,"l_" + str(image_count + 1) + ".png")
	#	cv2.imwrite(image_path,head_cam_link.get_image())
	#	print("successfully saved image")
	if key == ord('s'):
		cv2.imwrite('./baxterimg.jpg',head_cam_link.get_image())
		imgtosend = cv2.imread('./baxterimg.jpg')
		send_image('172.16.208.22',5001,imgtosend)
	if key == ord('l'):
		print("l is pressed")
		right_base1[1] = right_base1[1] + 0.01
		right_base2[3] += 0.01
		right_arm_view = ik_get('right', get_pose(right_base1,right_base2))
		limb_right.move_to_joint_positions(right_arm_view)
	if key == ord('w'):
		cv2.imwrite(save_dir+'/testimg.png',head_cam_link.get_image())
		compimg = cv2.imread(save_dir+'/testimg.png')
		gray = cv2.cvtColor(compimg,cv2.COLOR_BGR2GRAY)
		diff = cv2.absdiff(background_gray,gray)
        	image_count = len(os.listdir(save_dir))
        	image_path = os.path.join(save_dir,"yolo_" + str(image_count + 1) + ".png")
        	cv2.imwrite(image_path,results)
        	print("successfully saved image")
	if key == ord('c'):
		#position in front of coffee maker
		left_pos = ik_get('left',  get_pose( [0.844961051425,  -0.0145305739475,  0.217117615184],[ -0.0354530949479, 0.873945082545, -0.0576079952568, 0.481294493669]) )
		limb_left.move_to_joint_positions(left_pos)
        if cv2.waitKey(1) & 0xFF == ord('q'):
        	break
        rate.sleep()

def main():
    """Demonstrate access to camera output."""
    rospy.init_node('camera_demo', anonymous=True)
    head = baxter_interface.Head()
    limb_left = baxter_interface.Limb('left')
    limb_right = baxter_interface.Limb('right')
    #left_arm_view = ik_get('left',  get_pose( [  0.6827,  0.5185,  0.6923 ],[ -0.0792, 0.9388, -0.1927, 0.2744 ]) )
    right_arm_view = ik_get('right',  get_pose( [0.346689104635,  -0.554945461976,  -0.001543541374],[ -0.0179540327606, 0.689243036276, 0.00840373721407, 0.724258977752]) )
    if(#left_arm_view == 0 or left_arm_view == 1 or
       right_arm_view == 0 or right_arm_view == 1):
	return
    left_pos =  { 'left_e0'  :  -2*np.pi/5, 'left_e1'  :  3*np.pi/5, 'left_s0'  : 0, 'left_s1'  :  -3*np.pi/10, 'left_w0'  :   np.pi/5, 'left_w1'  :  3*np.pi/10, 'left_w2'  :  -np.pi/5}
    right_pos = { 'right_e0' :   2*np.pi/5, 'right_e1' :  3*np.pi/5, 'right_s0' : 0, 'right_s1' :  -3*np.pi/10, 'right_w0' :  -np.pi/5, 'right_w1' :  3*np.pi/10, 'right_w2' :   np.pi/5}
    limb_left.move_to_joint_positions(left_pos)
    limb_right.move_to_joint_positions(right_pos)
    rospy.sleep(1)
    #limb_left.move_to_joint_positions(left_arm_view)
    limb_right.move_to_joint_positions(right_arm_view)
    head.set_pan(0,speed = 0.5)
    print("Initializing Node...")
    
    print("Getting robot state...")
    rs = baxter_interface.RobotEnable(CHECK_VERSION)
    init_state = rs.state().enabled
    def clean_shutdown():
        print("\nExiting example...")
        cv2.destroyAllWindows()
        if not init_state:
            print("Disabling robot...")
            rs.disable()
    rospy.on_shutdown(clean_shutdown)
    head_stream()
    #for cam in cameras:
	#close_cam(cam)
  
if __name__ == '__main__':
    main()

