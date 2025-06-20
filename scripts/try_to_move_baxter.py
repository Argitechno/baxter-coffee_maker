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
import baxter_interface.analog_io as AIO
from baxter_interface import Gripper
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
    PointStamped,
    Quaternion,
)
base_dir = os.path.dirname(os.path.abspath(__file__))
pub = rospy.Publisher('/robot/xdisplay', Image, latch = True, queue_size = 1)

rospy.init_node('camera_demo', anonymous=True)
# Camera intrinsics (fx, fy, cx0, cy0) for the right_hand_camera.
# These should be set from your camera calibration:
fx = 406.9   # example focal length in pixels (to be tuned/calibrated)
fy = 406.9   # example focal length in pixels (assume square pixels for now)
cx0 = 656.5  # principal point x (image center x in pixels for 640x480 resolution)
cy0 = 421.9  # principal point y (image center y in pixels)
# Fixed depth assumption (in meters):
fixed_Z = 1.0  # e.g., assume cup is ~1 meter away from the camera (tune as needed)
# Set up TF listener to get transforms
tf_listener = tf.TransformListener()
limb_left = baxter_interface.Limb('left')
limb_right = baxter_interface.Limb('right')
def Kcup_center_callback(cx,cy,bx,by):
	"""
	This function takes in parameters to calculate the location of the K-Cup
	cx - The location of the center of the cup on x
	cy - The location of the center of the cup on y
	bx - The width of the cup on x
	by - The height of the cup on y
	"""
	try:
		pixel_cx = 320 
		pixel_cy = 200
		scale_y = 0.0508/by
		scale_x = 0.0508/bx
		offset_x = (cx - pixel_cx) * scale_x
		offset_y = (cy - pixel_cy) * scale_y
		focal_len = 396.8888888888888889
		Kcup_height = 0.0508
		move_forward_distance = (focal_len * Kcup_height)/by
		current_pose = limb_left.endpoint_pose()
		#print("moving " + offset_y + " left " + " and " + offset_x + " Up")
		Px = current_pose['position'].x - offset_y
		Py = current_pose['position'].y - offset_x
		Pz = current_pose['position'].z - move_forward_distance / 2
		orientation_q = current_pose['orientation']
		print(Px)
		print(Py)
		print(Pz)
		print(current_pose)
        	limb_left.move_to_joint_positions(ik_get('left',get_pose([Px,Py,Pz],orientation_q)))
	except Exception as e:
		angry = load_face('angry')
		pub.publish(angry)
        	rospy.sleep(1)
	        rospy.logerr("Error in Kcup_center_callback: %s", e)
def cup_center_callback(cx,cy,bx,by):
    """Callback for receiving the cup's center pixel coordinates."""
    try:
        # 1. Get pixel coordinates from message
        u = cx  # pixel coordinate x
        v = cy  # pixel coordinate y
	pixel_cx = 320.0
	pixel_cy = 208.0 +50
	scale_y = 0.1143/by #cups height
	scale_x = 0.0889/bx #cups width
	focal_len = 396.8888888888888889
	cup_height = 0.1143
        # 2. Back-project pixel (u,v) to 3D point in camera frame using fixed depth
        #Z = fixed_Z
        #X = (u - cx0) * Z / fx
        #Y = (v - cy0) * Z / fy
	offset_y = (cx - pixel_cx) * scale_x
	offset_z = (cy - pixel_cy) * scale_y
	#offset_x = offset_y * scale_z
	move_forward_distance = (focal_len * cup_height)/by
        current_pose = limb_right.endpoint_pose()  # current EE pose (dict with 'orientation'):contentReference[oaicite:4]{index=4}
	Py = current_pose['position'].y - offset_y + 0.03810
	Pz = current_pose['position'].z - offset_z 
	Px = current_pose['position'].x + move_forward_distance - 0.1
        # 3. Transform this point to base frame using TF

        # 4. Use IK to move right arm so that the wrist (end effector) goes to (Px,Py,Pz)
        # Use current end-effector orientation to maintain camera facing

        orientation_q = current_pose['orientation']  # quaternion (x,y,z,w)
        # Construct the PoseStamped for the target in base frame
	print(Px)
	print(Py)
	print(Pz)
	print(current_pose)
        limb_right.move_to_joint_positions(ik_get('right',get_pose([Px,Py,max(Pz,-0.06)],orientation_q)))

    except Exception as e:
	angry = load_face('angry')
	pub.publish(angry)
        rospy.sleep(1)
        rospy.logerr("Error in cup_center_callback: %s", e)


right_base1 = [0.346689104635,  -0.554945461976,  -0.001543541374]
right_base2 = [-0.0179540327606, 0.689243036276, 0.00840373721407, 0.724258977752]
cameras = ['head_camera','right_hand_camera','left_hand_camera']
def send_image(host, port,img):
	"""
	This function sends an image to a host through their ip address and port
	host - The ip address of the server you are sending it
	port - The port that is open on that computer
	img - The image in jpg format that is sent to the computer
	"""
	
	s = socket.socket()
	s.connect((host,port))
	
	_, img_encoded = cv2.imencode('.jpg',img)
	img_bytes = img_encoded.tostring()
	s.sendall(struct.pack('>I',len(img_bytes)))
	s.sendall(img_bytes)
	time.sleep(0.1)
	s.shutdown(socket.SHUT_WR)
	
	data = s.recv(32)
	s.close()

	msg = data.decode('utf-8')
	if msg == "none:none":
		return None,None,None,None
	else:
		cx,cy,bx,by = map(float, msg.strip().split(":"))
		print("success")
		return cx,cy,bx,by
		

def get_pose(pos_list, quat_list):
                                                                                                                  
    hdr = Header(stamp=rospy.Time.now(), frame_id='base')
    return PoseStamped(
        header = hdr,
        pose = Pose(
            position=Point(pos_list[0], pos_list[1], pos_list[2]),
            orientation = Quaternion(quat_list[0], quat_list[1], quat_list[2], quat_list[3]),
        )
    )
def move_forward(limb,dx,dy,dz):
	"""
	   This is a basic function to move the robots arm forwards
	   limb should be passed in as a string and move either the left or right arm
	   dx is the change you want to happen in the x direction
	   dy is the change you want to happen in the y direction
	   dz is the change you want to happen in the z direction
	"""
	if limb == "right":
		current_pose = limb_right.endpoint_pose()
		limb_right.move_to_joint_positions(ik_get(limb,get_pose([current_pose['position'].x+dx,current_pose['position'].y+dy,current_pose['position'].z+dz],current_pose['orientation'])))
		print("moving forward")
	else:
		current_pose = limb_left.endpoint_pose()
		limb_left.move_to_joint_positions(ik_get(limb,get_pose([current_pose['position'].x+dx,current_pose['position'].y+dy,current_pose['position'].z+dz],current_pose['orientation'])))
		print("moving forward")

def get_ir_distance():
	ir_sensor = AIO.AnalogIO('right_hand_range')
	raw = ir_sensor.state() - 48 # state in mm 
	return raw/1000.0


def ik_get(limb, pose):
    ns = "ExternalTools/" + limb + "/PositionKinematicsNode/IKService"
    iksvc = rospy.ServiceProxy(ns, SolvePositionIK)
    for offset in [0.0,0.1,-0.1]:
    	ikreq = SolvePositionIKRequest()
	pose.pose.position.x += offset
    	ikreq.pose_stamp.append(pose)
	ik_resp = iksvc(ikreq)
    	if(ik_resp.isValid[0]):
        	rospy.wait_for_service(ns, 5.0)
        	resp = iksvc(ikreq)
		break
   	else :
		print("invaid pose rejected")
        	rospy.logwarn("Ik failed given offset", offset)
        	pose.pose.position -= offset
		#return 1
    
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
def load_face(face_name):
	face = base_dir + '/../assets/baxter_faces/' + face_name + '.png'
	if not os.path.isfile(face):
		print("No file exists at: ")
		print(face)
		return
	img = cv2.imread(face)
	#img = resize(img)
    	#img = border(img)
	msg = cv_bridge.CvBridge().cv2_to_imgmsg(img, encoding="bgr8")
	return msg

def make_coffee():
    """This function is the core of the program, it sends images to a server and evaluates the location of both paper cups and KCups based on those locations. 	
	The keurig that it is loading must be in a static position roughly 30 inches from the center of baxter
	
	"""
    save_dir = "/home/rosbox/ros_ws/src/baxter-coffee_maker/scripts/images"
    head_cam_link = CameraLink(cameras[2], (640, 400), cameras[0])
    #close_cam(cameras[0])
    rate = rospy.Rate(60)
    grip = baxter_interface.Gripper('right')
    suck = baxter_interface.Gripper('left')
    print("Hit ^C in terminal or q in window to exit stream.")
    default_baxter = load_face('default')
    down_left = load_face('down_left')
    down_right = load_face('down_right')
    smug = load_face('smug')
    pub.publish(default_baxter)
    rospy.sleep(1)
    while not rospy.is_shutdown():
        display_link(head_cam_link)
	key = cv2.waitKey(1)
	#if key == ord('l'):
	#	image_count = len(os.listdir(save_dir))
	#	image_path = os.path.join(save_dir,"l_" + str(image_count + 1) + ".png")
	#	cv2.imwrite(image_path,head_cam_link.get_image())
	#	print("successfully saved image")
	if key == ord('s'):
		cv2.destroyAllWindows()
		head_cam_link = CameraLink(cameras[1], (640, 400), cameras[0])
		focal_len = 396.9
		cup_height = 0.4826
		pub.publish(down_left)
    		rospy.sleep(1)
		while(head_cam_link.get_image() == None):
			pass
		while(1):
			
			display_link(head_cam_link)
			cv2.imwrite('./baxterimg.jpg',head_cam_link.get_image())
			imgtosend = cv2.imread('./baxterimg.jpg')
			cx,cy,bx,by = send_image('rpij.local',5001,imgtosend)
			if(cx == None):
				continue
			cup_center_callback(cx,cy,bx,by)
			move_forward_distance = (focal_len * cup_height)/by
			print("moving forward distance of: ",move_forward_distance/8)
			#move_forward("right",move_forward_distance/8,0.0,0.0)
			print('height of the box is: ', by)
			print('width of the box is: ', bx)
			try:
				ir_dist = get_ir_distance()
				print("cup is ", ir_dist, " meters away")
				if(ir_dist < 1):
					#grip = baxter_interface.Gripper('right')
					grip.calibrate()
					rospy.sleep(2)
					move_forward("right",0.1,0.0,-0.0381)
					#ir_dist = get_ir_distance()
					#while(ir_dist > 0.1):
					#	move_forward("right",ir_dist,0.05,0.0)
					#	ir_dist = get_ir_distance()
					grip.close() # Grab cup
					rospy.sleep(3)
					right_pos = ik_get('right',  get_pose( [0.842125516735,  -0.440169063219,
0.0735905255022],[ -0.0458756226957, 0.721067691325, -0.0761771192002, 0.687134527061]) )
					limb_right.move_to_joint_positions(right_pos, timeout = 1.0) # Get away from table
					right_pos = ik_get('right',  get_pose( [0.826843880597,  -0.01,
0.052957527754],[ 0.00446905833676, 0.751319567359, -0.0419635670956, 0.658587878918]) )
					limb_right.move_to_joint_positions(right_pos) # Line up with coffee maker 
					right_pos = ik_get('right',  get_pose( [1.065097199,  -0.01,-0.085048819978],
[ -0.0864798637417, 0.797232121823, -0.0248118289732, 0.650712156416]) )
					limb_right.move_to_joint_positions(right_pos) # Place under coffee maker
					grip.open() #Place Cup
					rospy.sleep(3)
					right_pos = ik_get('right',  get_pose( [0.885999881544,  0.0,
0.0170840675894],[ 0.0215639816244, 0.780826629206, -0.0453920024769, 0.622723322135]) )
					limb_right.move_to_joint_positions(right_pos) #Move away from coffee maker
					right_pos = { 'right_e0' :   2*np.pi/5, 'right_e1' :  3*np.pi/5, 'right_s0' : 0, 'right_s1' :  -3*np.pi/10, 'right_w0' :  -np.pi/5, 'right_w1' :  3*np.pi/10, 'right_w2' :   np.pi/5}
					limb_right.move_to_joint_positions(right_pos) #Move to default position
					#right_pos = ik_get('right',  get_pose( [0.346689104635,  -0.554945461976,  -0.001543541374],[ -0.0179540327606, 0.689243036276, 0.00840373721407, 0.724258977752]) )
					#limb_right.move_to_joint_positions(right_pos) #Reset Position for testing
					break
			except Exception as e:
				rospy.logerr("Error with finding ir dist %s", e)
			rate.sleep()
	#if key == ord('z'):
		cv2.destroyAllWindows()
		head_cam_link = CameraLink(cameras[2], (640, 400), cameras[0])
		pub.publish(down_right)
    		rospy.sleep(1)
		while(head_cam_link.get_image() == None):
			pass
		while(1):
			display_link(head_cam_link)
			cv2.imwrite('./baxterimg.jpg',head_cam_link.get_image())
			imgtosend = cv2.imread('./baxterimg.jpg')
			cx,cy,bx,by = send_image('rpij.local',5001,imgtosend)
			if(cx == None):
				continue
			print(by*bx)
			if(bx * by > 15000): #signifies close enough although would like to refine this
				print("box is right size")
				move_forward("left",-0.03,0.01,-0.074)
				#suck
				suck.command_suction(timeout = 20.0)
				rospy.sleep(3)
				#Move to reasonable position
				left_pos = ik_get('left',  get_pose( [0.740488592688,  0.0506900586718,  0.179745358746],[ 0.385677692675, 0.918434515193, 0.0220751369166, -0.0851084426167]) )
				limb_left.move_to_joint_positions(left_pos)
				#position in front of coffee maker
				left_pos = ik_get('left',  get_pose( [0.844961051425,  -0.0145305739475,  0.217117615184],[ -0.0354530949479, 0.873945082545, 0, 0.481294493669]) )
				limb_left.move_to_joint_positions(left_pos)
				#place in coffee maker
				left_pos = ik_get('left',  get_pose( [1.03540600665,  -0.0171270203022,  0.120716446644],[ -0.121541808251, 0.863538699465, -0.0752181661096, 0.483601830911]) )
				limb_left.move_to_joint_positions(left_pos)
				#Unsuck
				suck.open()
				rospy.sleep(3)
				left_pos = ik_get('left',  get_pose( [0.874961051425,  -0.0145305739475,  0.217117615184],[ -0.0354530949479, 0.873945082545, -0.0576079952568, 0.481294493669]) )
				limb_left.move_to_joint_positions(left_pos)
				left_pos =  { 'left_e0'  :  -2*np.pi/5, 'left_e1'  :  3*np.pi/5, 'left_s0'  : 0, 'left_s1'  :  -3*np.pi/10, 'left_w0'  :   np.pi/5, 'left_w1'  :  3*np.pi/10, 'left_w2'  :  -np.pi/5}
				limb_left.move_to_joint_positions(left_pos)
				break
			Kcup_center_callback(cx,cy,bx,by)
			#move_forward("left",0.0,0.0,-0.1)
			rate.sleep()
	#if key ==ord('x'):
		pub.publish(default_baxter)
		rospy.sleep(1)
		right_pos = { 'right_e0' :   2*np.pi/5, 'right_e1' :  3*np.pi/5, 'right_s0' : 0, 'right_s1' :  -3*np.pi/10, 'right_w0' :  -np.pi/5, 'right_w1' :  3*np.pi/10, 'right_w2' :   np.pi/5}
		limb_right.move_to_joint_positions(right_pos)
		#Move above coffee maker
		grip.calibrate()
		rospy.sleep(2)
		right_pos = ik_get('right',  get_pose( [1.04630812739,  -0.012,
0.311012518761],[ -0.0198851834142, 0.944136290081, 0.0324115856014, 0.327354142103]) )
		limb_right.move_to_joint_positions(right_pos)
		grip.close()
		rospy.sleep(3)
		#Close coffee maker partially
		right_pos = ik_get('right',  get_pose( [0.940561294413,  -0.012,
0.0974252809088],[-0.0398131075215, 0.969647500762, -0.0526957589748,  0.235418346187]) )
		limb_right.move_to_joint_positions(right_pos)
		#Move to close mostly
		right_pos = ik_get('right',  get_pose( [0.89703411648,  -0.012,
0.126225687911],[-0.69296061468, 0.699897758176, -0.145658584667,  0.0934467298244]) )
		limb_right.move_to_joint_positions(right_pos)
		#close all the way
		right_pos = ik_get('right',  get_pose( [0.89703411648,  -0.012,
0.00],[-0.671511730442, 0.714274970592,-0.166110557039, 0.106256976734]) )
		limb_right.move_to_joint_positions(right_pos)
		right_pos = ik_get('right',  get_pose( [0.89703411648,  -0.012,
0.126225687911],[-0.69296061468, 0.699897758176, -0.145658584667,  0.0934467298244]) )
		limb_right.move_to_joint_positions(right_pos)
		right_pos = ik_get('right',  get_pose( [0.925,  -0.012,
0.005],[-0.671511730442, 0.714274970592,-0.166110557039, 0.106256976734]) )
		limb_right.move_to_joint_positions(right_pos)
		rospy.sleep(10)
		#position to press button
		right_pos = ik_get('right',  get_pose( [0.89703411648,  -0.0236694247161,
0.126225687911],[-0.69296061468, 0.699897758176, -0.145658584667,  0.0934467298244]) )
		limb_right.move_to_joint_positions(right_pos)
		#press the button
		right_pos = ik_get('right',  get_pose( [0.947852428985,  -0.0236694247161,
0.0294414445126],[-0.69296061468, 0.739470896465, -0.181808543959,  0.118372799094]) )
		limb_right.move_to_joint_positions(right_pos, timeout = 3.0)
		pub.publish(smug)
		rospy.sleep(1)
		# Line up with coffee maker 
		right_pos = ik_get('right',  get_pose( [0.816843880597,  -0.01,
0.052957527754],[ 0.00446905833676, 0.751319567359, -0.0419635670956, 0.658587878918]) )
		limb_right.move_to_joint_positions(right_pos) 
		grip.open() #Open for grabbing cup
		rospy.sleep(3)
		right_pos = ik_get('right',  get_pose( [1.065097199,  -0.01,-0.085048819978],
[ -0.0864798637417, 0.797232121823, -0.0248118289732, 0.650712156416]) )
		limb_right.move_to_joint_positions(right_pos) # Place under coffee maker
		grip.close() #Grab Cup
		rospy.sleep(45)
		pub.publish(default_baxter)
    		rospy.sleep(1)
		right_pos = ik_get('right',  get_pose( [0.816843880597,  -0.01,
0.052957527754],[ 0.00446905833676, 0.751319567359, -0.0419635670956, 0.658587878918]) )
		limb_right.move_to_joint_positions(right_pos)
		right_pos = ik_get('right',  get_pose( [0.962934308736,  -0.402781956726,
-0.115501408724],[ 0.201007587294, 0.733594611274, -0.205263070882, 0.615874961248]) )
		limb_right.move_to_joint_positions(right_pos)
		grip.open()
		rospy.sleep(3)
		right_pos = ik_get('right',  get_pose( [0.816843880597,  -0.01,
0.052957527754],[ 0.00446905833676, 0.751319567359, -0.0419635670956, 0.658587878918]) )
		limb_right.move_to_joint_positions(right_pos)
		right_pos = { 'right_e0' :   2*np.pi/5, 'right_e1' :  3*np.pi/5, 'right_s0' : 0, 'right_s1' :  -3*np.pi/10, 'right_w0' :  -np.pi/5, 'right_w1' :  3*np.pi/10, 'right_w2' :   np.pi/5}
		limb_right.move_to_joint_positions(right_pos)
		right_pos = ik_get('right',  get_pose( [0.346689104635,  -0.554945461976,  -0.001543541374],[ -0.0179540327606, 0.689243036276, 0.00840373721407, 0.724258977752]) )
		limb_right.move_to_joint_positions(right_pos)
		left_pos = ik_get('left',  get_pose( [0.692416376858,  0.358725879897,  0.426139034853 ],[ 0.06642962845, 0.997116186539, -0.0310567192663, 0.019542138916]) )
   		limb_left.move_to_joint_positions(left_pos)
	if key == ord('d'):
		print("cup is ", get_ir_distance(), " away")
        if cv2.waitKey(1) & 0xFF == ord('q'):
        	break
        rate.sleep()

def main():
    """Demonstrate access to camera output."""
#    rospy.init_node('camera_demo', anonymous=True)
    head = baxter_interface.Head()
    left_arm_view = ik_get('left',  get_pose( [0.692416376858,  0.358725879897,  0.426139034853 ],[ 0.06642962845, 0.997116186539, -0.0310567192663, 0.019542138916]) )
    right_arm_view = ik_get('right',  get_pose( [0.346689104635,  -0.554945461976,  -0.001543541374],[ -0.0179540327606, 0.689243036276, 0.00840373721407, 0.724258977752]) )
    if(left_arm_view == 0 or left_arm_view == 1 or
       right_arm_view == 0 or right_arm_view == 1):
	return
    left_pos =  { 'left_e0'  :  -2*np.pi/5, 'left_e1'  :  3*np.pi/5, 'left_s0'  : 0, 'left_s1'  :  -3*np.pi/10, 'left_w0'  :   np.pi/5, 'left_w1'  :  3*np.pi/10, 'left_w2'  :  -np.pi/5}
    right_pos = { 'right_e0' :   2*np.pi/5, 'right_e1' :  3*np.pi/5, 'right_s0' : 0, 'right_s1' :  -3*np.pi/10, 'right_w0' :  -np.pi/5, 'right_w1' :  3*np.pi/10, 'right_w2' :   np.pi/5}
    limb_left.move_to_joint_positions(left_pos)
    limb_right.move_to_joint_positions(right_pos)
    rospy.sleep(1)
    limb_left.move_to_joint_positions(left_arm_view)
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
    make_coffee()
    #for cam in cameras:
	#close_cam(cam)
  
if __name__ == '__main__':
    main()

