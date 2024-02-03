#!/usr/bin/env python 
import rospy
import tf
from tf2_geometry_msgs import PoseStamped as PS2
 

def updateQRPose(msg):          # Callback function of the object position subscriber
    global qrPose
    qrPose = msg.pose

def getQRframe():               # Function to fetch the current value of the global variable qrpose
    return qrPose
 
if __name__ == '__main__':
    qrPose = PS2()
    rospy.init_node('fixed_qr_camera_tf_broadcaster')
    rospy.Subscriber("/visp_auto_tracker/object_position", PS2, updateQRPose)
    rospy.wait_for_message("/visp_auto_tracker/object_position", PS2, 3.0)  #Waits for one single msg, timeout 3 sec
    qrPose = getQRframe()    # Gets the QR pose as seen by the camera
    br = tf.TransformBroadcaster()
    rate = rospy.Rate(10.0)
    
    while not rospy.is_shutdown():
        # Sends the transform between camera and the QR we're currently looking at ("qrframe" in rviz)
        br.sendTransform((qrPose.position.x, qrPose.position.y, qrPose.position.z),
                         (qrPose.orientation.x, qrPose.orientation.y, qrPose.orientation.z, qrPose.orientation.w),
                         rospy.Time.now(),
                         "qrframe",
                         "camera_optical_link")
        rate.sleep()

