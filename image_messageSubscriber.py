import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

def image_callback(msg):
    # Initialize the OpenCV bridge for converting between OpenCV images and ROS messages
    bridge = CvBridge()

    # Convert the ROS image message to an OpenCV image
    try:
        img = bridge.imgmsg_to_cv2(msg, "bgr8")
    except CvBridgeError as e:
        rospy.logerr(e)
        return
    
    # Display the OpenCV image using cv2.imshow()
    #print(img)
    cv2.imshow('Image', img)
    cv2.waitKey(1)

def subscribe_image(topic_name='image_topic'):
    # Initialize your node
    rospy.init_node('image_subscriber_node', anonymous=True)
    
    # Create a subscriber for the 'image_topic' topic
    rospy.Subscriber(topic_name, Image, image_callback)
    
    # Spin until the node is stopped
    rospy.spin()

if __name__ == '__main__':
    try:
        subscribe_image()
    except rospy.ROSInterruptException:
        pass