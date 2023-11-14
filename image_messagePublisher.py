import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2

def publish_image(img, topic_name='image_topic'):
    # Initialize your node
    rospy.init_node('image_publisher_node', anonymous=True)
    
    # Create a publisher for the 'image_topic' topic
    publisher = rospy.Publisher(topic_name, Image, queue_size=10)
    
    # Initialize the OpenCV bridge for converting between OpenCV images and ROS messages
    bridge = CvBridge()
    
    # Convert the OpenCV image to a ROS message
    try:
        image_msg = bridge.cv2_to_imgmsg(img, "bgr8")
    except CvBridgeError as e:
        rospy.logerr(e)
        return
    
    # Publish the ROS image message
    publisher.publish(image_msg)
    #cv2.imshow("",img)
    #cv2.waitKey(1)

    # Sleep to allow time for the message to be published
    rospy.sleep(1)