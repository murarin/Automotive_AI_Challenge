from geometry_msgs.msg import PoseWithCovarianceStamped
import rospy
import time

if __name__ == "__main__":
    # Initialize publisher node
    rospy.init_node("initial_pose_pub", anonymous=True)
    publisher = rospy.Publisher("initialpose", PoseWithCovarianceStamped, queue_size=1, latch=True)
    # Run publisher
    msg = PoseWithCovarianceStamped()
    msg.header.frame_id = "map"
    msg.pose.pose.position.x = 21.674
    msg.pose.pose.position.y = -47.98
    msg.pose.pose.position.z = 0.0
    msg.pose.pose.orientation.x = -0.0061
    msg.pose.pose.orientation.y = 0.01226
    msg.pose.pose.orientation.z = 0.7073
    msg.pose.pose.orientation.w = 0.7067
    msg.pose.covariance[0] = 0.25
    msg.pose.covariance[7] = 0.25
    msg.pose.covariance[35] = 0.06853892326654787
    time.sleep(1)
    publisher.publish(msg)
    rospy.spin()
