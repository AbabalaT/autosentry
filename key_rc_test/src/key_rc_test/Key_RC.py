import keyboard
import rospy
from geometry_msgs.msg import Twist

def callback(x):
    buff = Twist()
    if x.event_type == 'down' and x.name == 'w':
        buff.linear.x=1.0
    if x.event_type == 'down' and x.name == 's':
        buff.linear.x=-1.0
    if x.event_type == 'down' and x.name == 'a':
        buff.linear.y=1.0
    if x.event_type == 'down' and x.name == 'd':
        buff.linear.y=-1.0
    if x.event_type == 'down' and x.name == 'q':
        buff.angular.z=1.0
    if x.event_type == 'down' and x.name == 'e':
        buff.angular.z=-1.0
    if x.event_type == 'up' and x.name == 'w':
        buff.linear.x=0.0
    if x.event_type == 'up' and x.name == 's':
        buff.linear.x=0.0
    if x.event_type == 'up' and x.name == 'a':
        buff.linear.y=0.0
    if x.event_type == 'up' and x.name == 'd':
        buff.linear.y=0.0
    if x.event_type == 'up' and x.name == 'q':
        buff.angular.z=0.0
    if x.event_type == 'up' and x.name == 'e':
        buff.angular.z=0.0
    twist_pub.publish(buff)

if __name__ == '__main__':
    keyboard.hook(callback)
    rospy.init_node('key_rc_test_node')
    twist_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    rospy.spin()

