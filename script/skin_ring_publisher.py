from dynamic_graph_bridge_msgs.msg import Vector
import rospy

class RingPublisher:
    def __init__(self,name):
        self.ring_publisher = rospy.Publisher('sot_collision_distance', Vector)   
        self.size = 11; 
        message = [0.06,]*self.size
        message[5] = 0.03
        self.message = tuple(message)        
           
if __name__ == '__main__':
    rospy.init_node('ring_publisher')
    ring_pub = RingPublisher(rospy.get_name())
    while not rospy.is_shutdown():
        ring_pub.ring_publisher.publish(ring_pub.message)



