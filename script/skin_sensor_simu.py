#!/usr/bin/env python
import rospy
from dynamic_graph.sot.core.matrix_util import matrixToTuple
from dynamic_graph_bridge_msgs.msg import Vector
from dynamic_graph_bridge_msgs.msg import Matrix
#from cellularskin_msgs.msg import SkinPatch
from dynamic_graph.sot.core.utils.thread_interruptible_loop import loopInThread,loopShortcuts
import time
import numpy as np
import tf
import tf_conversions as tf_c
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
np.set_printoptions(suppress=True)


class proximity_range_bridge:
    def __init__(self):
        rospy.init_node('proximity_range_bridge', anonymous=True)
        self.listener = tf.TransformListener() 
        self.rate = rospy.Rate(10) # 10hz
        self.defineskinMarker()
        self.defineProximityData()

    def defineProximityData(self):
        self.proximity_vector = Vector()
        self.proximity_vector.data = [0.06,] *8
        self.proximity_pose = Matrix()
        self.proximity_pose.data = [0,]*56
        self.proximity_pose.width = 7
        self.pub_proximity_data= rospy.Publisher('proximity_data', Vector, queue_size=10)
        self.pub_proximity_pose = rospy.Publisher('proximity_pose', Matrix, queue_size=10)  
        self.sub_closest_pointj = rospy.Subscriber('/ca/closestPoints', Matrix, self.callback)   


    def callback(self,data):
        cp = np.array(data.data)
        cp = cp.reshape((cp.size/data.width,data.width))
        self.cpi=cp[:,0:3]
        self.cpj=cp[:,3:6]
        self.marker = Marker()
        self.marker.id = 0
        self.marker.header.frame_id = "/world"
        self.marker.type = self.marker.LINE_LIST
        self.marker.action = self.marker.ADD
        self.marker.scale.x = 0.002
        self.marker.scale.y = 0.002
        self.marker.scale.z = 0.002
        self.marker.color.a = 1.0
        self.marker.color.r = 1.0
        self.marker.header.stamp = rospy.Time.now()
        for i in range(8):
            pi = Point();pi.x = self.cpi[i,0];pi.y = self.cpi[i,1];pi.z = self.cpi[i,2];
            self.marker.points.append(pi)
            pj = Point();pj.x = self.cpj[i,0];pj.y = self.cpj[i,1];pj.z = self.cpj[i,2];
            self.marker.points.append(pj)  
        self.skin_pub.publish(self.marker)  



    def defineskinMarker(self): 
        self.skin_pub = rospy.Publisher('skin_normals', Marker, queue_size=10)
 
    def run(self):

        try:
            (trans0,rot0) = self.listener.lookupTransform('/world', '/forerarm_skin_link_0', rospy.Time(0))
            self.l0 = trans0+rot0
            #print l0
            (trans1,rot1) = self.listener.lookupTransform('/world', '/forerarm_skin_link_1', rospy.Time(0))   
            l1 = trans1+rot1                
            (trans2,rot2) = self.listener.lookupTransform('/world', '/forerarm_skin_link_2', rospy.Time(0))
            l2 = trans2+rot2       
            (trans3,rot3) = self.listener.lookupTransform('/world', '/forerarm_skin_link_3', rospy.Time(0)) 
            l3 = trans3+rot3                  
            (trans4,rot4) = self.listener.lookupTransform('/world', '/forerarm_skin_link_4', rospy.Time(0))
            l4 = trans4+rot4
            (trans5,rot5) = self.listener.lookupTransform('/world', '/forerarm_skin_link_5', rospy.Time(0))  
            l5 = trans5+rot5                 
            (trans6,rot6) = self.listener.lookupTransform('/world', '/forerarm_skin_link_6', rospy.Time(0))
            l6 = trans6+rot6
            (trans7,rot7) = self.listener.lookupTransform('/world', '/forerarm_skin_link_7', rospy.Time(0))  
            l7 = trans7+rot7       
            self.proximity_pose.data = list(self.l0+l1+l2+l3+l4+l5+l6+l7)        
        except (tf.LookupException, tf.ConnectivityException):
            print 'error looking up transforms' 
            #self.cyl_pub.publish(self.cyl)
        self.pub_proximity_pose.publish(self.proximity_pose)   
        self.pub_proximity_data.publish(self.proximity_vector)  
        self.rate.sleep()


# Controller Thread
prb = proximity_range_bridge()

@loopInThread
def pub():
    prb.run()


runner=pub()
runner.once()
[go,stop,next,n]=loopShortcuts(runner)

def slowupanddown():
    for j in range(10): 
        prb.proximity_vector.data[6] -=  0.001
        time.sleep(0.3)
    time.sleep(4)    
    for i in range(15): 
    	prb.proximity_vector.data[6] += 0.001
        time.sleep(0.3)

        





