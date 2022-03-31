#!/usr/bin/env python
import rospy
import sensor_msgs.point_cloud2 as pc2
import tf2_ros
import tf2_py as tf2
from sensor_msgs.msg import PointCloud2
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud

pc = PointCloud2()


def callback(data):
    global pc 
    pc = data


def listener():



    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)

    mg_pub = rospy.Publisher('/sensors/slam/cloud', PointCloud2, queue_size=10)

    rospy.Subscriber("/sensors/road_marking_localization/cropped_pcl", PointCloud2, callback)

    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)


    # spin() simply keeps python from exiting until this node is stopped
    rate = rospy.Rate(10) # 10hz
    
    #print(np.rint(a))

    pc_out = PointCloud2()

    while not rospy.is_shutdown():

        try:
            trans = tf_buffer.lookup_transform("base_link", "map",
                                           pc.header.stamp,
                                           rospy.Duration(10))
        except tf2.LookupException as ex:
            rospy.logwarn(ex)
            #return
        except tf2.ExtrapolationException as ex:
            rospy.logwarn(ex)
            #return

        pc_out = do_transform_cloud(pc, trans)
        pc_out.header.frame_id = "world"

        mg_pub.publish(pc_out)
        
        #print("hello?")

        rate.sleep()


if __name__ == '__main__':
    listener()
