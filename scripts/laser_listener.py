import rospy
import math
import copy
import matplotlib.pyplot as plt
from sensor_msgs.msg import LaserScan
from fetch_api import Base

left_27_degree_index = 331 - 27 * 3
right_27_degree_index = 331 + 27 * 3

def print_laser(data):
    global base
    n = 15
    index_list = [] 
    data.ranges = data.ranges[left_27_degree_index:right_27_degree_index]
    data.angle_min = math.radians(-27)
    data.angle_max = math.radians(27)
    pub.publish(data)
    counter = 0
    start = None
    end = None
    for i, dist in enumerate(data.ranges):
        if dist > 2:
            if start is None:
                start = i
            counter += 1
        if dist < 2 and counter >= n:
            end = i
            index_list.append((start,end))
            counter = 0
            start = None
            end = None
        elif dist < 2 and counter < n:
            counter = 0
            start = None
            end = None
    if counter >= n:
        index_list.append((start,len(data.ranges)))
    target_angle = -27 + ((index_list[0][0] + index_list[0][1]) / 2) * 0.33333
    # print index_list, ((index_list[0][0] + index_list[0][1]) / 2), len(data.ranges)
    # gap = copy.deepcopy(data)
    # empty_list = []
    # for i in range(len(data.ranges)):
    #     if i < index_list[0][0] or i > index_list[0][1]:
    #         empty_list.append(0)
    #     else:
    #         empty_list.append(data.ranges[i])
    # gap.ranges = empty_list
    # pub2.publish(gap)
    print index_list, -math.radians(target_angle)
    if abs(target_angle) > 5:
        base.turn(-math.radians(target_angle), speed = 0.5)
    base.go_forward(2, 0.5)
    # plt.plot(data.ranges[left_30_degree_index:right_30_degree_index])
    # plt.show()
    # for i, dist in enumerate(data.ranges[left_30_degree_index:right_30_degree_index]):
    #     print math.acos(math.radians(-30 + 0.33333 * i)) * dist


rospy.init_node('laser_listener')
base = Base()
# sub = rospy.Subscriber("base_scan", LaserScan, print_laser)
pub = rospy.Publisher("trim_laser", LaserScan, queue_size = 1)
pub2 = rospy.Publisher("gap_laser", LaserScan, queue_size = 1)

data = rospy.wait_for_message('base_scan', LaserScan, timeout=5)
print_laser(data)
rospy.spin()