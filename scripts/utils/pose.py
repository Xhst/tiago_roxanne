import rospy
import math
from geometry_msgs.msg import PoseStamped
from tf.transformations import quaternion_from_euler


def posestamped_from_positions_and_euler_angles(x, y, z, ax, ay, az):
    pose_stamped = PoseStamped()

    pose_stamped.header.frame_id = 'map'
    pose_stamped.header.stamp = rospy.Time.now()

    pose_stamped.pose.position.x = x
    pose_stamped.pose.position.y = y
    pose_stamped.pose.position.z = z

    quaternion = quaternion_from_euler(ax, ay, az)
    pose_stamped.pose.orientation.x = quaternion[0]
    pose_stamped.pose.orientation.y = quaternion[1]
    pose_stamped.pose.orientation.z = quaternion[2]
    pose_stamped.pose.orientation.w = quaternion[3]

    return pose_stamped


def posestamped_from_xy_and_cardinal_point(x,y, cardinal_point):
    angle = angle_from_cardinal_point(cardinal_point)
    return posestamped_from_positions_and_euler_angles(x, y, 0.0, 0.0, 0.0, angle)


def angle_from_cardinal_point(cardinal_point):
    if cardinal_point == 'north_west':
        return 7.0/4 * math.pi # 7pi/4
    if cardinal_point == 'north':
        return 3.0/2 * math.pi # 3pi/2
    if cardinal_point == 'north_east':
        return 5.0/4 * math.pi # 5pi/4
    if cardinal_point == 'east':
        return math.pi # pi
    if cardinal_point == 'south_east':
        return 3.0/4 * math.pi # 3pi/4
    if cardinal_point == 'south':
        return 1.0/2 * math.pi # pi/2
    if cardinal_point == 'south_west':
        return 1.0/4 * math.pi # pi/4

    # west as default return
    return 0