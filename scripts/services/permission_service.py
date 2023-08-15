#!/usr/bin/env python
import os
import stat
import rospy
from tiago_hrc.srv import permission, permissionResponse

def callback(request):
    '''Make the request path executable (chmod +x)'''
    try:
        st = os.stat(request.path)
        os.chmod(request.path, st.st_mode | stat.S_IEXEC)

        rospy.loginfo('Setting permission for %s', request.file)

        return permissionResponse(True)
    except Exception:
        return permissionResponse(False)


def start():
    rospy.init_node('permission_service')
    rospy.Service("permission", permission, callback)
    rospy.loginfo('Service "permission_service" started')
    rospy.spin()


if __name__ == '__main__':
    start()