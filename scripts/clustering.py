import rospy
import numpy as np
import copy
from sklearn.cluster import KMeans
from sklearn import mixture

from hrov_martech2023.msg import PointArray
from hrov_martech2023.srv import PlanGoal, PlanGoalRequest
from visualization_msgs.msg import (
    Marker,
    MarkerArray,
    InteractiveMarker,
    InteractiveMarkerControl,
)
from interactive_markers.interactive_marker_server import *
from interactive_markers.menu_handler import *

from geometry_msgs.msg import Pose


import tf2_ros
from tf.transformations import euler_from_quaternion

import roboticstoolbox as rb
import spatialmath.base as sm
from spatialmath import *

rospy.init_node("clustering_node", anonymous=True)

reqPath = rospy.ServiceProxy("getPath", PlanGoal)

tfBuffer = tf2_ros.Buffer()
listener = tf2_ros.TransformListener(tfBuffer)

while not rospy.is_shutdown() and True:
    try:
        trans = tfBuffer.lookup_transform(
            "girona1000/laser_link", "girona1000/base_link", rospy.Time()
        )
        print(trans)
        break
    except (
        tf2_ros.LookupException,
        tf2_ros.ConnectivityException,
        tf2_ros.ExtrapolationException,
    ):
        rospy.Rate(10).sleep()
        continue

######################################
server = None
menu_handler = MenuHandler()


def makeMenuMarker(name, pos):
    int_marker = InteractiveMarker()
    int_marker.header.frame_id = "world_ned"
    int_marker.pose.position.x = pos[0]
    int_marker.pose.position.y = pos[1]
    int_marker.pose.position.z = pos[2]
    int_marker.scale = 1
    int_marker.name = name

    control = InteractiveMarkerControl()
    control.interaction_mode = InteractiveMarkerControl.BUTTON
    control.always_visible = True

    marker = Marker()
    marker.type = Marker.SPHERE
    marker.scale.x = 0.5
    marker.scale.y = 0.5
    marker.scale.z = 0.5
    marker.color.r = 1
    marker.color.g = 0.796
    marker.color.b = 0.0
    marker.color.a = 1
    control.markers.append(marker)

    marker = Marker()
    marker.type = Marker.SPHERE
    marker.pose.position.x = trans.transform.translation.y
    marker.pose.position.z = trans.transform.translation.z
    marker.scale.x = 0.1
    marker.scale.y = 0.1
    marker.scale.z = 0.1
    marker.color.r = 1
    marker.color.g = 1
    marker.color.b = 1
    marker.color.a = 0
    control.markers.append(marker)

    marker = Marker()
    marker.type = Marker.ARROW
    marker.pose.position.x = -0.6
    marker.pose.position.z = -0.16
    marker.pose.orientation.y = -0.131
    marker.pose.orientation.w = 0.991
    marker.scale.x = 0.3
    marker.scale.y = 0.1
    marker.scale.z = 0.1
    marker.color.r = 0
    marker.color.g = 0.796
    marker.color.b = 1
    marker.color.a = 0
    control.markers.append(marker)

    int_marker.controls.append(copy.deepcopy(control))

    control = InteractiveMarkerControl()
    control.name = "move_x"
    control.orientation_mode = InteractiveMarkerControl.FIXED
    control.interaction_mode = InteractiveMarkerControl.NONE
    int_marker.controls.append(copy.deepcopy(control))

    control = InteractiveMarkerControl()
    control.orientation.w = 1
    control.orientation.x = 0
    control.orientation.y = 0
    control.orientation.z = 1
    control.name = "move_y"
    control.orientation_mode = InteractiveMarkerControl.FIXED
    control.interaction_mode = InteractiveMarkerControl.NONE
    int_marker.controls.append(copy.deepcopy(control))

    control = InteractiveMarkerControl()
    control.orientation.w = 1
    control.orientation.x = 0
    control.orientation.y = 1
    control.orientation.z = 0
    control.name = "move_z"
    control.orientation_mode = InteractiveMarkerControl.FIXED
    control.interaction_mode = InteractiveMarkerControl.NONE
    int_marker.controls.append(copy.deepcopy(control))

    control.name = "rotate_z"
    control.interaction_mode = InteractiveMarkerControl.NONE
    int_marker.controls.append(copy.deepcopy(control))

    server.insert(int_marker, markerCB)
    # server.insert(int_marker)


def markerCB(feedback: InteractiveMarkerFeedback):
    # print("left click clicked\n")
    # print(feedback.pose)
    feedback.client_id = 0


def menuCB(feedback: InteractiveMarkerFeedback):
    rospy.loginfo("Requesting path.")
    rospy.loginfo(feedback.marker_name)
    rospy.loginfo(feedback.menu_entry_id)
    # print(feedback.pose)

    # m1 = sm.transl(
    #     feedback.pose.position.x, feedback.pose.position.y, feedback.pose.position.z
    # )
    q1 = np.array(
        [
            feedback.pose.orientation.x,
            feedback.pose.orientation.y,
            feedback.pose.orientation.z,
            feedback.pose.orientation.w,
        ]
    ).round(3)

    q1 = q1 / np.linalg.norm(q1)

    print(sm.q2r(q1, order="xyzs"))

    m1 = SE3.Rt(
        SO3(sm.q2r(q1, order="xyzs")),
        [feedback.pose.position.x, feedback.pose.position.y, feedback.pose.position.z],
    )
    # sm.trprint(m1)
    print(m1)

    m2 = SE3(
        sm.transl(
            trans.transform.translation.y,
            trans.transform.translation.x,
            trans.transform.translation.z,
        )
    )
    print(m2)
    print(m1 * m2)
    m = SE3(m1 * m2)

    req = PlanGoalRequest()
    # req.position.x = feedback.pose.position.x + trans.transform.translation.y
    # req.position.y = feedback.pose.position.y + trans.transform.translation.x
    # req.position.z = feedback.pose.position.z + trans.transform.translation.z
    # req.position.x = feedback.pose.position.x
    # req.position.z = feedback.pose.position.z
    # req.position.y = feedback.pose.position.y
    req.position.x = m.t[0]
    req.position.y = m.t[1]
    req.position.z = m.t[2]

    # req.yaw = euler_from_quaternion(
    #     [
    #         feedback.pose.orientation.x,
    #         feedback.pose.orientation.y,
    #         feedback.pose.orientation.z,
    #         feedback.pose.orientation.w,
    #     ]
    # )[2]

    req.yaw = m.rpy()[2]
    print(m.rpy())
    print(req)
    reqPath(req)


def modCB(feedback: InteractiveMarkerFeedback):
    rospy.loginfo("Goal Selected.")
    rospy.loginfo(feedback.marker_name)
    rospy.loginfo(feedback.menu_entry_id)

    im = server.get("marker0")
    i = 0
    while im != None:
        im.controls[0].markers[0].color.b = 0
        im.controls[0].markers[1].color.a = 0
        im.controls[0].markers[2].color.a = 0
        im.controls[1].interaction_mode = InteractiveMarkerControl.NONE
        im.controls[2].interaction_mode = InteractiveMarkerControl.NONE
        im.controls[3].interaction_mode = InteractiveMarkerControl.NONE
        im.controls[4].interaction_mode = InteractiveMarkerControl.NONE
        server.insert(im)
        i = i + 1
        im = server.get("marker" + str(i))

    server.applyChanges()

    im = server.get(feedback.marker_name)
    im.controls[0].markers[0].color.b = 1
    im.controls[0].markers[1].color.a = 1
    im.controls[0].markers[2].color.a = 1
    im.controls[1].interaction_mode = InteractiveMarkerControl.MOVE_AXIS
    im.controls[2].interaction_mode = InteractiveMarkerControl.MOVE_AXIS
    im.controls[3].interaction_mode = InteractiveMarkerControl.MOVE_AXIS
    im.controls[4].interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
    server.insert(im)
    server.applyChanges()


def initMenu():
    menu_handler.insert("Select as goal", callback=modCB)
    menu_handler.insert("Request Path", callback=menuCB)
    menu_handler.insert("Move here", callback=menuCB)


server = InteractiveMarkerServer("menu")
initMenu()


marker_pub = rospy.Publisher("/kcentroids", MarkerArray, queue_size=0)
pt_array_msg = rospy.wait_for_message("/octo/unk_cells", PointArray)

lista = []
for pt in pt_array_msg.puntos:
    lista.append([pt.x, pt.y, pt.z])

X = np.around(np.array(lista), 1)

dpgmm = mixture.BayesianGaussianMixture(
    n_components=10,
    covariance_type="full",
    # mean_precision_prior=2,
    # tol=1e-4,
    # tol=1
).fit(X)

# print(dpgmm.means_)
# print(dpgmm.mean_precision_)

# print(dpgmm.weights_)
index = np.where(dpgmm.weights_ < 1e-02)
dpgmm.means_ = np.delete(dpgmm.means_, index, axis=0)
# print(dpgmm.means_)
dpgmm.covariances_ = np.delete(dpgmm.covariances_, index, axis=0)
# print(dpgmm.covariances_)

marker_array = MarkerArray()
marker = Marker()
marker.header.frame_id = "world_ned"
marker.ns = "centroids"
marker.type = Marker.SPHERE
marker.action = Marker.ADD

marker.color.r = 1
marker.color.g = 0.796
marker.color.b = 0.0
marker.color.a = 1
# marker.pose.orientation.w = 1

print("\nDibujando los esferoides\n")


for idx, (mean, covar) in enumerate(zip(dpgmm.means_, dpgmm.covariances_)):
    print(idx)

    makeMenuMarker("marker" + str(idx), mean)
    menu_handler.apply(server, "marker" + str(idx))

    # break

server.applyChanges()

# marker_pub.publish(marker_array)


rospy.spin()
