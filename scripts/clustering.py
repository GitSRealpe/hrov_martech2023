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

rospy.init_node("clustering_node", anonymous=True)

reqPath = rospy.ServiceProxy("getPath", PlanGoal)

######################################
server = None
menu_handler = MenuHandler()

h_first_entry = 0
h_mode_last = 0


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
    int_marker.controls.append(control)

    server.insert(int_marker, markerCB)


def markerCB(feedback: InteractiveMarkerFeedback):
    print("left click clicked\n")


def menuCB(feedback: InteractiveMarkerFeedback):
    # if feedback.event_type == feedback.MENU_SELECT:
    rospy.loginfo("Clicked menu.")
    rospy.loginfo(feedback.marker_name)
    rospy.loginfo(feedback.menu_entry_id)
    print(feedback.pose)
    # im = server.get(feedback.marker_name)
    # im.controls[0].markers[0].color.b = 1
    # server.insert(im)
    # server.applyChanges()
    req = PlanGoalRequest()
    req.position = feedback.pose.position
    req.yaw = 1.57
    reqPath(req)
    # res = reqPath(req)
    # print(res)


def initMenu():
    global h_first_entry, h_mode_last
    h_first_entry = menu_handler.insert("Select as goal", callback=menuCB)
    h_first_entry = menu_handler.insert("Request Path", callback=menuCB)


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
