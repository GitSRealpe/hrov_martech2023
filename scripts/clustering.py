import rospy
import numpy as np
import copy
from sklearn.cluster import KMeans
from sklearn import mixture

from hrov_martech2023.msg import PointArray
from visualization_msgs.msg import (
    Marker,
    MarkerArray,
    InteractiveMarker,
    InteractiveMarkerControl,
)
from interactive_markers.interactive_marker_server import *
from interactive_markers.menu_handler import *

from geometry_msgs.msg import Pose

import tf.transformations as tr
import spatialmath as sm
import spatialmath.base as smb

import plotly.graph_objects as go

rospy.init_node("clustering_node", anonymous=True)

######################################
server = None
menu_handler = MenuHandler()

h_first_entry = 0
h_mode_last = 0


def makeMenuMarker(name, pos):
    int_marker = InteractiveMarker()
    int_marker.header.frame_id = "world"
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

    server.insert(int_marker)


def deepCb(feedback: InteractiveMarkerFeedback):
    rospy.loginfo("The deep sub-menu has been found.")
    rospy.loginfo(feedback.marker_name)
    rospy.loginfo(feedback.menu_entry_id)
    print(feedback.pose)
    im = server.get(feedback.marker_name)
    im.controls[0].markers[0].color.b = 1
    server.insert(im)
    server.applyChanges()


def initMenu():
    global h_first_entry, h_mode_last
    h_first_entry = menu_handler.insert("Select as goal", callback=deepCb)
    h_first_entry = menu_handler.insert("Request Path", callback=deepCb)


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
marker.header.frame_id = "world"
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
    # val, vec = np.linalg.eig(covar)
    # marker.header.stamp = rospy.Time.now()
    # marker.id = idx
    # marker.pose.position.x = mean[0]
    # marker.pose.position.y = mean[1]
    # marker.pose.position.z = mean[2]
    # marker.scale.x = 0.5
    # marker.scale.y = 0.5
    # marker.scale.z = 0.5

    # marker.pose.orientation.x = 0
    # marker.pose.orientation.y = 0
    # marker.pose.orientation.z = 0
    # marker.pose.orientation.w = 1

    # marker_array.markers.append(copy.deepcopy(marker))

    makeMenuMarker("marker" + str(idx), mean)
    menu_handler.apply(server, "marker" + str(idx))

    # break

server.applyChanges()

# marker_pub.publish(marker_array)


rospy.spin()
