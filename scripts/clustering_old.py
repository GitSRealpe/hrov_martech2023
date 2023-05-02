
import rospy
import numpy as np
import copy
from sklearn.cluster import KMeans
from sklearn import mixture

from hrov_martech2023.msg import PointArray
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point

import matplotlib.pyplot as plt

rospy.init_node("clustering_node", anonymous=True)

marker_pub = rospy.Publisher("/kcentroids", MarkerArray, queue_size=0)

pt_array_msg = rospy.wait_for_message("/octo/unk_cells", PointArray)

lista = []
for pt in pt_array_msg.puntos:
    lista.append([pt.x, pt.y, pt.z])

X = np.around(np.array(lista), 1)

fig = plt.figure()
fig.set_size_inches(10,10)
ax = fig.add_subplot(projection="3d")
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')

# kmeans = KMeans(n_clusters=2, random_state=0, n_init="auto").fit(X)
# centroides=kmeans.cluster_centers_

# gmm=mixture.GaussianMixture(n_components=5,covariance_type="full").fit(X)
# centroides=gmm.means_

dpgmm = mixture.BayesianGaussianMixture(n_components=5, covariance_type="full").fit(X)
ax.scatter(X[:, 0], X[:, 1], X[:,2],s=40,c=dpgmm.predict(X),cmap="viridis")
print(dpgmm.n_features_in_)
print(dpgmm.means_)
# print(dpgmm.covariances_)
w,v=np.linalg.eig(dpgmm.covariances_)
print("values:\n", w)
print("vectors:\n", v)

# centroides = dpgmm.means_
# print(centroides)
# print(len(centroides))
# print(np.unique(np.around(centroides, 2), axis=0))
centroides = np.unique(np.around(dpgmm.means_, 1), axis=0)
ax.scatter(centroides[:, 0], centroides[:, 1], centroides[:, 2],s=40, c="red")

x_surf=np.arange(0, 1, 0.01)                # generate a mesh
y_surf=np.arange(0, 1, 0.01)
x_surf, y_surf = np.meshgrid(x_surf, y_surf)
z_surf = np.sqrt(x_surf+y_surf)             # ex. function, which depends on x and y
ax.plot_surface(x_surf, y_surf, z_surf, cmap="hot")

marker_array = MarkerArray()
marker = Marker()
marker.header.frame_id = "world"
marker.ns = "centroids"
marker.type = Marker().SPHERE
marker.action = Marker().ADD
marker.scale.x = 1
marker.scale.y = 1.5
marker.scale.z = 0.5
marker.color.r = 1
marker.color.g = 0.796
marker.color.b = 0.0
marker.color.a = 1
marker.pose.orientation.w = 1

for idx, c in enumerate(centroides):
    marker.header.stamp = rospy.Time.now()
    marker.id = idx
    marker.pose.position.x = c[0]
    marker.pose.position.y = c[1]
    marker.pose.position.z = c[2]

    marker_array.markers.append(copy.deepcopy(marker))

marker_pub.publish(marker_array)

plt.show()
