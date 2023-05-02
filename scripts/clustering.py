
import rospy
import numpy as np
import copy
from sklearn.cluster import KMeans
from sklearn import mixture

from hrov_martech2023.msg import PointArray
from visualization_msgs.msg import Marker, MarkerArray, InteractiveMarker, InteractiveMarkerControl
from interactive_markers.interactive_marker_server import *
from interactive_markers.menu_handler import *

from geometry_msgs.msg import Pose

import tf.transformations as tr
import spatialmath as sm
import spatialmath.base as smb

import plotly.graph_objects as go

def processFeedback(feedback: InteractiveMarkerFeedback):
    p = feedback.pose.position
    print(feedback.marker_name + " is now at " + str(p.x) + ", " + str(p.y) + ", " + str(p.z))

def plot_covar(fig: go.Figure, centroides, covar_matrix):
    eval,evec=np.linalg.eig(covar_matrix)
    # print("values:\n", eval)
    # print("vectors:\n", evec)
    a, b, c = eval[0], eval[1], eval[2]
    u, v = np.mgrid[0:2*np.pi:40j, 0:np.pi:20j]
    x1 = a * np.cos(u) * np.sin(v)
    y1 = b * np.sin(u) * np.sin(v)
    z1 = c * np.cos(v)
    # points on the ellipsoid
    points = np.stack([t.flatten() for t in [x1, y1, z1]])

    v1=evec[0]
    v2=evec[1]
    v3=evec[2]
    # 3x3 transformation matrix
    T = np.array([v1, v2, v3])

    new_points = T @ points
    x2 = new_points[0, :]
    y2 = new_points[1, :]
    z2 = new_points[2, :]
    x2, y2, z2 = [t.reshape(x1.shape) for t in [x2, y2, z2]]

    fig.add_trace(go.Surface(x=x2+centroides[0], y=y2+centroides[1], z=z2+centroides[2], opacity=0.35,
                         colorscale="aggrnyl", surfacecolor=y1,showscale=False,
                         contours = {
                            "x": {"highlight":False},
                            "y": {"highlight":False},
                            "z": {"highlight":False}},
                        hoverinfo="none",
                        legendgroup="centroides",
                        showlegend=True,
                        name="Covars"))
    
    
    print(eval)
    min=np.argmin(eval)
    print(min)
    print(evec)

    fig.add_trace(go.Scatter3d(x=[centroides[0],(evec[0][min])*0.5+centroides[0]],
                               y=[centroides[1],(evec[1][min])*0.5+centroides[1]],
                               z=[centroides[2],(evec[2][min])*0.5+centroides[2]],
                               line={"color":"darkblue","width":4},
                               marker={"size":4,"symbol":"cross"}))
    


rospy.init_node("clustering_node", anonymous=True)

marker_pub = rospy.Publisher("/kcentroids", MarkerArray, queue_size=0)
pt_array_msg = rospy.wait_for_message("/octo/unk_cells", PointArray)

lista = []
for pt in pt_array_msg.puntos:
    lista.append([pt.x, pt.y, pt.z])

X = np.around(np.array(lista), 1)

fig=go.Figure()

dpgmm = mixture.BayesianGaussianMixture(n_components=10, 
                                        covariance_type="full",
                                        # mean_precision_prior=2,
                                        # tol=1e-4,
                                        # tol=1
                                        ).fit(X)

fig.add_trace(go.Scatter3d(x=X[:,0],y=X[:,1],z=X[:,2], mode='markers',
    marker=dict(size=8,color=dpgmm.predict(X),colorscale='Viridis',opacity=0.8),name="Puntos"))

# print(dpgmm.means_)
# print(dpgmm.mean_precision_)

# print(dpgmm.weights_)
index=np.where(dpgmm.weights_<1e-02)
dpgmm.means_=np.delete(dpgmm.means_,index,axis=0)
# print(dpgmm.means_)
dpgmm.covariances_=np.delete(dpgmm.covariances_,index,axis=0)
# print(dpgmm.covariances_)

for mean, covar in zip(dpgmm.means_,dpgmm.covariances_):
    plot_covar(fig, mean, covar)


centroides=dpgmm.means_
fig.add_trace(go.Scatter3d(x=centroides[:, 0],y=centroides[:, 1],z=centroides[:, 2],mode='markers',
    marker=dict(size=8,color='red',opacity=0.8),name="Centers"))

fig.update_layout(margin=dict(l=0, b=0,t=0,r=0),
                  title_font_family="Times New Roman",
                  title_font_color="darkred",
                  title=dict(text="Frontiers: All directions3 <br><sup>Clustering Params: default</sup>", 
                             font=dict(size=50), automargin=True, yref='paper')
                #   ,scene={"aspectmode":'cube'}
                )
# fig.show()
fig.write_html("src/hrov_martech2023/graficos/test.html",include_plotlyjs='directory')
# fig.write_html("src/hrov_martech2023/graficos/graph2.html", include_plotlyjs=False, full_html=False)

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

print("\nploteando los esferoides\n")

for idx, (mean, covar) in enumerate(zip(dpgmm.means_,dpgmm.covariances_)):
    print(idx)
    val,vec=np.linalg.eig(covar)
    marker.header.stamp = rospy.Time.now()
    marker.id = idx
    marker.pose.position.x = mean[0]
    marker.pose.position.y = mean[1]
    marker.pose.position.z = mean[2]
    # marker.scale.x = val[0]
    # marker.scale.y = val[1]
    # marker.scale.z = val[2]
    marker.scale.x = 0.5
    marker.scale.y = 0.5
    marker.scale.z = 0.5

    print("covar\n ", covar)
    print(smb.isrot(covar))
    print("eigenvecs\n ", vec)
    print(smb.isrot(vec))
    
    transformada=sm.SE3().Rt(R=sm.SO3(vec))
    print(transformada)
    # print(smb.r2q(T.R,order="xyzs"))
    smb.qprint(smb.r2q(transformada.R))
    
    q=tr.quaternion_from_matrix(transformada)
    q=q/np.linalg.norm(q)
    
    print(q)

    marker.pose.orientation.x = 0
    marker.pose.orientation.y = 0
    marker.pose.orientation.z = 0
    marker.pose.orientation.w = 1

    marker_array.markers.append(copy.deepcopy(marker))

    # break

marker_pub.publish(marker_array)

######################################33
server = None
marker_pos = 0

menu_handler = MenuHandler()

h_first_entry = 0
h_mode_last = 0

def makeBox( msg ):
    marker = Marker()

    marker.type = Marker.CUBE
    marker.scale.x = msg.scale * 0.45
    marker.scale.y = msg.scale * 0.45
    marker.scale.z = msg.scale * 0.45
    marker.color.r = 0.5
    marker.color.g = 0.5
    marker.color.b = 0.5
    marker.color.a = 1.0

    return marker

def makeBoxControl( msg ):
    control = InteractiveMarkerControl()
    control.always_visible = True
    control.markers.append( makeBox(msg) )
    msg.controls.append( control )
    return control

def makeEmptyMarker( dummyBox=True ):
    global marker_pos
    int_marker = InteractiveMarker()
    int_marker.header.frame_id = "world"
    int_marker.pose.position.y = -3.0 * marker_pos
    marker_pos += 1
    int_marker.scale = 1
    return int_marker

def makeMenuMarker( name ):
    int_marker = makeEmptyMarker()
    int_marker.name = name

    control = InteractiveMarkerControl()

    control.interaction_mode = InteractiveMarkerControl.BUTTON
    control.always_visible = True

    control.markers.append( makeBox( int_marker ) )
    int_marker.controls.append(control)

    server.insert( int_marker )

def deepCb( feedback: InteractiveMarkerFeedback ):
    rospy.loginfo("The deep sub-menu has been found.")
    rospy.loginfo(feedback.marker_name)

def initMenu():
    global h_first_entry, h_mode_last
    h_first_entry = menu_handler.insert( "Select as goal",callback=deepCb )
    h_first_entry = menu_handler.insert( "Request Path",callback=deepCb )



server = InteractiveMarkerServer("menu")

initMenu()

makeMenuMarker( "marker1" )
makeMenuMarker( "marker2" )

menu_handler.apply( server, "marker1" )
menu_handler.apply( server, "marker2" )
server.applyChanges()

rospy.spin()
