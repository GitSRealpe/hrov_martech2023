#include <ros/ros.h>
#include <cmath>
#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <octomap_msgs/conversions.h>
#include <octomap_msgs/OctomapWithPose.h>

#include <visualization_msgs/MarkerArray.h>

struct Dato
{
    int cluster_id;
    double min_dist;
    octomap::point3d punto;

    Dato() : cluster_id(-1),
             min_dist(__DBL_MAX__),
             punto(octomap::point3d())
    {
    }
    Dato(octomap::point3d pt) : cluster_id(-1),
                                min_dist(__DBL_MAX__),
                                punto(pt)
    {
    }
    double distance(Dato dato)
    {
        return dato.punto.distance(punto);
        // return (p.x - x) * (p.x - x) + (p.y - y) * (p.y - y);
    }
};

// std::vector<Dato> newCentroids(std::vector<Dato> *puntos, std::vector<Dato> centroids)
std::vector<Dato> newCentroids(std::shared_ptr<std::vector<Dato>> puntos, std::vector<Dato> centroids)
{
    // start the centroid realocation
    std::vector<int> nPoints;
    std::vector<float> sumX, sumY, sumZ;
    // Initialise with zeroes
    for (int j = 0; j < centroids.size(); ++j)
    {
        nPoints.push_back(0);
        sumX.push_back(0.0);
        sumY.push_back(0.0);
        sumZ.push_back(0.0);
    }

    std::vector<Dato>::iterator ptsEnd = std::end(*puntos);
    // sumar puntos de cada cluster para sacar el promedio de cada cluster
    for (std::vector<Dato>::iterator it = std::begin(*puntos); it != ptsEnd; ++it)
    {
        int clusterId = it->cluster_id;
        nPoints[clusterId] += 1;
        sumX[clusterId] += it->punto.x();
        sumY[clusterId] += it->punto.y();
        sumZ[clusterId] += it->punto.z();

        it->min_dist = __DBL_MAX__; // reset distance
    }

    // el promedio de cada cluster sera el nuevo centroide
    for (std::vector<Dato>::iterator c = begin(centroids); c != end(centroids); ++c)
    {
        int clusterId = c - begin(centroids);
        c->punto.x() = sumX[clusterId] / nPoints[clusterId];
        c->punto.y() = sumY[clusterId] / nPoints[clusterId];
        c->punto.z() = sumZ[clusterId] / nPoints[clusterId];
    }
    return centroids;
}

// std::vector<Dato> kmeans(std::vector<Dato> *puntos, int k)
std::vector<Dato> kmeans(std::shared_ptr<std::vector<Dato>> puntos, int k)
{
    std::cout << "starting kmeans\n";
    std::vector<Dato> centroids;
    std::vector<Dato>::iterator ptsEnd = std::end(*puntos);
    // elegir centroides iniciales random
    for (size_t i = 0; i < k; i++)
    {
        centroids.push_back(puntos->at(rand() % puntos->size()));
    }

    // aqui comienza el for gigante de iteracion del k means
    for (size_t i = 0; i < 2; i++)
    {
        // asignar puntos a cluster segun distancia
        for (std::vector<Dato>::iterator c = std::begin(centroids); c != std::end(centroids); ++c)
        {
            // index del current iterator
            int clusterId = c - std::begin(centroids);
            std::cout << "cluster: " << clusterId << "\n";

            for (std::vector<Dato>::iterator it = std::begin(*puntos); it != ptsEnd; ++it)
            {
                double dist = c->distance(*it);
                std::cout << dist << ", ";

                if (dist < it->min_dist)
                {
                    it->min_dist = dist;
                    it->cluster_id = clusterId;
                }
            }
            std::cout << "\n";
        }
        /* only for debug, maybe put a method to improve the algotrithm convergence by looking at std_dev or smthing around the points
        see: https://www.youtube.com/watch?v=4b5d3muPQmA */
        for (std::vector<Dato>::iterator it = std::begin(*puntos); it != ptsEnd; ++it)
        {
            std::cout << "distancia a cluster: " << it->min_dist << ", cluster: " << it->cluster_id << "\n";
        }

        centroids = newCentroids(puntos, centroids);
    }

    return centroids;
}

int main(int argc, char **argv)
{
    std::cout << "yeah octostuff\n";
    ros::init(argc, argv, "octo_proc");
    ros::NodeHandle nh;

    ros::Publisher marker_pub = nh.advertise<visualization_msgs::MarkerArray>("/kcentroids", 10);

    octomap_msgs::OctomapConstPtr mapa_msg = ros::topic::waitForMessage<octomap_msgs::Octomap>("/octomap_full");
    // octomap_msgs::OctomapConstPtr mapa_msg = ros::topic::waitForMessage<octomap_msgs::Octomap>("/octomap_binary");

    // despues me preocupo por los memory leaks del pointer este cando reciba desde una subscripcion
    octomap::AbstractOcTree *abs_tree = octomap_msgs::msgToMap(*mapa_msg);
    // octomap::AbstractOcTree *abs_tree = octomap_msgs::binaryMsgToMap(*mapa_msg);
    // std::unique_ptr<octomap::AbstractOcTree> abs_tree(octomap_msgs::msgToMap(*mapa_msg));

    std::cout << "resolution " << abs_tree->getResolution() << "\n";
    std::cout << "type " << abs_tree->getTreeType() << "\n";

    std::unique_ptr<octomap::OcTree> tree(dynamic_cast<octomap::OcTree *>(abs_tree));
    std::cout << "resolution " << tree->getResolution() << "\n";
    std::cout << "NumNodes " << tree->calcNumNodes() << "\n";
    std::cout << "Size=Nodes " << tree->size() << "\n";
    std::cout << "NumLeafs " << tree->getNumLeafNodes() << "\n";
    std::cout << "TreeDepth " << tree->getTreeDepth() << "\n";

    // no puedo iterar sobre nodos que no existen?, no, pero puedo checkear alrededor de los que si existem
    // check adjacent cubes to see if this cube is in a frontier or it isnt
    // mandar un raycast
    // contar los hijos de cada nodo, menos de 8 es porque hay unknowns(depende si es free o ocupied?)
    // https://github.com/RobustFieldAutonomyLab/turtlebot_exploration_3d/blob/master/include/exploration.h
    // tree->getTreeDepth()

    // crear los markers
    visualization_msgs::MarkerArray marker_array;
    visualization_msgs::Marker marker;
    // marker.action = visualization_msgs::Marker::DELETEALL;
    // marker_array.markers.push_back(marker);
    // marker_pub.publish((marker_array));

    marker.header.frame_id = "world";
    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.color.r = 0.659;
    marker.color.g = 0.659;
    marker.color.b = 0.659;
    marker.color.a = 1;
    marker.pose.orientation.w = 1;

    int i = 0;
    int depth = 15;
    // int factor = pow(2, 16 - depth);
    float dist = tree->getResolution() * pow(2, 16 - depth);
    marker.scale.x = dist - 0.01;
    marker.scale.y = dist - 0.01;
    marker.scale.z = dist - 0.01;
    std::shared_ptr<std::vector<Dato>> puntos = std::make_shared<std::vector<Dato>>();
    for (octomap::OcTree::leaf_iterator n = tree->begin_leafs(depth), end = tree->end_leafs(); n != end; ++n)
    {
        if (!tree->isNodeOccupied(*n))
        {
            octomap::point3d pt = n.getCoordinate();
            // *2 por la resolucion del tree y el depth por encima del maximo
            // pt.z() = pt.z() - tree->getResolution() * 2;
            pt.x() = pt.x() + dist;
            if (tree->search(pt, depth) == NULL)
            {
                std::cout << "Node center: " << n.getCoordinate() << "\n";
                std::cout << "al lado en x Unknown\n";

                puntos->push_back(pt);
                // puntos->push_back(Dato(n.getCoordinate()));

                marker.ns = "cubos_x";
                marker.header.stamp = ros::Time::now();
                marker.id = i++;
                marker.pose.position.x = n.getCoordinate().x() + dist;
                marker.pose.position.y = n.getCoordinate().y();
                marker.pose.position.z = n.getCoordinate().z();
                marker_array.markers.push_back(marker);

                std::cout << "-------------\n";
            }
        }
    }

    if (puntos->empty())
    {
        std::cout << "no hay nodos sin explorar, finalizando programa\n";
        return 1;
    }

    // funcion que recibe puntos y numero de cluster, retorna vector de centroides de cada cluster
    int k = 2;
    std::vector<Dato> centroids = kmeans(puntos, k);

    // crear los markers
    // visualization_msgs::MarkerArray marker_array;
    // visualization_msgs::Marker marker;
    marker.header.frame_id = "world";
    marker.ns = "centroids";
    marker.type = visualization_msgs::Marker::SPHERE;
    // marker.action = visualization_msgs::Marker::ADD;
    marker.scale.x = 1;
    marker.scale.y = 1.5;
    marker.scale.z = 0.5;
    marker.color.r = 1;
    marker.color.g = 0.796;
    marker.color.b = 0.0;
    // marker.color.a = 1;
    // marker.pose.orientation.w = 1;
    for (std::vector<Dato>::iterator c = std::begin(centroids); c != std::end(centroids); ++c)
    {
        // print los centroides for debuging
        std::cout << c->punto << "\n";

        marker.header.stamp = ros::Time::now();
        marker.id = c - std::begin(centroids);
        marker.pose.position.x = c->punto.x();
        marker.pose.position.y = c->punto.y();
        marker.pose.position.z = c->punto.z();

        marker_array.markers.push_back(marker);
    }
    marker_pub.publish((marker_array));
}