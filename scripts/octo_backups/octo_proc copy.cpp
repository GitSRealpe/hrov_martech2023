#include <ros/ros.h>
#include <octomap/octomap.h>
#include <octomap/OcTree.h>
// #include <octomap_ros/conversions.h>
#include <octomap_msgs/conversions.h>

// #include <octomap/AbstractOcTree.h>

int main(int argc, char **argv)
{
    std::cout << "yeah octostuff\n";
    ros::init(argc, argv, "octo_proc");
    ros::NodeHandle nh;

    // octomap_msgs::OctomapConstPtr mapa;
    octomap_msgs::OctomapConstPtr mapa_msg = ros::topic::waitForMessage<octomap_msgs::Octomap>("/octomap_full");

    octomap::AbstractOcTree *abs_tree = octomap_msgs::msgToMap(*mapa_msg);
    // std::unique_ptr<octomap::AbstractOcTree> abs_tree(octomap_msgs::msgToMap(*mapa_msg));

    std::cout << "resolution " << abs_tree->getResolution() << "\n";
    std::cout << "type " << abs_tree->getTreeType() << "\n";

    std::unique_ptr<octomap::OcTree> tree(dynamic_cast<octomap::OcTree *>(abs_tree));
    std::cout << "resolution " << tree->getResolution() << "\n";
    std::cout << "NumNodes " << tree->calcNumNodes() << "\n";
    std::cout << "Size=Nodes " << tree->size() << "\n";
    std::cout << "NumLeafs " << tree->getNumLeafNodes() << "\n";
    std::cout << "TreeDepth " << tree->getTreeDepth() << "\n";

    // std::cout << "root pos " << tree->getRoot() << "\n";

    // no puedo iterar sobre nodos que no existen?, no, pero puedo checkear alrededor de los que si existem
    // check adjacent cubes to see if this cube is in a frontier or it isnt
    // mandar un raycast
    // contar los hijos de cada nodo, menos de 8 es porque hay unknowns
    // https://github.com/RobustFieldAutonomyLab/turtlebot_exploration_3d/blob/master/include/exploration.h
    // tree->getTreeDepth()
    for (octomap::OcTree::leaf_iterator n = tree->begin_leafs(14), end = tree->end_leafs(); n != end; ++n)
    {

        // std::cout << "Node size: " << n.getSize() << std::endl;
        // std::cout << "Node value: " << n->getValue() << std::endl;

        if (!tree->isNodeOccupied(*n) && tree->nodeHasChildren(&*n))
        {
            std::cout << "Node center: " << n.getCoordinate() << std::endl;
            for (size_t i = 0; i < 8; i++)
            {
                if (!tree->nodeChildExists(&*n, i))
                {
                    std::cout << "no tiene el " << i << "\n";
                }
            }
            std::cout << "-------------\n";
        }

        // if (!tree->isNodeOccupied(*n))
        // {
        //     std::cout << n.getX() << " " << n.getY() << " " << n.getZ() << "\n";
        // }
    }

    // delete abs_tree;
}