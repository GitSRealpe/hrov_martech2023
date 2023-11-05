#include <ros/ros.h>

#include <image_transport/image_transport.h>
#include <image_geometry/pinhole_camera_model.h>
// #include <tf/transform_listener.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/aruco.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d.hpp>

#include <message_filters/subscriber.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>

// #include <tf2_eigen/tf2_eigen.h>
#include <tf2/convert.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <geometry_msgs/Pose.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

static const std::string OPENCV_WINDOW = "Finestra";

struct CamData
{
    cv::Mat K;
    std::vector<double> D;
};

class ImageConverter
{
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    ros::Subscriber cam_info_sub_;
    image_transport::Publisher image_pub_;

    CamData camData;

public:
    ImageConverter() : it_(nh_)
    {
        image_sub_ = it_.subscribe("/girona1000/arm_camara/image_color", 1, &ImageConverter::imageCb, this);
        cam_info_sub_ = nh_.subscribe("/girona1000/arm_camara/camera_info", 1, &ImageConverter::cameraInfoCB, this);

        image_pub_ = it_.advertise("/image_converter/output_video", 1);

        cv::namedWindow(OPENCV_WINDOW, cv::WindowFlags::WINDOW_AUTOSIZE);
    }

    ~ImageConverter()
    {
        cv::destroyWindow(OPENCV_WINDOW);
    }

    void tf_pub(cv::Vec3d pos, cv::Vec3d rot, int id)
    {

        static tf2_ros::TransformBroadcaster br;
        geometry_msgs::TransformStamped transformStamped;

        transformStamped.header.stamp = ros::Time::now();
        transformStamped.header.frame_id = "girona1000/bravo/camera";
        transformStamped.child_frame_id = std::to_string(id);
        transformStamped.transform.translation.x = pos[0];
        transformStamped.transform.translation.y = pos[1];
        transformStamped.transform.translation.z = pos[2];

        cv::Mat rotM(3, 3, CV_64FC1);
        cv::Rodrigues(rot, rotM);
        tf2::Matrix3x3 tf_rotM(rotM.at<double>(0, 0), rotM.at<double>(0, 1), rotM.at<double>(0, 2),
                               rotM.at<double>(1, 0), rotM.at<double>(1, 1), rotM.at<double>(1, 2),
                               rotM.at<double>(2, 0), rotM.at<double>(2, 1), rotM.at<double>(2, 2));
        tf2::Quaternion tf_quat;
        tf_rotM.getRotation(tf_quat);
        tf2::convert(tf_quat, transformStamped.transform.rotation);

        br.sendTransform(transformStamped);
    }

    void cameraInfoCB(const sensor_msgs::CameraInfo &caminfo)
    {
        // Get calibration
        ROS_INFO("Reading calibration from camera_info topic");
        cv::Mat cameraMatrix(3, 3, CV_64F);
        cv::Mat distortionCoeff(5, 1, CV_64F);
        // Intrinsics
        cv::Mat K = (cv::Mat_<double>(3, 3) << caminfo.K[0], caminfo.K[1], caminfo.K[2],
                     caminfo.K[3], caminfo.K[4], caminfo.K[5],
                     caminfo.K[6], caminfo.K[7], caminfo.K[8]);
        camData.K = K;
        // std::cout << "K = " << std::endl
        //           << " " << K << std::endl
        //           << std::endl;
        // Distortion
        camData.D = caminfo.D;
        // std::cout << caminfo.D.data() << "\n";

        cam_info_sub_.shutdown();
        ROS_INFO("Done calibration");
    }

    void imageCb(const sensor_msgs::ImageConstPtr &msg)
    {
        cv_bridge::CvImagePtr cv_ptr;
        try
        {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
        }
        catch (cv_bridge::Exception &e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }

        cv::Mat image_rect = cv_ptr->image;
        // cv::undistort(cv_ptr->image, image_rect, camData.K, camData.D);
        // cv::equalizeHist(image_rect, image_rect);

        std::vector<int> markerIds;
        std::vector<std::vector<cv::Point2f>> markerCorners, rejectedCandidates;
        cv::Ptr<cv::aruco::DetectorParameters> parameters = cv::aruco::DetectorParameters::create();
        cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_APRILTAG_36h11);
        cv::aruco::detectMarkers(image_rect, dictionary, markerCorners, markerIds, parameters, rejectedCandidates);
        cv::aruco::drawDetectedMarkers(image_rect, markerCorners, markerIds);

        std::vector<cv::Vec3d> rvecs, tvecs;
        cv::aruco::estimatePoseSingleMarkers(markerCorners, 0.12, camData.K, camData.D, rvecs, tvecs);

        cv::cvtColor(image_rect, image_rect, cv::COLOR_GRAY2BGR);

        for (int i = 0; i < rvecs.size(); ++i)
        {
            auto rvec = rvecs[i];
            auto tvec = tvecs[i];
            cv::aruco::drawAxis(image_rect, camData.K, camData.D, rvec, tvec, 0.06);
            std::cout << "position: in cam frame: \n";
            std::cout << tvecs[i] << "\n";
            std::cout << "orientation in cam frame: \n";
            std::cout << rvecs[i] << "\n";
            tf_pub(tvecs[i], rvecs[i], markerIds[i]);
        }

        cv_bridge::CvImage out_msg;
        out_msg.header = cv_ptr->header;     // Same timestamp and tf frame as input image
        out_msg.encoding = cv_ptr->encoding; // Or whatever
        out_msg.image = image_rect;          // Your cv::Mat
        // Output modified video stream
        image_pub_.publish(out_msg.toImageMsg());

        cv::arrowedLine(image_rect, {int(image_rect.cols / 2), int(image_rect.rows / 2)}, {image_rect.cols, int(image_rect.rows / 2)}, CV_RGB(255, 0, 0), 2);

        cv::arrowedLine(image_rect, {int(image_rect.cols / 2), int(image_rect.rows / 2)}, {image_rect.cols / 2, int(image_rect.rows)}, CV_RGB(0, 255, 0), 2);

        // Update GUI Window
        cv::imshow(OPENCV_WINDOW, image_rect);
        cv::waitKey(3);
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "image_converter");
    ImageConverter ic;
    ros::spin();
    return 0;
}