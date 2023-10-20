#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <image_geometry/pinhole_camera_model.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/image_encodings.h>

#include <opencv2/aruco.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>

#include <sensor_msgs/CameraInfo.h>

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
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        }
        catch (cv_bridge::Exception &e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }

        cv::Mat image_rect = cv_ptr->image;
        // cv::undistort(cv_ptr->image, image_rect, camData.K, camData.D);

        std::vector<int> markerIds;
        std::vector<std::vector<cv::Point2f>> markerCorners, rejectedCandidates;
        cv::Ptr<cv::aruco::DetectorParameters> parameters = cv::aruco::DetectorParameters::create();
        cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_APRILTAG_36h11);
        cv::aruco::detectMarkers(image_rect, dictionary, markerCorners, markerIds, parameters, rejectedCandidates);
        cv::aruco::drawDetectedMarkers(image_rect, markerCorners, markerIds);

        std::vector<cv::Vec3d> rvecs, tvecs;
        cv::aruco::estimatePoseSingleMarkers(markerCorners, 0.05, camData.K, camData.D, rvecs, tvecs);

        for (int i = 0; i < rvecs.size(); ++i)
        {
            auto rvec = rvecs[i];
            auto tvec = tvecs[i];
            cv::aruco::drawAxis(image_rect, camData.K, camData.D, rvec, tvec, 0.1);
            std::cout << tvecs[i] << "\n";
        }

        cv_bridge::CvImage out_msg;
        out_msg.header = cv_ptr->header;     // Same timestamp and tf frame as input image
        out_msg.encoding = cv_ptr->encoding; // Or whatever
        out_msg.image = image_rect;          // Your cv::Mat
        // Output modified video stream
        image_pub_.publish(out_msg.toImageMsg());

        // image_rect.rows=heigth
        // image_rect.cols=width
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