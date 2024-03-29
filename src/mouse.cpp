#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <geometry_msgs/PointStamped.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

class ImageProcessor
{
public:
    ImageProcessor(ros::NodeHandle& nh) : nh_(nh), it_(nh_), clicked_point_(0, 0)
    {
        // 订阅图像主题
        image_sub_ = it_.subscribe("/yolov8_seg/mask", 1, &ImageProcessor::imageCallback, this);

        // 发布点坐标
        point_pub_ = nh_.advertise<geometry_msgs::PointStamped>("/clicked_point", 10);

        // 设置OpenCV的鼠标回调
        cv::namedWindow("Image window");
        cv::setMouseCallback("Image window", ImageProcessor::mouseCallback, this);
    }

    ~ImageProcessor()
    {
        cv::destroyWindow("Image window");
    }

    void imageCallback(const sensor_msgs::ImageConstPtr& msg)
    {
        try
        {
            cv_ptr_ = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
            // ROS_INFO("Image converted successfully. Image size: %d x %d", cv_ptr_->image.cols, cv_ptr_->image.rows);
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }

        if (!cv_ptr_->image.empty())
        {
            // 在图像中绘制点
            // cv::circle(cv_ptr_->image, cv::Point(100,100), 5, CV_RGB(255,0,0), -1);

            // 显示图像
            cv::imshow("Image window", cv_ptr_->image);
            cv::waitKey(30);

            // 发布点坐标
            publishPoint(clicked_point_.x, clicked_point_.y);
        }
        else
        {
            ROS_ERROR("Received an empty image.");
        }
    }

    static void mouseCallback(int event, int x, int y, int flags, void* userdata)
    {
        if (userdata == nullptr)
        {
            ROS_ERROR("Mouse callback received null userdata.");
            return;
        }

        ImageProcessor* processor = reinterpret_cast<ImageProcessor*>(userdata);
        if (event == cv::EVENT_LBUTTONDOWN)
        {
            processor->clicked_point_ = cv::Point(x, y);
        }
    }

    void publishPoint(int x, int y)
    {
        geometry_msgs::PointStamped point_msg;
        point_msg.header.stamp = ros::Time::now();
        point_msg.header.frame_id = "camera_frame"; // 根据你的相机坐标系进行调整
        point_msg.point.x = x;
        point_msg.point.y = y;
        point_msg.point.z = 0; // 2D图像中的z坐标通常为0

        point_pub_.publish(point_msg);
    }

private:
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    ros::Publisher point_pub_;
    cv_bridge::CvImagePtr cv_ptr_;
    cv::Point clicked_point_; // 存储点击的点
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "image_processor");
    ros::NodeHandle nh;

    ImageProcessor processor(nh);

    ros::spin();
    return 0;
}
