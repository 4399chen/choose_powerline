#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <vision_msgs/Detection2DArray.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <visualization_msgs/Marker.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <fstream>
#include <deque>


struct CameraCalibrationData {
    float fx, fy, cx, cy;
    // 可以添加其他标定参数，如畸变系数等
};

class MinRectsProcessor
{
public:

    std::deque<cv::Point3f> pointsBuffer25; // 为point25维护一个缓冲区
    std::deque<cv::Point3f> pointsBuffer75; // 为point75维护一个缓冲区
    const size_t bufferSize = 2; // 缓冲区大小，即平均的点数

    MinRectsProcessor(ros::NodeHandle& nh) : nh_(nh)
    {
        image_transport::ImageTransport it(nh);
        processed_image_pub_ = it.advertise("/processed_depth_image", 1);
        // linePosePublisher = nh_.advertise<geometry_msgs::Pose>("/line_pose", 10);
        linePosePublisher = nh_.advertise<geometry_msgs::PoseStamped>("/line_pose", 10);

        markerPub = nh_.advertise<visualization_msgs::Marker>("/visualization_marker", 10);

        // 从ROS参数服务器获取相机内参
        nh_.param("camera_fx", cameraCalibrationData.fx, 644.7825927734375f);
        nh_.param("camera_fy", cameraCalibrationData.fy, 644.0340576171875f);
        nh_.param("camera_cx", cameraCalibrationData.cx, 636.1322631835938f);
        nh_.param("camera_cy", cameraCalibrationData.cy, 367.4698486328125f);

        // 在构造函数中打开文件
        // std::string fileName = generateFileName();
        // outputFile.open(fileName);
        // if (!outputFile.is_open()) {
        //     ROS_ERROR("Could not open file: %s", fileName.c_str());
        //     // 考虑是否需要退出程序或采取其他错误处理措施
        // }



        depthImage_sub_.subscribe(nh_, "/yolov8_seg/modified_depth_image", 10);
        minRects_sub_.subscribe(nh_, "/yolov8_seg/minRects", 10);

        clicked_point_sub_.subscribe(nh_, "/clicked_point", 10);
        sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(SyncPolicy(10), minRects_sub_, depthImage_sub_, clicked_point_sub_);
        sync_->registerCallback(boost::bind(&MinRectsProcessor::callback, this, _1, _2, _3));

        cv::namedWindow("MinRects Visualization", cv::WINDOW_NORMAL);
    }

    std::string generateFileName()
    {
        std::time_t t = std::time(nullptr);
        char timeStr[100];
        std::strftime(timeStr, sizeof(timeStr), "%Y%m%d-%H%M%S", std::localtime(&t));

        std::string filePath = "/home/work/";
        std::string fileName = "RealWorldPoints-" + std::string(timeStr) + ".txt";

        return filePath + fileName; // 返回包含路径的文件名
    }

    cv::Mat processDepthImage(const sensor_msgs::Image::ConstPtr& msg, const vision_msgs::BoundingBox2D& selectedBox)
    {
        if (!msg) {
            ROS_ERROR("Received null image message");
            return cv::Mat();
        }

        // 将sensor_msgs::Image转换为cv::Mat
        cv::Mat depthImage = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1)->image;

        if (depthImage.empty()) {
            ROS_ERROR("Converted depth image is empty");
            return cv::Mat();
        }

        // 创建一个与深度图同样大小的掩码，初始化为0（黑色）
        cv::Mat mask = cv::Mat::zeros(depthImage.rows, depthImage.cols, CV_16UC1);

        // 使用selectedBox中的信息绘制旋转的矩形
        cv::Point2f rect_center(selectedBox.center.x, selectedBox.center.y);
        cv::Size2f rect_size(selectedBox.size_x, selectedBox.size_y);
        float rect_angle = selectedBox.center.theta; // 这里需要转换角度，如果需要
        cv::RotatedRect rotatedRect(rect_center, rect_size, rect_angle);
        cv::Point2f vertices[4]; 
        rotatedRect.points(vertices);
        std::vector<cv::Point> points;
        for (int i = 0; i < 4; i++) {
            points.push_back(vertices[i]);
        }
        cv::fillConvexPoly(mask, points, cv::Scalar(1));

        // 将深度图与掩码相乘
        depthImage = depthImage.mul(mask);

        // 返回处理后的深度图
        return depthImage;
    }

    vision_msgs::BoundingBox2D processMinRects(const vision_msgs::Detection2DArray::ConstPtr& msg, const cv::Size& imageSize, const cv::Point2f& targetPoint)
    {
        // 初始化变量
        vision_msgs::BoundingBox2D selectedBox;
        float minDistance = std::numeric_limits<float>::max();

        for (const auto& det : msg->detections) {
            cv::Point2f detCenter(det.bbox.center.x, det.bbox.center.y);

            float dx = detCenter.x - targetPoint.x;
            float dy = detCenter.y - targetPoint.y;
            float distance = dx * dx + dy * dy;

            if (distance < minDistance) {
                minDistance = distance;
                selectedBox = det.bbox;
            }
        }

        return selectedBox;
    }

    cv::Point3f calculatePoint(const vision_msgs::BoundingBox2D& box, float percentage) {
        // 计算旋转矩形的四个顶点
        cv::RotatedRect rotatedRect(cv::Point2f(box.center.x, box.center.y),
                                    cv::Size2f(box.size_x, box.size_y),
                                    box.center.theta);
        cv::Point2f vertices[4];
        rotatedRect.points(vertices);

        // 计算两对对角顶点的中点
        cv::Point2f midPoint1 = (vertices[0] + vertices[1]) * 0.5;
        cv::Point2f midPoint2 = (vertices[2] + vertices[3]) * 0.5;
        cv::Point2f midPoint3 = (vertices[1] + vertices[2]) * 0.5;
        cv::Point2f midPoint4 = (vertices[3] + vertices[0]) * 0.5;

        // 比较两对中点，找到代表最长边的一对
        float dist1 = cv::norm(midPoint1 - midPoint2);
        float dist2 = cv::norm(midPoint3 - midPoint4);
        cv::Point2f chosenMidPoint1, chosenMidPoint2;

        if (dist1 > dist2) {
            chosenMidPoint1 = midPoint1;
            chosenMidPoint2 = midPoint2;
        } else {
            chosenMidPoint1 = midPoint3;
            chosenMidPoint2 = midPoint4;
        }

        // 线性插值以找到百分比位置的点
        cv::Point2f pointOnLine = chosenMidPoint1 + percentage * (chosenMidPoint2 - chosenMidPoint1);

        // 返回3D点（假设z坐标为0）
        return cv::Point3f(pointOnLine.x, pointOnLine.y, 0.0f);
    }

    float calculateAverageDepth(const cv::Mat& depthMat, const cv::Point3f& point) {
        if (depthMat.empty()) {
            ROS_ERROR("Depth image is empty");
            return 0.0f;
        }

        // 定义ROI的尺寸（例如5x5）
        int roiSize = 50;
        int halfSize = roiSize / 2;
        cv::Rect roi(point.x - halfSize, point.y - halfSize, roiSize, roiSize);

        // 确保ROI在图像内
        roi = roi & cv::Rect(0, 0, depthMat.cols, depthMat.rows);

        // 提取ROI中的深度值
        cv::Mat roiDepth = depthMat(roi);
        double sumDepth = 0.0;
        int countValid = 0;
        for (int y = 0; y < roiDepth.rows; ++y) {
            for (int x = 0; x < roiDepth.cols; ++x) {
                ushort depth = roiDepth.at<ushort>(y, x);
                if (depth > 0) { // 忽略深度为0的像素
                    sumDepth += depth;
                    countValid++;
                }
            }
        }

        if (countValid == 0) return 0.0f; // 避免除以0

        return static_cast<float>(sumDepth / countValid);
    }

    cv::Point3f calculateRealWorldCoordinates(const cv::Point3f& point, float depth, const CameraCalibrationData& calibrationData) {
        // 解析相机内参
        float fx = calibrationData.fx;
        float fy = calibrationData.fy;
        float cx = calibrationData.cx;
        float cy = calibrationData.cy;

        // 将像素坐标转换为真实世界坐标
        float x = (point.x - cx) * depth / fx;
        float y = (point.y - cy) * depth / fy;
        float z = depth;

        return cv::Point3f(x, y, z);
    }

    geometry_msgs::Pose fitLineIn3D(const cv::Point3f& point1, const cv::Point3f& point2) {
        geometry_msgs::Pose linePose;

        // 设置直线上的一个点
        linePose.position.x = point1.x;
        linePose.position.y = point1.y;
        linePose.position.z = point1.z;

        // 计算方向向量
        cv::Point3f direction = point2 - point1;

        // 将方向向量转换为四元数
        tf2::Vector3 axis(direction.x, direction.y, direction.z);
        axis.normalize();
        tf2::Quaternion q;
        q.setRotation(axis, 1.0); // 可能需要调整以匹配您的坐标系统
        linePose.orientation.x = q.x();
        linePose.orientation.y = q.y();
        linePose.orientation.z = q.z();
        linePose.orientation.w = q.w();

        return linePose;
    }

    void publishLineMarker(const cv::Point3f& point1, const cv::Point3f& point2, ros::Publisher& markerPub, const ros::Time& timeStamp) {
        visualization_msgs::Marker lineMarker;
        lineMarker.header.frame_id = "map";
        // lineMarker.header.stamp = ros::Time::now();
        lineMarker.header.stamp = timeStamp; // 使用传递的时间戳

        lineMarker.ns = "line";
        lineMarker.id = 0;
        lineMarker.type = visualization_msgs::Marker::LINE_STRIP;
        lineMarker.action = visualization_msgs::Marker::ADD;

        // 设置 Marker 的比例，颜色等属性
        lineMarker.scale.x = 0.05; // 线宽
        lineMarker.color.r = 1.0;
        lineMarker.color.g = 0.0;
        lineMarker.color.b = 0.0;
        lineMarker.color.a = 1.0;

        // 设置直线的两个端点
        geometry_msgs::Point start, end;

        start.x = point1.x / 1000.0; // 转换为米
        start.y = point1.y / 1000.0;
        start.z = point1.z / 1000.0;
        end.x   = point2.x / 1000.0;
        end.y   = point2.y / 1000.0;
        end.z   = point2.z / 1000.0;

        lineMarker.points.push_back(start);
        lineMarker.points.push_back(end);

        markerPub.publish(lineMarker);
    }

    cv::Point3f calculateMovingAverage(const cv::Point3f& newPoint, std::deque<cv::Point3f>& buffer) {
        buffer.push_back(newPoint);

        if (buffer.size() > bufferSize) {
            buffer.pop_front();
        }

        cv::Point3f sum = {0, 0, 0};
        for (const auto& point : buffer) {
            sum.x += point.x;
            sum.y += point.y;
            sum.z += point.z;
        }

        return cv::Point3f{sum.x / buffer.size(), sum.y / buffer.size(), sum.z / buffer.size()};
    }

    void callback(const vision_msgs::Detection2DArray::ConstPtr& minRectsMsg, const sensor_msgs::Image::ConstPtr& depthImageMsg, const geometry_msgs::PointStamped::ConstPtr& clickedPointMsg)
    {
        // cameraCalibrationData.fx = 644.7825927734375;
        // cameraCalibrationData.fy = 644.0340576171875;
        // cameraCalibrationData.cx = 636.1322631835938;
        // cameraCalibrationData.cy = 367.4698486328125;

        // 获取深度图像的尺寸
        cv::Size imageSize(depthImageMsg->width, depthImageMsg->height);

        // 使用clickedPointMsg中的点作为目标点
        cv::Point2f targetPoint(clickedPointMsg->point.x, clickedPointMsg->point.y);
        vision_msgs::BoundingBox2D selectedBox = processMinRects(minRectsMsg, imageSize, targetPoint);
        
        // 根据选中的矩形过滤深度图
        cv::Mat processedImage = processDepthImage(depthImageMsg, selectedBox);

        // 将cv::Mat转换回sensor_msgs::Image
        cv_bridge::CvImage cv_image;
        cv_image.image = processedImage;
        cv_image.encoding = "16UC1"; // 或者根据processedImage的类型
        sensor_msgs::Image ros_image;
        cv_image.toImageMsg(ros_image);

        // 发布图像
        processed_image_pub_.publish(ros_image);

        // 取selectedBox较长的中线上25%和75%的点
        cv::Point3f point25 = calculatePoint(selectedBox, 0.15);
        cv::Point3f point75 = calculatePoint(selectedBox, 0.85);

        // 在深度图对两个点周围2500个像素取平均（剔除无效值），得到两个点的深度
        float depth25 = calculateAverageDepth(processedImage, point25);
        float depth75 = calculateAverageDepth(processedImage, point75);

        // 根据两个点的xy和相机标定数据，算出真实度量的xy
        cv::Point3f realWorldPoint25 = calculateRealWorldCoordinates(point25, depth25, cameraCalibrationData);
        cv::Point3f realWorldPoint75 = calculateRealWorldCoordinates(point75, depth75, cameraCalibrationData);

        // 平滑
        cv::Point3f smoothedPoint25 = calculateMovingAverage(realWorldPoint25, pointsBuffer25);
        cv::Point3f smoothedPoint75 = calculateMovingAverage(realWorldPoint75, pointsBuffer75);

        // 根据两个点的真实度量xyz，拟合出空间中的一根直线，存储格式为geometry_msgs/Pose
        // 将geometry_msgs/Pose格式的直线作为ROS topic发送出去
        geometry_msgs::PoseStamped linePoseStamped;
        linePoseStamped.header.stamp = depthImageMsg->header.stamp;
        linePoseStamped.header.frame_id = "map";  // 根据需要替换为适当的帧 ID
        linePoseStamped.pose = fitLineIn3D(smoothedPoint25, smoothedPoint75);
        // linePoseStamped.pose = fitLineIn3D(realWorldPoint25, realWorldPoint75);
        linePosePublisher.publish(linePoseStamped);

        // 将数据写入文件
        // outputFile << "Timestamp: " << ros::Time::now() << ", "
        //         << "RealWorldPoint25: (" << realWorldPoint25.x << ", " << realWorldPoint25.y << ", " << realWorldPoint25.z << "), "
        //         << "RealWorldPoint75: (" << realWorldPoint75.x << ", " << realWorldPoint75.y << ", " << realWorldPoint75.z << ")\n";

        // 使用linePose的数据，构建一个visualization_msgs/Marker，发布为rostopic，使用RVIZ订阅查看，header.frame_id = "map"
        // publishLineMarker(realWorldPoint25, realWorldPoint75, markerPub, depthImageMsg->header.stamp);
        publishLineMarker(smoothedPoint25, smoothedPoint75, markerPub, depthImageMsg->header.stamp);

    }

    ~MinRectsProcessor()
    {
        cv::destroyAllWindows();

        // if (outputFile.is_open()) {
        //     outputFile.close();
        // }
    }

private:
    ros::NodeHandle& nh_;

    message_filters::Subscriber<vision_msgs::Detection2DArray> minRects_sub_;
    message_filters::Subscriber<sensor_msgs::Image> depthImage_sub_;
    message_filters::Subscriber<geometry_msgs::PointStamped> clicked_point_sub_;

    typedef message_filters::sync_policies::ApproximateTime<vision_msgs::Detection2DArray, sensor_msgs::Image, geometry_msgs::PointStamped> SyncPolicy;
    // typedef message_filters::sync_policies::ApproximateTime<vision_msgs::Detection2DArray, sensor_msgs::Image> SyncPolicy;
    std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;

    image_transport::Publisher processed_image_pub_;
    ros::Publisher linePosePublisher;
    ros::Publisher markerPub;
    CameraCalibrationData cameraCalibrationData;

    std::ofstream outputFile;
    // std::string generateFileName();
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "min_rects_processor_node");
    ros::NodeHandle nh;

    MinRectsProcessor processor(nh);

    ros::spin();

    return 0;
}
