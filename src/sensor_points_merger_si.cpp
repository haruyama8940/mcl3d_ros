#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <pcl/point_cloud.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <mcl3d_ros/Pose.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

class SensorPointsMerger :public rclcpp::Node {

private:
    std::string baseFrame_;
    std::vector<std::string> sensorFrames_, sensorTopicNames_;
    std::vector<rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr> pointsSubs_;
    std::vector<mcl3d::Pose> displacements_;
    std::vector<bool> gotPoints_;

    std::string mergedPointsName_, mergedPointsFrame_;
    //publisher
    sensor_msgs::msg::PointCloud mergedPoints_;
    // tf2_ros::TransformListener tfLisner_;
    tf2::Transform tfLisner_;
    

public:
    SensorPointsMerger(void)
        : Node("sensor_points_merger"),
        baseFrame_("base_link"),
        sensorTopicNames_({"/cloud", "/cloud"}),
        sensorFrames_({"hokuyo3d_front", "hokuyo3d_rear"}),
        gotPoints_({false, false}),
        mergedPointsName_("/velodyne_points"),
        mergedPointsFrame_("velodyne")
    {
        this->declare_parameter("base_frame", baseFrame_);
        this->declare_parameter("sensor_frames", sensorFrames_);
        this->declare_parameter("sensor_topic_names", sensorTopicNames_);
        this->declare_parameter("merged_points_name", mergedPointsName_);
        this->declare_parameter("merged_points_frame", mergedPointsFrame_);

        this->get_parameter("base_frame", baseFrame_);
        this->get_parameter("sensor_frames", sensorFrames_);
        this->get_parameter("sensor_topic_names", sensorTopicNames_);
        this->get_parameter("merged_points_name", mergedPointsName_);
        this->get_parameter("merged_points_frame", mergedPointsFrame_);

        tf_buffer_ =std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ =std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        for (int i = 0; i < (int)sensorFrames_.size(); ++i) {
            // geometry_msgs::msg::TransformStamped trans;
            tf2::Transform trans;
            int tfFailedCnt = 0;
            rclcpp::Rate loopRate(10.0);
            rclcpp::Duration d = rclcpp::Duration::from_seconds(1.0);
            while (rclcpp::ok())
                try
                {
                    rclcpp::Time now = rclcpp::Clock(RCL_ROS_TIME).now();
                    // tf_buffer_->waitForTransform(
                    //     baseFrame_,sensorFrames_[i],now,d.seconds());
                    tf_buffer_->lookupTransform(
                         baseFrame_, sensorFrames_[i],now);
                        //  tf2::TimePointZero);
                }
                catch(tf2::TransformException ex){
                    tfFailedCnt++;
                    if (tfFailedCnt >= 100) {
                       RCLCPP_ERROR(get_logger(), "Cannot get the relative pose from the base link to the laser from the tf tree."
                                    " Did you set the static transform publisher between %s to %s?",
                                    baseFrame_.c_str(), sensorFrames_[i].c_str());
                        exit(1);
                }
            loopRate.sleep();
        }
    
    tf2::Quaternion quat(trans.getRotation().x(),trans.getRotation().y(),trans.getRotation().z(),trans.getRotation().w());
    double roll, pitch, yaw;
    tf2::Matrix3x3 risMat(quat);
    mcl3d::Pose poseTrans(trans.getOrigin().x(), trans.getOrigin().y(), trans.getOrigin().z(), roll, pitch, yaw);
    displacements_.push_back(poseTrans);

    }
    for (size_t i = 0; i < (int)sensorTopicNames_.size(); ++i) {
            auto sub = this->create_subscription<sensor_msgs::msg::PointCloud2>(
                sensorTopicNames_[i], 1,std::bind(&SensorPointsMerger::PointCB, this, std::placeholders::_1);
            pointsSubs_.push_back(sub);
        }
    pointsPub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(mergedPointsName_, 5);
    
    mergedPoints_.points.clear();
    printf("Initialization has been done.\n");
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    }
    void pointsCB(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        // std::string topicName = event.getConnectionHeader().at("topic");
        // std::string frame = event.getMessage()->header.frame_id;
        // printf("topicName = %s, frame = %s\n", topicName.c_str(), frame.c_str());
        mcl3d::Pose poseTrans;
        for (int i = 0; i < (int)sensorTopicNames_.size(); ++i) {
             if (gotPoints_[i])
                    return;
                // add obtained points
                poseTrans = displacements_[i];
                gotPoints_[i] = true;
                break;
        }

        double cr = cos(poseTrans.getRoll());
        double sr = sin(poseTrans.getRoll());
        double cp = cos(poseTrans.getPitch());
        double sp = sin(poseTrans.getPitch());
        double cy = cos(poseTrans.getYaw());
        double sy = sin(poseTrans.getYaw());

        double m11 = cy * cp;
        double m12 = cy * sp * sr - sy * cr;
        double m13 = sy * sr + cy * sp * cr;
        double m21 = sy * cp;
        double m22 = cy * cr + sy * sp * sr;
        double m23 = sy * sp * cr - cy * sr;
        double m31 = -sp;
        double m32 = cp * sr;
        double m33 = cp * cr;
        pcl::PointCloud<pcl::PointXYZ> points;
        pcl::fromROSMsg(*msg,points);

        for (int i = 0; i < (int)points.size(); ++i) {
            float x = points.points[i].x;
            float y = points.points[i].y;
            float z = points.points[i].z;
            geometry_msgs::msg::Point32 p;
            p.x = x * m11 + y * m12 + z * m13 + poseTrans.getX();
            p.y = x * m21 + y * m22 + z * m23 + poseTrans.getY();
            p.z = x * m31 + y * m32 + z * m33 + poseTrans.getZ();
            mergedPoints_.points.push_back(p);
        }

        for (int i = 0; i < (int)gotPoints_.size(); ++i) {
            if (!gotPoints_[i])
                return;
        }

        sensor_msgs::msg::PointCloud2 sensorPoints;
        sensor_msgs::msg::
};


