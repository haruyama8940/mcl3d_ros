#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
// #include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.hpp>
#include <mcl3d_ros/MCL.hpp>
#include <mcl3d_ros/IMU.hpp>
#include <chrono>

class MCLNode : public rclcpp::Node {
    private:
        std::string mapYamlFile_;
        //subscriber kaku
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sensorPointsSub_;
        rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imuSub_;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odomSub_; 
        rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initialPoseSub_;
        
        std::string mapFrame_, odomFrame_, 
                    baseLinkFrame_, laserFrame_, optPoseFrame_;
        //publiser kaku
        rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr particlesPub_;
        rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr optParticlesPub_;
        // rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr particlesPub_;
        rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr posePub_;
        rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr optPosePub_;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud>::SharedPtr alignedPointsOptPub_;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud>::SharedPtr mapPointsPub_;


        mcl3d::MCL mcl_;
        mcl3d::Pose initialPose_, initialNoise_;
        mcl3d::IMU imu_;

        rclcpp::Time mclPoseStamp_ = rclcpp::Clock(RCL_ROS_TIME).now();
        double transformTolerance_;
        // tf2_ros::TransformBroadcaster tfBroadcaster_;
        //  tf2_ros::TransformListener tfListener_;
        std::unique_ptr<tf2_ros::TransformBroadcaster> tfBroadcaster_;
        std::shared_ptr<tf2_ros::TransformListener> tfListener_{nullptr};
        std::unique_ptr<tf2_ros::Buffer> tfBuffer_;
        bool broadcastTF_, useOdomTF_;

        std::string logFile_;
        bool writeLog_;
    public:
        MCLNode(void):Node("mcl3d")
        // MCLNode(void):
            // mapFrame_("map"),
            // odomFrame_("odom"),
            // baseLinkFrame_("base_link"),
            // laserFrame_("laser"),
            // optPoseFrame_("opt_pose"),
            // transformTolerance_(0.0),
            // broadcastTF_(true),
            // useOdomTF_(false),
            // logFile_("/tmp/als_ros_3d_log.txt"),
            // writeLog_(false)
        {
            //set subscribers
            std::string sensorPointsName = "/velodyne_points", imuName = "/imu/data", odomName = "/odom";
            bool useIMU = false, useOdom = false, useInitialPoseCB = true;
            this->declare_parameter("sensor_points_name",sensorPointsName);
            this->get_parameter("sensor_points_name",sensorPointsName);
            this->declare_parameter("imu_name",imuName);
            this->get_parameter("imu_name",imuName);
            this->declare_parameter("odom_name",odomName);
            this->get_parameter("odom_name",odomName);
            this->declare_parameter("use_imu",useIMU);
            this->get_parameter("use_imu",useIMU);
            this->declare_parameter("use_odom",useOdom);
            this->get_parameter("use_odom",useOdom);
            this->declare_parameter("use_initial_pose_cb",useInitialPoseCB);
            this->get_parameter("use_initial_pose_cb",useInitialPoseCB);
            this->declare_parameter("use_odom_tf",useOdomTF_);
            this->get_parameter("use_odom_tf",useOdomTF_);
            sensorPointsSub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(sensorPointsName,1,
                                                        std::bind(&MCLNode::sensorPointCB, this, std::placeholders::_1));
            if (useIMU)
                imuSub_ = this -> create_subscription<sensor_msgs::msg::Imu>(imuName,1, 
                                                    std::bind(&MCLNode::imuCB, this, std::placeholders::_1));
            if (useOdom || useOdomTF_)
                odomSub_ = this -> create_subscription<nav_msgs::msg::Odometry>(odomName,1,
                                                    std::bind(&MCLNode::odomCB, this, std::placeholders::_1));
            if (useInitialPoseCB)
                initialPoseSub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>
                                ("/initialpose",1,std::bind(&MCLNode::initialPoseCB, this, std::placeholders::_1));

            //set publishers
            std::string particlesName = "/particles", optParticlesName = "/optimized_particles";
            std::string poseName = "/mcl_pose", optPoseName = "/opt_pose";
            std::string alignedPointsOptName = "/aligned_points_opt";
            std::string mapPointsName = "/df_map_points";
            this->declare_parameter("particles_name",particlesName);
            this->get_parameter("particles_name",particlesName);
            this->declare_parameter("opt_particles_name",optParticlesName);
            this->get_parameter("opt_particles_name",optParticlesName);
            this->declare_parameter("pose_name",poseName);
            this->get_parameter("pose_name",poseName);
            this->declare_parameter("opt_pose_name",optPoseName);
            this->get_parameter("opt_pose_name",optPoseName);
            this->declare_parameter("aligned_points_opt_name",alignedPointsOptName);
            this->get_parameter("aligned_points_opt_name",alignedPointsOptName);
            this->declare_parameter("map_points_name", mapPointsName);
            this->get_parameter("map_points_name", mapPointsName);
            //pub
            particlesPub_ = this->create_publisher<geometry_msgs::msg::PoseArray>(particlesName,1);
            optParticlesPub_ = this->create_publisher<geometry_msgs::msg::PoseArray>(optParticlesName,1);
            posePub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(poseName,1);
            optPosePub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(optPoseName,1);
            alignedPointsOptPub_= this->create_publisher<sensor_msgs::msg::PointCloud>(alignedPointsOptName,1);
            mapPointsPub_ = this->create_publisher<sensor_msgs::msg::PointCloud>(mapPointsName,1);

            // read tf frame names
            tfBroadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
            tfBuffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
            tfListener_ = std::make_shared<tf2_ros::TransformListener>(*tfBuffer_);
            this->declare_parameter("map_frame",mapFrame_);
            this->get_parameter("map_frame",mapFrame_);
            this->declare_parameter("odom_frame",odomFrame_);
            this->get_parameter("odom_frame",odomFrame_);
            this->declare_parameter("base_link_frame",baseLinkFrame_);
            this->get_parameter("base_link_frame",baseLinkFrame_);
            this->declare_parameter("laser_frame",laserFrame_);
            this->get_parameter("laser_frame",laserFrame_);
            this->declare_parameter("opt_pose_frame",optPoseFrame_);
            this->get_parameter("opt_pose_frame",optPoseFrame_);

            // set localization mode
            int localizationMode = 0, measurementModelType = 3;
            this->declare_parameter("localization_mode",localizationMode);
            this->get_parameter("localization_mode",localizationMode);
            this->declare_parameter("measurement_model_type",measurementModelType);
            this->get_parameter("measurement_model_type",measurementModelType);
            mcl_.setLocalizationMode(localizationMode);
            mcl_.setMeasurementModelType(measurementModelType);

            // set number of particles
            int particleNum = 500;
            this->declare_parameter("particle_num",particleNum);
            this->get_parameter("particle_num",particleNum);
            mcl_.setParticleNum(particleNum);

             // set number of points used for likelihood calculation
            int sensorPointsNum = 200;
            this->declare_parameter("setSensorPointsNum",sensorPointsNum);
            this->get_parameter("setSensorPointsNum",sensorPointsNum);
            mcl_.setSensorPointsNum(sensorPointsNum);

            // set voxel leaf size
            double voxelLeafSize = 1.0; 
            this->declare_parameter("voxel_leaf_size",voxelLeafSize);
            this->get_parameter("voxel_leaf_size",voxelLeafSize);
            mcl_.setVoxelLeafSize(voxelLeafSize);

            // set pose related parameters
            std::vector<double> initialPose = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
            std::vector<double> initialNoise = {0.1, 0.1, 0.1, 0.05, 0.05, 0.05};
            std::vector<double> baseLink2Laser = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
            this->declare_parameter("initial_pose",initialPose);
            this->get_parameter("initial_pose",initialPose);
            this->declare_parameter("initial_noise",initialNoise);
            this->get_parameter("initial_noise",initialNoise);
            this->declare_parameter("base_link_2_laser",baseLink2Laser);
            this->get_parameter("base_link_2_laser",baseLink2Laser);
            initialPose[3] *= M_PI / 180.0;
            initialPose[4] *= M_PI / 180.0;
            initialPose[5] *= M_PI / 180.0;
            baseLink2Laser[3] *= M_PI / 180.0;
            baseLink2Laser[4] *= M_PI / 180.0;
            baseLink2Laser[5] *= M_PI / 180.0;
            initialPose_.setPose(initialPose[0], initialPose[1], initialPose[2],
                initialPose[3], initialPose[4], initialPose[5]);
            initialNoise_.setPose(initialNoise[0], initialNoise[1], initialNoise[2],
                initialNoise[3], initialNoise[4], initialNoise[5]);
            mcl3d::Pose baseLink2Laser_(baseLink2Laser[0], baseLink2Laser[1], baseLink2Laser[2],
                baseLink2Laser[3], baseLink2Laser[4], baseLink2Laser[5]);
            mcl_.setInitialPose(initialPose_);
            mcl_.setBaseLink2Laser(baseLink2Laser_);
            mcl_.initializeParticles(initialPose_, initialNoise_);

             // set IMU parameters
            if (useIMU) {
                imu_.init();
                double imuSampleFreq = 100.0, AHRSFilterKp = 0.0, AHRSFilterKi = 0.0;
                this->declare_parameter("imu_sample_freq",imuSampleFreq);
                this->get_parameter("imu_sample_freq",imuSampleFreq);
                this->declare_parameter("ahrs_filter_kp",AHRSFilterKp);
                this->get_parameter("ahrs_filter_kp",AHRSFilterKp);
                this->declare_parameter("ahrs_filter_ki",AHRSFilterKi);
                this->get_parameter("ahrs_filter_ki",AHRSFilterKi);
                imu_.setSampleFreq(imuSampleFreq);
                imu_.setAHRSFilterGains(AHRSFilterKp, AHRSFilterKi);
            }

            // set optimization parameters

            // set measurement model parameters
            double zHit = 0.9, zRand = 0.05, zMax = 0.05;
            double varHit = 0.01, unknownLambda = 0.001, rangeReso = 0.1, rangeMax = 120.0;
            this->declare_parameter("z_hit",zHit);
            this->get_parameter("z_hit",zHit);
            this->declare_parameter("z_rand",zRand);
            this->get_parameter("z_rand",zRand);
            this->declare_parameter("z_max",zMax);
            this->get_parameter("z_max",zMax);
            this->declare_parameter("var_hit",varHit);
            this->get_parameter("var_hit",varHit);
            this->declare_parameter("unknown_lambda", unknownLambda);
            this->get_parameter("unknown_lambda", unknownLambda);
            this->declare_parameter("range_reso", rangeReso);
            this->get_parameter("range_reso", rangeReso);
            this->declare_parameter("range_max", rangeMax);
            this->get_parameter("range_max", rangeMax);
            mcl_.setMeasurementModelParameters(zHit, zRand, zMax, varHit, unknownLambda, rangeReso, rangeMax);

            std::vector<double> odomNoise = {1.0, 0.1, 0.1, 0.1, 0.1, 0.1,
                                            0.1, 1.0, 0.1, 0.1, 0.1, 0.1,
                                            0.1, 0.1, 1.0, 0.1, 0.1, 0.1,
                                            0.1, 0.1, 0.1, 1.0, 0.1, 0.1,
                                            0.1, 0.1, 0.1, 0.1, 1.0, 0.1,
                                            0.1, 0.1, 0.1, 0.1, 0.1, 1.0};
            this->declare_parameter("odom_noise", odomNoise);
            this->get_parameter("odom_noise", odomNoise);
            mcl_.setOdomNoise(odomNoise);

            bool useLinearInterpolation = true;
            this->declare_parameter("use_linear_interpolation", useLinearInterpolation);
            this->get_parameter("use_linear_interpolation", useLinearInterpolation);
            mcl_.setUseLinearInterpolation(useLinearInterpolation);

            //206
            double randomParticleRate = 0.1;
            double resampleThreshold = 0.5;
            std::vector<double> resampleNoise = {0.1, 0.1, 0.1, 0.05, 0.05, 0.05};
            this->declare_parameter("random_particle_rate", randomParticleRate);
            this->get_parameter("random_particle_rate", randomParticleRate);
            this->declare_parameter("resample_threshold", resampleThreshold);
            this->get_parameter("resample_threshold", resampleThreshold);
            this->declare_parameter("resample_noise", resampleNoise);
            this->get_parameter("resample_noise", resampleNoise);
             mcl3d::Pose resampleNoise_(resampleNoise[0], resampleNoise[1], resampleNoise[2],
            resampleNoise[3], resampleNoise[4], resampleNoise[5]);
            mcl_.setRandomParticleRate(randomParticleRate);
            mcl_.setResampleThreshold(resampleThreshold);
            mcl_.setResampleNoise(resampleNoise_);

            int optMaxIterNum = 30;
            double optMaxError = 1.0, convergenceThreshold = 0.02;
            this->declare_parameter("opt_max_iter_num", optMaxIterNum);
            this->get_parameter("opt_max_iter_num", optMaxIterNum);
            this->declare_parameter("opt_max_error", optMaxError);
            this->get_parameter("opt_max_error", optMaxError);
            this->declare_parameter("convergence_threshold", convergenceThreshold);
            this->get_parameter("convergence_threshold", convergenceThreshold);
            mcl_.setOptMaxIterNum(optMaxIterNum);
            mcl_.setOptMaxError(optMaxError);
            mcl_.setConvergenceThreshold(convergenceThreshold);

             // set fusion parameters
            int optParticleNum = 100;
            double optPoseCovCoef = 1.0, gmmPosVar = 0.3, gmmAngVar = 0.1;
            this->declare_parameter("optimized_particle_num", optParticleNum);
            this->get_parameter("optimized_particle_num", optParticleNum);
            this->declare_parameter("optimized_pose_cov_coef", optPoseCovCoef);
            this->get_parameter("optimized_pose_cov_coef", optPoseCovCoef);
            this->declare_parameter("gmm_postion_var", gmmPosVar);
            this->get_parameter("gmm_postion_var", gmmPosVar);
            this->declare_parameter("gmm_angle_var", gmmAngVar);
            this->get_parameter("gmm_angle_var", gmmAngVar);
            mcl_.setOptParticlsNum(optParticleNum);
            mcl_.setOptPoseCovCoef(optPoseCovCoef);
            mcl_.setGMMPosVar(gmmPosVar);
            mcl_.setGMMAngVar(gmmAngVar);

            // read tf parameters
            this->declare_parameter("transform_tolerance", transformTolerance_);
            this->get_parameter("transform_tolerance", transformTolerance_);
            this->declare_parameter("broadcast_tf", broadcastTF_);
            this->get_parameter("broadcast_tf", broadcastTF_);
            this->declare_parameter("use_odom_tf", useOdomTF_);
            this->get_parameter("use_odom_tf", useOdomTF_);

             // initialization for MCL
            if (!mcl_.checkParameters()) {
                RCLCPP_ERROR(this->get_logger(),"Incorrect parameters are given for MCL.");
                exit(1);
            }

            // load map or set map points subscriber
            std::string mapYamlFile = "/home/akai/Dropbox/git/git_ros_ws/src/als_ros_3d/data/dist_map_mcl_3dl.yaml";
            this->declare_parameter("map_yaml_file", mapYamlFile);
            this->get_parameter("map_yaml_file", mapYamlFile);
            RCLCPP_INFO(this->get_logger(),"The given map yaml file is %s.Start map loading.", mapYamlFile.c_str());
            if (!mcl_.loadDistanceMap(mapYamlFile)) {
                RCLCPP_ERROR(this->get_logger(),"Cannot read map yaml file -> %s", mapYamlFile.c_str());
                exit(1);
            }

            std::vector<mcl3d::Point> mapPoints = mcl_.getMapPoints();
            // sensor_msgs::msg::PointCloud2::SharedPtr mapPointsMsg;
             sensor_msgs::msg::PointCloud::SharedPtr mapPointsMsg;
            mapPointsMsg->header.frame_id = mapFrame_;
            // mapPointsMsg->data.resize((int)mapPoints.size());
            mapPointsMsg->points.resize((int)mapPoints.size());
            for (int i = 0; i < (int)mapPoints.size(); ++i) {
                geometry_msgs::msg::Point32::SharedPtr p;
                p->x = mapPoints[i].getX();
                p->y = mapPoints[i].getY();
                p->z = mapPoints[i].getZ();
                // mapPointsMsg->data[i] = *p;
                mapPointsMsg->points[i] = *p;
            }
            rclcpp::Time current_point_stamp = rclcpp::Clock(RCL_ROS_TIME).now();
            mapPointsMsg->header.stamp = current_point_stamp;
            mapPointsPub_ -> publish(*mapPointsMsg);
            //272
            this->declare_parameter("log_file", logFile_);
            this->get_parameter("log_file", logFile_);

            RCLCPP_INFO(this->get_logger(),"Initialization for MCL has been done.");
        }
        void sensorPointCB(sensor_msgs::msg::PointCloud2::SharedPtr msg){
            static double totalTime = 0.0;
            static int count = 0;

            mclPoseStamp_ = msg->header.stamp;
            pcl::PointCloud<pcl::PointXYZ> sensorPointsTmp;
            pcl::fromROSMsg(*msg, sensorPointsTmp);
            pcl::PointCloud<pcl::PointXYZ>::Ptr sensorPoints(new pcl::PointCloud<pcl::PointXYZ>);
            *sensorPoints = sensorPointsTmp;

            mcl_.updatePoses();
            std::chrono::system_clock::time_point start = std::chrono::system_clock::now();
            mcl_.calculateLikelihoodsByMeasurementModel(sensorPoints);
            mcl_.optimizeMeasurementModel(sensorPoints);
            mcl_.resampleParticles1();
            mcl_.resampleParticles2();
            std::chrono::system_clock::time_point end = std::chrono::system_clock::now();
            double time = static_cast<double>(std::chrono::duration_cast<std::chrono::microseconds>(end - start).count() / 1000.0);
            totalTime += time;
            count++;
            printf("time = %lf, average = %lf\n", time, totalTime / (double)count);
            mcl_.printMCLResult();
            publishROSMessages();
            broadcastTF();
            writeLog();
        }
        void imuCB(sensor_msgs::msg::Imu::SharedPtr msg){
            double qx = msg->orientation.x;
            double qy = msg->orientation.y;
            double qz = msg->orientation.z;
            double qw = msg->orientation.w;
            double norm = sqrt(qx * qx + qy * qy + qz * qz + qw * qw);
            if (norm > 0.99) {
                tf2::Quaternion q(qx, qy, qz, qw);
                double roll, pitch, yaw;
                tf2::Matrix3x3 mat(q);
                mat.getRPY(roll, pitch, yaw);
                imu_.setRPY(roll, pitch, yaw);
            } else {
                imu_.setAcceleration(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);
                imu_.setAngularVelocities(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);
                imu_.updateOrientation();
            }
            mcl_.setIMU(imu_);
        }
        void odomCB(nav_msgs::msg::Odometry::SharedPtr msg){
            tf2::Quaternion q(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
                            msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
            double roll, pitch, yaw;
            tf2::Matrix3x3 m(q);
            m.getRPY(roll, pitch, yaw);
            mcl3d::Pose odomPose(msg->pose.pose.position.x, msg->pose.pose.position.y,
                                    msg->pose.pose.position.z, roll, pitch, yaw);
            mcl3d::Pose odomVel(msg->twist.twist.linear.x, msg->twist.twist.linear.y, msg->twist.twist.linear.z,
            msg->twist.twist.angular.x, msg->twist.twist.angular.y, msg->twist.twist.angular.z);
            mcl_.setOdomPose(odomPose);
            mcl_.setOdomVelocities(odomVel);
        }

        void initialPoseCB(geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg){
            tf2::Quaternion q(msg->pose.pose.orientation.x,
                            msg->pose.pose.orientation.y,
                            msg->pose.pose.orientation.z,
                            msg->pose.pose.orientation.w);
            double roll, pitch, yaw;
            tf2::Matrix3x3 m(q);
            m.getRPY(roll,pitch,yaw);
            mcl3d::Pose initialPose(msg->pose.pose.position.x, msg->pose.pose.position.y,
                                    msg->pose.pose.position.z, roll, pitch, yaw);
            mcl_.setInitialPose(initialPose);
            mcl_.initializeParticles(initialPose, initialNoise_);
        }

        void publishROSMessages(void) {
            //particles
            std::vector<mcl3d::Particle> particles = mcl_.getParticles();
            geometry_msgs::msg::PoseArray::SharedPtr particlesPoses;
            particlesPoses->header.frame_id = mapFrame_;
            particlesPoses->header.stamp = mclPoseStamp_;
            particlesPoses->poses.resize((int)particles.size());
            for (int i = 0; i < (int)particles.size(); ++i){
                geometry_msgs::msg::Pose::SharedPtr pose;
                pose->position.x = particles[i].getX();
                pose->position.y = particles[i].getY();
                pose->position.z = particles[i].getZ();
                tf2::Quaternion q;
                q.setRPY(particles[i].getRoll(),particles[i].getRoll(),particles[i].getYaw());
                // tf2::convert(pose->orientation, mclQuat);
                // tf2::fromMsg(mclQuat,pose->orientation);
                pose->orientation = tf2::toMsg(q);
                particlesPoses->poses[i] = *pose;
            }
            //pub
            particlesPub_->publish(*particlesPoses);

            //optimized particles
            std::vector<mcl3d::Particle> optParticles = mcl_.getOptParticles();
            geometry_msgs::msg::PoseArray::SharedPtr optParticlesPoses;
            optParticlesPoses->header.frame_id =mapFrame_;
            optParticlesPoses->header.stamp =mclPoseStamp_;
            optParticlesPoses->poses.resize((int)optParticles.size());
            for (int i = 0; i < (int)optParticles.size(); ++i) {
                geometry_msgs::msg::Pose::SharedPtr pose;
                pose->position.x = optParticles[i].getX();
                pose->position.y = optParticles[i].getY();
                pose->position.z = optParticles[i].getZ();
                tf2::Quaternion q;
                q.setRPY(optParticles[i].getRoll(),optParticles[i].getPitch(),optParticles[i].getYaw());
                pose->orientation = tf2::toMsg(q);
                // tf2::convert(q, pose->orientation);
                // tf2::fromMsg(q,pose->orientation);
                optParticlesPoses->poses[i] = *pose;
            }
            //pub
            optParticlesPub_->publish(*optParticlesPoses);
            
            // mcl and optimized poses 396
            geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr mclPoseMsg, optPoseMsg;
            mcl3d::Pose mclPose = mcl_.getMCLPose();
            mcl3d::Pose optPose = mcl_.getOptPose();

            mclPoseMsg->header.frame_id = mapFrame_;
            mclPoseMsg->header.stamp = mclPoseStamp_;
            mclPoseMsg->pose.pose.position.x = mclPose.getX();
            mclPoseMsg->pose.pose.position.y = mclPose.getY();
            mclPoseMsg->pose.pose.position.z = mclPose.getZ();
            tf2::Quaternion mclQuat;
            mclQuat.setRPY(mclPose.getRoll(),mclPose.getPitch(),mclPose.getYaw());
            mclPoseMsg->pose.pose.orientation = tf2::toMsg(mclQuat);
            // tf2::convert(mclQuat, mclPoseMsg->pose.pose.orientation);
            // tf2::fromMsg(mclQuat, mclPoseMsg->pose.pose.orientation);

            optPoseMsg->header.frame_id = mapFrame_;
            optPoseMsg->header.stamp = mclPoseStamp_;
            optPoseMsg->pose.pose.position.x = optPose.getX();
            optPoseMsg->pose.pose.position.y = optPose.getY();
            optPoseMsg->pose.pose.position.z = optPose.getZ();
            tf2::Quaternion optQuat;
            optQuat.setRPY(optPose.getRoll(),optPose.getPitch(),optPose.getYaw());
            optPoseMsg->pose.pose.orientation = tf2::toMsg(optQuat);
            // tf2::convert(optQuat, optPoseMsg->pose.pose.orientation);
            // tf2::fromMsg(optQuat, optPoseMsg->pose.pose.orientation);

            std::vector<std::vector<double>> poseCov = mcl_.getPoseCovariance();
            std::vector<std::vector<double>> optPoseCov = mcl_.getOptPoseCovariance();
            for (int i = 0; i < 6; ++i) {
                for (int j = 0; j < 6; ++j) {
                mclPoseMsg->pose.covariance[j *6 + i] = poseCov[i][j];
                optPoseMsg->pose.covariance[j *6 + i] = optPoseCov[i][j];
                }
            }
            //pub
            posePub_->publish(*mclPoseMsg);
            optPosePub_->publish(*optPoseMsg);

            // aligned points
            pcl::PointCloud<pcl::PointXYZ> alignedPointsOpt = mcl_.getAlignedPointsOpt();
            sensor_msgs::msg::PointCloud::SharedPtr alignedPointsOptMsg;
            // sensor_msgs::msg::PointCloud2::SharedPtr alignedPointsOptMsg;
            // alignedPointsOptMsg->data.resize((int)alignedPointsOpt.points.size());
            alignedPointsOptMsg->points.resize((int)alignedPointsOpt.points.size());
            for (int i = 0; i < (int)alignedPointsOpt.points.size(); ++i) {
                geometry_msgs::msg::Point32::SharedPtr p;
                p->x = alignedPointsOpt.points[i].x;
                p->y = alignedPointsOpt.points[i].y;
                p->z = alignedPointsOpt.points[i].z;
                
                alignedPointsOptMsg->points[i] = *p;
            }
            alignedPointsOptMsg->header.frame_id = laserFrame_;
            alignedPointsOptMsg->header.stamp = mclPoseStamp_;
            alignedPointsOptPub_->publish(*alignedPointsOptMsg);
    }
    void broadcastTF(void) {
        if (!broadcastTF_)
            return;

        mcl3d::Pose mclPose = mcl_.getMCLPose();
        geometry_msgs::msg::Pose poseOnMap;
        poseOnMap.position.x = mclPose.getX();
        poseOnMap.position.y = mclPose.getY();
        poseOnMap.position.z = mclPose.getZ();
        tf2::Quaternion mclQuat;
        mclQuat.setRPY(mclPose.getRoll(),mclPose.getPitch(),mclPose.getYaw());
        poseOnMap.orientation = tf2::toMsg(mclQuat);
        tf2::Transform map2baseTrans;
        tf2::convert(poseOnMap,map2baseTrans);
        // tf2::convert(mclQuat, poseOnMap.orientation);
        // tf2::convert(mclQuat, poseOnMap.orientation);

        if (useOdomTF_) {
            // make TF tree as map -> odom -> base_link -> laser
            mcl3d::Pose odomPose = mcl_.getOdomPose();
            geometry_msgs::msg::Pose poseOnOdom;
            poseOnOdom.position.x = odomPose.getX();
            poseOnOdom.position.y = odomPose.getY();
            poseOnOdom.position.z = odomPose.getZ();
            tf2::Quaternion odomQuat;
            odomQuat.setRPY(odomPose.getRoll(),odomPose.getPitch(),odomPose.getYaw());
            poseOnOdom.orientation= tf2::toMsg(odomQuat);
            tf2::Transform odom2baseTrans;
            tf2::convert(poseOnOdom,odom2baseTrans);

            tf2::Transform map2odomTrans = map2baseTrans * odom2baseTrans.inverse();
            rclcpp::Duration trans_duration = rclcpp::Duration::from_seconds(transformTolerance_);
            rclcpp::Time transformExpiration =(mclPoseStamp_ + trans_duration);
            geometry_msgs::msg::TransformStamped map2odomStampedTrans;
            map2odomStampedTrans.header.stamp = transformExpiration;
            map2odomStampedTrans.header.frame_id = mapFrame_;
            map2odomStampedTrans.child_frame_id = odomFrame_;
            tf2::convert(map2odomTrans, map2odomStampedTrans.transform);
            tfBroadcaster_->sendTransform(map2odomStampedTrans);

        }
        else {
            // make TF tree as map -> base_link -> laser
            geometry_msgs::msg::TransformStamped map2baseStampedTrans;
            map2baseStampedTrans.header.stamp = mclPoseStamp_;
            map2baseStampedTrans.header.frame_id = mapFrame_;
            map2baseStampedTrans.child_frame_id = baseLinkFrame_;
            tf2::convert(map2baseTrans, map2baseStampedTrans.transform);
            tfBroadcaster_->sendTransform(map2baseStampedTrans);
        }

        // optimized pose
        mcl3d::Pose optPose = mcl_.getOptPose();
        geometry_msgs::msg::Pose optPoseOnMap;
        optPoseOnMap.position.x = optPose.getX();
        optPoseOnMap.position.y = optPose.getY();
        optPoseOnMap.position.z = optPose.getZ();
        tf2::Quaternion optQuat ;
        optQuat.setRPY(optPose.getRoll(),optPose.getPitch(),optPose.getYaw());
        optPoseOnMap.orientation =tf2::toMsg(optQuat);
        tf2::Transform map2optPoseTrans;
        tf2::convert(optPoseOnMap, map2optPoseTrans);
        geometry_msgs::msg::TransformStamped map2optPoseStampedTrans;
        map2optPoseStampedTrans.header.stamp = mclPoseStamp_;
        map2optPoseStampedTrans.header.frame_id = mapFrame_;
        map2optPoseStampedTrans.child_frame_id = optPoseFrame_;
        tf2::convert(map2optPoseTrans, map2optPoseStampedTrans.transform);
        tfBroadcaster_->sendTransform(map2optPoseStampedTrans);
    }
    void writeLog(void) {
        if (!writeLog_)
            return;

        static FILE *fp;
        if (fp == NULL) {
            fp = fopen(logFile_.c_str(), "w");
            if (fp == NULL) {
                fprintf(stderr, "Cannot open %s\n", logFile_.c_str());
                return;
            }
        }

        mcl3d::Pose mclPose = mcl_.getMCLPose();
        mcl3d::Pose optPose = mcl_.getOptPose();
        fprintf(fp, "%ld "
            "%lf %lf %lf %lf %lf %lf "
            "%lf %lf %lf %lf %lf %lf\n",
            mclPoseStamp_.nanoseconds(),
            mclPose.getX(), mclPose.getY(), mclPose.getZ(), mclPose.getRoll(), mclPose.getPitch(), mclPose.getYaw(),
            optPose.getX(), optPose.getY(), optPose.getZ(), optPose.getRoll(), optPose.getPitch(), optPose.getYaw());
    }
    // 
    



    
            };
    int main(int argc, char * argv[])
    {
        rclcpp::init(argc, argv);
        auto node = std::make_shared<MCLNode>();
        rclcpp::Rate loop_rate(20.0);
        while (rclcpp::ok())
        {
            rclcpp::spin_some(node);
            loop_rate.sleep();
        }
        return 0;
    }