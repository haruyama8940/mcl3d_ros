#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.hpp>
#include <pcl/io/pcd_io.h>
#include <mcl3d_ros/DistanceField.hpp>

mcl3d::DistanceField distMap;
bool doneMapBuild = false;

std::string mapFileName, yamlFilePath;
float resolution, subMapResolution, mapMargin;

bool buildDistanceField(pcl::PointCloud<pcl::PointXYZ> mapPoints, std::string mapFileName,
    float resolution, float subMapResolution, float mapMargin, std::string yamlFilePath)
{
    mcl3d::Point minPoint(mapPoints.points[0].x, mapPoints.points[0].y, mapPoints.points[0].z);
    mcl3d::Point maxPoint(mapPoints.points[0].x, mapPoints.points[0].y, mapPoints.points[0].z);
    for (int i = 1; i < (int)mapPoints.size(); ++i) {
        float x = mapPoints[i].x;
        float y = mapPoints[i].y;
        float z = mapPoints[i].z;
        if (minPoint.getX() > x)
            minPoint.setX(x);
        if (minPoint.getY() > y)
            minPoint.setY(y);
        if (minPoint.getZ() > z)
            minPoint.setZ(z);
        if (maxPoint.getX() < x)
            maxPoint.setX(x);
        if (maxPoint.getY() < y)
            maxPoint.setY(y);
        if (maxPoint.getZ() < z)
            maxPoint.setZ(z);
    }
    RCLCPP_INFO(rclcpp::get_logger("map_builder"), "Min map point: %f %f %f", minPoint.getX(), minPoint.getY(), minPoint.getZ());
    RCLCPP_INFO(rclcpp::get_logger("map_builder"), "Max map point: %f %f %f", maxPoint.getX(), maxPoint.getY(), maxPoint.getZ());

    distMap = mcl3d::DistanceField(mapFileName, resolution, subMapResolution, mapMargin, minPoint, maxPoint, yamlFilePath);
    // FILE *fp = fopen("/tmp/map_points_by_pc_to_df.txt", "w");
    for (int i = 0; i < (int)mapPoints.size(); ++i) {
        float x = mapPoints.points[i].x;
        float y = mapPoints.points[i].y;
        float z = mapPoints.points[i].z;
        // fprintf(fp, "%f %f %f\n", x, y, z);
        distMap.addPoint(x, y, z);
    }
    // fclose(fp);

    // distMap.writeMapPoints("/tmp/df_map_points.txt", minPoint.getX(), maxPoint.getX(),
    //     minPoint.getY(), maxPoint.getY(), minPoint.getZ(), maxPoint.getZ());
    // exit(0);

    if (!distMap.saveDistanceMap()) {
        RCLCPP_ERROR(rclcpp::get_logger("map_builder"), "Error occurred during the distance field building.");
        return false;
    }

    return true;
}

void mapPointsCB(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    pcl::PointCloud<pcl::PointXYZ> mapPoints;
    pcl::fromROSMsg(*msg, mapPoints);
    doneMapBuild = buildDistanceField(mapPoints, mapFileName, resolution, subMapResolution, mapMargin, yamlFilePath);
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;
    auto node = rclcpp::Node::make_shared("pc_to_df", options);
    /*
    if (argv[4] != NULL && argv[6] != NULL) {
    // if (argc >= 7) {
        // build a distance field map from a PCD file
        // do not use any ROS functions
        std::string pcdFile = argv[1];
        mapFileName = argv[2];
        resolution = std::atof(argv[3]);
        subMapResolution = std::stof(argv[4]);
        mapMargin = std::atof(argv[5]);
        yamlFilePath = argv[6];

        RCLCPP_INFO(node->get_logger(), "pcdFile: %s", pcdFile.c_str());
        RCLCPP_INFO(node->get_logger(), "mapFileName: %s", mapFileName.c_str());
        RCLCPP_INFO(node->get_logger(), "resolution: %f [m]", resolution);
        RCLCPP_INFO(node->get_logger(), "subMapResolution: %f [m]", subMapResolution);
        RCLCPP_INFO(node->get_logger(), "mapMargin: %f [m]", mapMargin);
        RCLCPP_INFO(node->get_logger(), "yamlFilePath: %s", yamlFilePath.c_str());

        pcl::PointCloud<pcl::PointXYZ> mapPoints;
        pcl::io::loadPCDFile(pcdFile, mapPoints);
        doneMapBuild = buildDistanceField(mapPoints, mapFileName, resolution, subMapResolution, mapMargin, yamlFilePath);
    } else {
        */
        // get point cloud from a ROS message and build a distance field map
        std::string pcdFile = "map.pcd";
        std::string mapPointsName = "/map_points";
        mapFileName = "dist_map.bin";
        resolution = 5.0f;
        subMapResolution = 0.1f;
        mapMargin = 1.0f;
        yamlFilePath = "/tmp/dist_map.yaml";
        node->declare_parameter("pcd_file",pcdFile);
        node->declare_parameter("map_points_name", mapPointsName);
        node->declare_parameter("map_file_name", mapFileName);
        node->declare_parameter("resolution", resolution);
        node->declare_parameter("sub_map_resolution", subMapResolution);
        node->declare_parameter("map_margin", mapMargin);
        node->declare_parameter("yaml_file_path", yamlFilePath);
        node->get_parameter("pcd_file",pcdFile);
        node->get_parameter("map_points_name", mapPointsName);
        node->get_parameter("map_file_name", mapFileName);
        node->get_parameter("resolution", resolution);
        node->get_parameter("sub_map_resolution", subMapResolution);
        node->get_parameter("map_margin", mapMargin);
        node->get_parameter("yaml_file_path", yamlFilePath);

        RCLCPP_INFO(node->get_logger(), "mapPointsName: %s", mapPointsName.c_str());
        RCLCPP_INFO(node->get_logger(), "mapFileName: %s", mapFileName.c_str());
        RCLCPP_INFO(node->get_logger(), "resolution: %f [m]", resolution);
        RCLCPP_INFO(node->get_logger(), "subMapResolution: %f [m]", subMapResolution);
        RCLCPP_INFO(node->get_logger(), "mapMargin: %f [m]", mapMargin);
        RCLCPP_INFO(node->get_logger(), "yamlFilePath: %s", yamlFilePath.c_str());

        pcl::PointCloud<pcl::PointXYZ> mapPoints;
        pcl::io::loadPCDFile(pcdFile, mapPoints);
        doneMapBuild = buildDistanceField(mapPoints, mapFileName, resolution, subMapResolution, mapMargin, yamlFilePath);
        // auto mapPointsCBFunc = [&node](const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        //     pcl::PointCloud<pcl::PointXYZ> mapPoints;
        //     pcl::fromROSMsg(*msg, mapPoints);
        //     doneMapBuild = buildDistanceField(mapPoints, mapFileName, resolution, subMapResolution, mapMargin, yamlFilePath);
        // };

        // auto mapPointsSub = node->create_subscription<sensor_msgs::msg::PointCloud2>(
        //     mapPointsName, 1, mapPointsCBFunc);

        rclcpp::Rate loopRate(1.0);
        while (rclcpp::ok()) {
            rclcpp::spin_some(node);
            if (doneMapBuild)
                break;
            loopRate.sleep();
        }
    // }

    rclcpp::shutdown();

    return 0;
}
