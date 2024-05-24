#include <iostream>
#include <argparse/argparse.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/filters/crop_hull.h>
#include "easylogging++.h"
#include <pcl/common/common.h>
#include <pcl/filters/passthrough.h>


INITIALIZE_EASYLOGGINGPP


// Function to load point cloud
pcl::PointCloud<pcl::PointXYZ>::Ptr loadPointCloud(const std::string &filename) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(filename, *cloud) == -1) {
        PCL_ERROR("Couldn't read file %s \n", filename.c_str());
        return nullptr;
    }
    return cloud;
}

// Function to load camera point cloud and compute convex hull
std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, std::vector<pcl::Vertices>>
loadCameraAndComputeHull(const std::string &cameras) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cameraCloud = loadPointCloud(cameras);
    if (!cameraCloud) {
        return {nullptr, {}};
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr hullCloud(new pcl::PointCloud<pcl::PointXYZ>);
    std::vector<pcl::Vertices> hullPolygons;
    pcl::ConvexHull<pcl::PointXYZ> chull;
    chull.setInputCloud(cameraCloud);
    chull.reconstruct(*hullCloud, hullPolygons);

    return {hullCloud, hullPolygons};
}


pcl::PointCloud<pcl::PointXYZ>::Ptr cropByBounds(const pcl::PointCloud<pcl::PointXYZ>::Ptr &inputCloud, const pcl::PointCloud<pcl::PointXYZ>::Ptr &hullCloud, float margin = 0.1){

    // Create a bounding box from the hull
    pcl::PointXYZ minPt, maxPt;
    pcl::getMinMax3D(*hullCloud, minPt, maxPt);

    pcl::PointCloud<pcl::PointXYZ>::Ptr filteredCloud(new pcl::PointCloud<pcl::PointXYZ>);


    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(inputCloud);
    pass.setFilterFieldName("x");
    pass.setFilterLimits(minPt.x - (minPt.x * margin), maxPt.x + (minPt.x * margin));
    pass.filter(*filteredCloud);

    pass.setInputCloud(filteredCloud);
    pass.setFilterFieldName("y");
    pass.setFilterLimits(minPt.y - (minPt.y * margin), maxPt.y + (minPt.y * margin));
    pass.filter(*filteredCloud);

    pass.setInputCloud(filteredCloud);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(minPt.z - (minPt.z * margin), maxPt.z + (minPt.z * margin));
    pass.filter(*filteredCloud);

    return filteredCloud;
}

// Function to crop points within hull
pcl::PointCloud<pcl::PointXYZ>::Ptr
cropPoints(const pcl::PointCloud<pcl::PointXYZ>::Ptr &inputCloud, const pcl::PointCloud<pcl::PointXYZ>::Ptr &hullCloud,
           const std::vector<pcl::Vertices> &hullPolygons) {
    pcl::CropHull<pcl::PointXYZ> cropHullFilter;
    cropHullFilter.setHullIndices(hullPolygons);
    cropHullFilter.setHullCloud(hullCloud);
    cropHullFilter.setDim(3); // Use 3D cropping


    pcl::PointCloud<pcl::PointXYZ>::Ptr filteredCloud(new pcl::PointCloud<pcl::PointXYZ>);


    cropHullFilter.setInputCloud(inputCloud);
    cropHullFilter.filter(*filteredCloud);
    return filteredCloud;
}

int main(int argc, char *argv[]) {
    argparse::ArgumentParser program("pointcloud_utils", "0.1");
    START_EASYLOGGINGPP(argc, argv);


    program.add_argument("inputFilePath").help("Input point cloud file").required();
    program.add_argument("cameraFilePath").help("Camera position point cloud file").required();
    program.add_argument("outputFilePath").help("Output point cloud file").required();

    try {
        program.parse_args(argc, argv);
    }
    catch (const std::exception &err) {
        std::cerr << err.what() << std::endl;
        std::cerr << program;
        return 1;
    }

    auto inputFilePath = program.get<std::string>("inputFilePath");
    auto cameraFilePath = program.get<std::string>("cameraFilePath");
    auto outputFilePath = program.get<std::string>("outputFilePath");

    LOG(INFO) << "Input: " << inputFilePath << std::endl;
    LOG(INFO) << "Cameras: " << cameraFilePath << std::endl;
    LOG(INFO) << "Output: " << outputFilePath << std::endl;

    // Load main cloud
    auto cloud = loadPointCloud(inputFilePath);
    if (!cloud) {
        return -1;
    }

    LOG(INFO) << "Loaded " << cloud->width * cloud->height << " data points from " << inputFilePath << std::endl;


    // Load camera cloud and compute hull
    auto [hullCloud, hullPolygons] = loadCameraAndComputeHull(cameraFilePath);
    if (!hullCloud) {
        return -1;
    }


    LOG(INFO) << "Convex hull computed" << std::endl;


    auto preCroppedCount = cloud->width * cloud->height;

    // Crop points within bounding box
    auto filteredCloud = cropByBounds(cloud, hullCloud);
    auto postBoxCroppedCount = filteredCloud->width * filteredCloud->height;

    auto diff = preCroppedCount - postBoxCroppedCount;

    LOG(INFO) << "Filtered " << diff << " data points" << std::endl;


    // Crop points within the hull
    auto newCloud = cropPoints(filteredCloud, hullCloud, hullPolygons);
    auto postHullCroppedCount = newCloud->width * newCloud->height;

    auto diff2 = postBoxCroppedCount - postHullCroppedCount;

    LOG(INFO) << "Filtered " << diff2 << " data points" << std::endl;


    LOG(INFO) << "Removed total of " << diff + diff2 << " data points" << std::endl;

    // Save filtered point cloud
    pcl::io::savePCDFile(outputFilePath, *newCloud);
    LOG(INFO) << "Saved filtered point cloud to " << outputFilePath << std::endl;

    return 0;
}
