#include <iostream>
#include <argparse/argparse.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/filters/crop_hull.h>

// Function to load point cloud
pcl::PointCloud<pcl::PointXYZ>::Ptr loadPointCloud(const std::string &filename) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(filename, *cloud) == -1) {
        PCL_ERROR("Couldn't read file %s \n", filename.c_str());
        return nullptr;
    }
    std::cout << "Loaded " << cloud->width * cloud->height << " data points from " << filename << std::endl;
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

    std::cout << "Convex hull computed" << std::endl;
    return {hullCloud, hullPolygons};
}

// Function to crop points within hull
void cropPoints(const pcl::PointCloud<pcl::PointXYZ>::Ptr &inputCloud, const pcl::PointCloud<pcl::PointXYZ>::Ptr &hullCloud,
                const std::vector<pcl::Vertices> &hullPolygons, pcl::PointCloud<pcl::PointXYZ>::Ptr &outputCloud) {
    pcl::CropHull<pcl::PointXYZ> cropHullFilter;
    cropHullFilter.setHullIndices(hullPolygons);
    cropHullFilter.setHullCloud(hullCloud);
    cropHullFilter.setDim(3); // Use 3D cropping

    cropHullFilter.setInputCloud(inputCloud);
    cropHullFilter.filter(*outputCloud);
}

int main(int argc, char *argv[]) {
    argparse::ArgumentParser program("pointcloud_utils", "0.1");

    program.add_argument("input").help("Input point cloud file").required();
    program.add_argument("cameras").help("Camera position point cloud file").required();
    program.add_argument("output").help("Output point cloud file").required();

    try {
        program.parse_args(argc, argv);
    }
    catch (const std::exception &err) {
        std::cerr << err.what() << std::endl;
        std::cerr << program;
        return 1;
    }

    auto input = program.get<std::string>("input");
    auto cameras = program.get<std::string>("cameras");
    auto output = program.get<std::string>("output");

    std::cout << "Cameras: " << cameras << std::endl;
    std::cout << "Input: " << input << std::endl;
    std::cout << "Output: " << output << std::endl;

    // Load main cloud
    auto cloud = loadPointCloud(input);
    if (!cloud) {
        return -1;
    }

    // Load camera cloud and compute hull
    auto [hullCloud, hullPolygons] = loadCameraAndComputeHull(cameras);
    if (!hullCloud) {
        return -1;
    }

    // Crop points within the hull
    pcl::PointCloud<pcl::PointXYZ>::Ptr filteredCloud(new pcl::PointCloud<pcl::PointXYZ>);
    cropPoints(cloud, hullCloud, hullPolygons, filteredCloud);

    // Save filtered point cloud
    pcl::io::savePCDFile(output, *filteredCloud);
    std::cout << "Saved filtered point cloud to " << output << std::endl;

    return 0;
}
