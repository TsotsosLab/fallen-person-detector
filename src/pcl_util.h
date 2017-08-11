//
// Created by markus d. solbach
// solbach@eecs.yorku.ca
//

#ifndef FALLEN_PERSON_DETECTOR_PCL_UTIL_H
#define FALLEN_PERSON_DETECTOR_PCL_UTIL_H

#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/progressive_morphological_filter.h>
#include <pcl/common/centroid.h>
#include <pcl/common/geometry.h>
#include <pcl/surface/convex_hull.h>

class pcl_util {
public:

    struct color {
        uint8_t r;
        uint8_t g;
        uint8_t b;
    };

    pcl_util();

    void displayCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, std::string name);

    void displayCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in, std::string name);

    void displayCloud(pcl::PCLPointCloud2::Ptr cloud_in, std::string name);

    void colorCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in,
                    color color);

    void colorCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in, pcl::PointIndices::Ptr indices,
                    color color);

    void printCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud) const;

    void printIndices(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, const pcl::PointIndices::Ptr &inliers) const;

    void concatClouds(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_dest,
                      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_src,
                      pcl_util::color color);

    pcl::PointCloud<pcl::PointXYZ>::Ptr &
    generateRandomCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, int elements) const;

    int getColorSize();

    pcl_util::color getColor(int index);

    pcl::_PointXYZRGB vector4fToPointRGB(Eigen::Vector4f in, pcl_util::color color);

    pcl::_PointXYZRGB calculateCentroidWAugmentation(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_dest,
                                                     pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_src,
                                                     pcl_util::color color);

    pcl::_PointXYZRGB calculateCentroid(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_src,
                                        pcl_util::color color);

    float calculateShortestDistance(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointXYZ point);

    float calculateShortestDistance(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointXYZRGB point);

    std::vector<double> calculateAreaVolume(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in);

private:

    void init(void);

    pcl::visualization::CloudViewer *viewer;
    std::vector<pcl_util::color> util_color;

    float euclideanDistance(pcl::PointXYZ pt1, pcl::PointXYZ pt2);
};

#endif //FALLEN_PERSON_DETECTOR_PCL_UTIL_H
