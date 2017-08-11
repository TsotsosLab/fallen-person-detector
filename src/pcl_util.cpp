//
// Created by markus d. solbach
// solbach@eecs.yorku.ca
//

#include "pcl_util.h"

pcl_util::pcl_util() {
    this->init();
}

void pcl_util::init() {
    // init 10 color
    pcl_util::color col;
    // Red
    col.r = 255;
    col.g = 0;
    col.b = 0;
    this->util_color.push_back(col);
    // Green
    col.r = 0;
    col.g = 190;
    col.b = 0;
    this->util_color.push_back(col);
    // Blue
    col.r = 67;
    col.g = 133;
    col.b = 255;
    this->util_color.push_back(col);
    // Orange
    col.r = 255;
    col.g = 150;
    col.b = 0;
    this->util_color.push_back(col);
    // Yellow
    col.r = 255;
    col.g = 235;
    col.b = 0;
    this->util_color.push_back(col);
    // Lavender
    col.r = 230;
    col.g = 190;
    col.b = 255;
    this->util_color.push_back(col);
    // Teal
    col.r = 0;
    col.g = 128;
    col.b = 128;
    this->util_color.push_back(col);
    // Magenta
    col.r = 255;
    col.g = 0;
    col.b = 255;
    this->util_color.push_back(col);
    // Olive
    col.r = 128;
    col.g = 128;
    col.b = 0;
    this->util_color.push_back(col);
    // Pink
    col.r = 255;
    col.g = 200;
    col.b = 220;
    this->util_color.push_back(col);
}

void pcl_util::displayCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, std::string name) {

    this->viewer = new pcl::visualization::CloudViewer(name);
    viewer->showCloud(cloud_in);

    while (!viewer->wasStopped()) {
    }
}

void pcl_util::displayCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in, std::string name) {

    this->viewer = new pcl::visualization::CloudViewer(name);
    viewer->showCloud(cloud_in);

    while (!viewer->wasStopped()) {
    }
}

void pcl_util::displayCloud(pcl::PCLPointCloud2::Ptr cloud_in, std::string name) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_display(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(*cloud_in, *cloud_display);

    this->viewer = new pcl::visualization::CloudViewer(name);
    viewer->showCloud(cloud_display);

    while (!viewer->wasStopped()) {
    }
}

pcl::PointCloud<pcl::PointXYZ>::Ptr &
pcl_util::generateRandomCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, int elements) const {// Fill in the cloud data
    cloud->width = elements;
    cloud->height = 1;
    cloud->points.resize(cloud->width * cloud->height);

    // Generate the data
    for (size_t i = 0; i < cloud->points.size(); ++i) {
        cloud->points[i].x = 1024 * rand() / (RAND_MAX + 1.0f);
        cloud->points[i].y = 1024 * rand() / (RAND_MAX + 1.0f);
        cloud->points[i].z = 1.0;
    }

    // Set a few outliers
    cloud->points[0].z = 2.0;
    cloud->points[3].z = -2.0;
    cloud->points[6].z = 4.0;
    return cloud;
}

void pcl_util::printIndices(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
                            const pcl::PointIndices::Ptr &inliers) const {
    for (size_t i = 0; i < inliers->indices.size(); ++i)
        cerr << inliers->indices[i] << "    " << cloud->points[inliers->indices[i]].x << " "
             << cloud->points[inliers->indices[i]].y << " "
             << cloud->points[inliers->indices[i]].z << endl;
}

void pcl_util::printCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud) const {
    for (size_t i = 0; i < cloud->points.size(); ++i)
        cerr << "    " << cloud->points[i].x << " "
             << cloud->points[i].y << " "
             << cloud->points[i].z << endl;
}

void pcl_util::colorCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in, pcl_util::color color) {
    for (int i = 0; i < cloud_in->points.size(); i++) {
        cloud_in->points[i].r = color.r;
        cloud_in->points[i].g = color.g;
        cloud_in->points[i].b = color.b;
    }
}

void pcl_util::colorCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in, pcl::PointIndices::Ptr indices,
                          pcl_util::color color) {

    for (int i = 0; i < indices->indices.size(); i++) {
        //std::cout << i << ". Index: " << indices->indices.at(i) << std::endl;
        cloud_in->points[indices->indices.at(i)].r = color.r;
        cloud_in->points[indices->indices.at(i)].g = color.g;
        cloud_in->points[indices->indices.at(i)].b = color.b;
    }
}


void pcl_util::concatClouds(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_dest,
                            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_src,
                            pcl_util::color color) {
    pcl::_PointXYZRGB pt;
    for (int i = 0; i < cloud_src->size(); i++) {
        //std::cout << i << ". Index: " << indices->indices.at(i) << std::endl;
        pt.x = cloud_src->points[i].x;
        pt.y = cloud_src->points[i].y;
        pt.z = cloud_src->points[i].z;
        pt.r = color.r;
        pt.g = color.g;
        pt.b = color.b;
        cloud_dest->push_back(pt);
    }
}

int pcl_util::getColorSize() {
    return this->util_color.size();
}

pcl_util::color pcl_util::getColor(int index) {
    if (index >= this->util_color.size()) {
        return this->util_color.at(0);
    } else {
        return this->util_color.at(index);
    }
}


pcl::_PointXYZRGB pcl_util::vector4fToPointRGB(Eigen::Vector4f in, pcl_util::color color) {
    pcl::_PointXYZRGB pt;
    pt.x = in.x();
    pt.y = in.y();
    pt.z = in.z();
    pt.r = color.r;
    pt.g = color.g;
    pt.b = color.b;

    return pt;
}

pcl::_PointXYZRGB pcl_util::calculateCentroidWAugmentation(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_dest,
                                                           pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_src,
                                                           pcl_util::color color) {

    pcl::PointXYZRGB centroidRGB;
    centroidRGB = pcl_util::calculateCentroid(cloud_src, color);
    cloud_dest->push_back(centroidRGB);
    return centroidRGB;
}

pcl::_PointXYZRGB pcl_util::calculateCentroid(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_src,
                                              pcl_util::color color) {
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*cloud_src, centroid);
    pcl::PointXYZRGB centroidRGB;
    centroidRGB = pcl_util::vector4fToPointRGB(centroid, color);
    return centroidRGB;
}

float pcl_util::euclideanDistance(pcl::PointXYZ pt1, pcl::PointXYZ pt2) {
    float result;
    result = ((pt1.x - pt2.x) * (pt1.x - pt2.x)) + ((pt1.y - pt2.y) * (pt1.y - pt2.y)) +
             ((pt1.z - pt2.z) * (pt1.z - pt2.z));
    return pow(result, 0.5);
}

float pcl_util::calculateShortestDistance(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, const pcl::PointXYZ point) {
    pcl::PointXYZ temp_pt;
    float shortest = 999999;
    float temp_dis;
    for (int i = 0; i < cloud->size(); ++i) {
        temp_pt.x = cloud->points[i].x;
        temp_pt.y = cloud->points[i].y;
        temp_pt.z = cloud->points[i].z;

        temp_dis = pcl_util::euclideanDistance(temp_pt, point);
        if (temp_dis < shortest)
            shortest = temp_dis;
    }

    return shortest;
}

float pcl_util::calculateShortestDistance(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, const pcl::PointXYZRGB point_in) {
    pcl::PointXYZ ref_point;
    ref_point.x = point_in.x;
    ref_point.y = point_in.y;
    ref_point.z = point_in.z;

    return pcl_util::calculateShortestDistance(cloud, ref_point);
}

std::vector<double> pcl_util::calculateAreaVolume(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in) {

    std::vector<double> results;
    pcl::ConvexHull<pcl::PointXYZ> hull;
    pcl::PointCloud<pcl::PointXYZ> cloud_convex;
    hull.setDimension(3);
    hull.setComputeAreaVolume(true);
    hull.setInputCloud(cloud_in);
    hull.reconstruct(cloud_convex);

    results.push_back(hull.getTotalArea());
    results.push_back(hull.getTotalVolume());

    return results;
}
