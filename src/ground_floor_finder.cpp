//
// Created by markus d. solbach
// solbach@eecs.yorku.ca
//
#include "ground_floor_finder.h"

ground_floor_finder::ground_floor_finder() {
    init();
}

void ground_floor_finder::init() {
    this->util = new pcl_util();
}

/*
 * extractGroundFloor() is based on
 * "Zhang, Keqi, et al. "A progressive morphological filter for removing nonground measurements from airborne LIDAR data."
 * IEEE transactions on geoscience and remote sensing 41.4 (2003): 872-882."
 *
 * */
pcl::PointCloud<pcl::PointXYZ>::Ptr
ground_floor_finder::extractGroundFloor(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, bool vis) {

    //std::cout << "[GFF] Start ..." << std::endl;
    this->start = std::chrono::high_resolution_clock::now();
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_voxel(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ground(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_object(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointIndicesPtr ground(new pcl::PointIndices);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_rgb(new pcl::PointCloud<pcl::PointXYZRGB>);

    pcl::copyPointCloud(*cloud_in, *cloud);

    // std::cerr << "Cloud before filtering: " << cloud->size() << std::endl;
    pcl::VoxelGrid<pcl::PointXYZ> sor;
    sor.setInputCloud(cloud);
    sor.setLeafSize(0.15f, 0.15f, 0.15f);
    sor.filter(*cloud_voxel);
    // std::cerr << "Cloud after filtering: " << cloud_voxel->size() << std::endl;

    // Create the filtering object
    pcl::ProgressiveMorphologicalFilter<pcl::PointXYZ> pmf;
    pmf.setInputCloud(cloud_voxel);
    pmf.setMaxWindowSize(20);
    pmf.setSlope(0.3f);
    pmf.setInitialDistance(0.2f);
    pmf.setMaxDistance(1.0f);
    pmf.extract(ground->indices);

    // Create the filtering object
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(cloud_voxel);
    extract.setIndices(ground);
    extract.filter(*cloud_ground);

    // std::cerr << "Ground cloud after filtering: " << cloud_ground->size() << std::endl;

    this->end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
    //std::cout << "[GFF] Finished in " << duration << "ms" << std::endl;

    // Store cloud_p in separate cloud with distinct color.
    if (vis) {
        util->concatClouds(cloud_rgb, cloud_ground, util->getColor(1) /*1 -> green*/);
        // Extract non-ground returns
        extract.setNegative(true);
        extract.filter(*cloud_object);
        // std::cerr << "Object cloud after filtering: " << cloud_object->size() << std::endl;
        util->concatClouds(cloud_rgb, cloud_object, util->getColor(0 /*0 -> red*/));
        util->displayCloud(cloud_rgb, "Ground");
    }
    return cloud_ground;
}

/*####################################################################################################################*/
/*################################################## Some old Code ###################################################*/

void ground_floor_finder::extractGroundFloorBasedOnSegmentation(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in) {

    pcl::PCLPointCloud2::Ptr cloud_blob(new pcl::PCLPointCloud2), cloud_filtered_blob(new pcl::PCLPointCloud2);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>),
            cloud_p(new pcl::PointCloud<pcl::PointXYZ>),
            cloud_f(new pcl::PointCloud<pcl::PointXYZ>);

    // Fill in the cloud data
    //std::string input_file = "/home/markus/git/ros/fallen_cws/src/fallen_person_detector/misc/table_scene.pcd";
    // pcl::PCDReader reader;
    // reader.read (input_file, *cloud_blob);

    pcl::toPCLPointCloud2(*cloud_in, *cloud_blob);
    //pcl::copyPointCloud(*cloud_blob, *cloud_rgb);
    //cloud_rgb = cloud_blob.get();

    /*
    pcl::PointCloud<pcl::PointXYZ>::Ptr ptCldXYZ(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr ptCldXYZRGB(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::copyPointCloud(*ptCldXYZ,*ptCldXYZRGB);
    */
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_rgb(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::fromPCLPointCloud2(*cloud_blob, *cloud_rgb);
    /*
    colorCloud(cloud_rgb, this->util_color.at(ct));
    displayCloud(cloud_rgb, "RGB Version");
    */
    std::cerr << "PointCloud before filtering: " << cloud_blob->width * cloud_blob->height << " data points."
              << std::endl;

    // Create the filtering object: downsample the dataset using a leaf size of 1cm
    pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
    sor.setInputCloud(cloud_blob);
    sor.setLeafSize(0.05f, 0.05f, 0.05f);
    sor.filter(*cloud_filtered_blob);

    // Convert to the templated PointCloud
    pcl::fromPCLPointCloud2(*cloud_filtered_blob, *cloud_filtered);

    std::cerr << "PointCloud after filtering: " << cloud_filtered->width * cloud_filtered->height << " data points."
              << std::endl;

    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    // Optional
    seg.setOptimizeCoefficients(true);
    // Mandatory
    seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(1000);
    seg.setDistanceThreshold(0.09);

    // Create the filtering object
    pcl::ExtractIndices<pcl::PointXYZ> extract;

    int i = 0, nr_points = (int) cloud_filtered->points.size();
    // While 30% of the original cloud is still there
    while (cloud_filtered->points.size() > 0.3 * nr_points) {
        // Segment the largest planar component from the remaining cloud
        seg.setInputCloud(cloud_filtered);
        seg.segment(*inliers, *coefficients);
        if (inliers->indices.size() == 0) {
            std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
            break;
        }

        // Extract the inliers
        extract.setInputCloud(cloud_filtered);
        extract.setIndices(inliers);
        extract.setNegative(false);
        extract.filter(*cloud_p);
        std::cerr << "PointCloud representing the planar component: " << cloud_p->width * cloud_p->height
                  << " data points." << std::endl;

        // Store cloud_p in separate cloud with distinct color.
        util->concatClouds(cloud_rgb, cloud_p, util->getColor(rand() % 9));

        // Create the filtering object
        extract.setNegative(true);
        extract.filter(*cloud_f);
        cloud_filtered.swap(cloud_f);
        i++;
    }
    util->displayCloud(cloud_rgb, "Result");
}

pcl::PointCloud<pcl::PointXYZ>::Ptr &ground_floor_finder::calculatePlane(
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in) {

    //pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    //cloud = cloud_in;
    //cloud = util->generateRandomCloud(cloud, 10000);

    std::cerr << "Point cloud data: " << cloud_in->points.size() << " points" << std::endl;

    //printCloud(cloud);

    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    // Optional
    seg.setOptimizeCoefficients(true);
    // Mandatory
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(1000);
    seg.setDistanceThreshold(0.01);

    seg.setInputCloud(cloud_in);
    seg.segment(*inliers, *coefficients);

    if (inliers->indices.size() == 0) {
        PCL_ERROR ("Could not estimate a planar model for the given dataset.");
        return cloud_in;
    }

    std::cerr << "Model coefficients: " << coefficients->values[0] << " "
              << coefficients->values[1] << " "
              << coefficients->values[2] << " "
              << coefficients->values[3] << std::endl;

    std::cerr << "Model inliers: " << inliers->indices.size() << std::endl;

    //printIndices(cloud, inliers);

    util->displayCloud(cloud_in, "Segmentation result");

    return cloud_in;
}