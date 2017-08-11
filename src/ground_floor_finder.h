//
// Created by markus d. solbach
// solbach@eecs.yorku.ca
//
#ifndef FALLEN_PERSON_DETECTOR_GROUND_FLOOR_FINDER_H
#define FALLEN_PERSON_DETECTOR_GROUND_FLOOR_FINDER_H

#include "pcl_util.h"

#include <chrono>


class ground_floor_finder {

public:
    ground_floor_finder(void);
    pcl::PointCloud<pcl::PointXYZ>::Ptr extractGroundFloor(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, bool vis);

private:
    void init();

    pcl_util *util;
    std::chrono::high_resolution_clock::time_point start, end;

/*####################################################################################################################*/
/*################################################## Some old Code ###################################################*/
    pcl::PointCloud<pcl::PointXYZ>::Ptr& calculatePlane(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in);
    void extractGroundFloorBasedOnSegmentation(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in);
};

#endif //FALLEN_PERSON_DETECTOR_GROUND_FLOOR_FINDER_H
