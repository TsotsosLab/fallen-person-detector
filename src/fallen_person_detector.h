//
// Created by markus d. solbach
// solbach@eecs.yorku.ca
//
#ifndef FALLEN_PERSON_DETECTOR_FALLEN_PERSON_DETECTOR_H
#define FALLEN_PERSON_DETECTOR_FALLEN_PERSON_DETECTOR_H

// include everything we need
#include "ground_floor_finder.h"
#include <openpose_ros/Person.h>

#include <ros/node_handle.h>
#include <ros/service_client.h>
#include <ros/init.h>
#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include "sensor_msgs/PointCloud2.h"

#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <std_srvs/Empty.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>


#endif //FALLEN_PERSON_DETECTOR_FALLEN_PERSON_DETECTOR_H
