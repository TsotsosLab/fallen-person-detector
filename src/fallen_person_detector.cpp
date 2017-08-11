//
// Created by markus d. solbach
// solbach@eecs.yorku.ca
//
#include "fallen_person_detector.h"

ros::ServiceClient srv_client;
openpose_ros::Person srv;
image_transport::Publisher openposeImagePub;
image_transport::Publisher openposeDepthPub;
image_transport::Publisher fallenVisPub;
image_transport::Publisher fallenDetectPub;
ros::Publisher groundPub;
ros::Publisher detectionPub;
ros::Publisher origCloudPub;

ground_floor_finder *gff;
pcl_util *util;

int count = 0;
bool camParamSet;

//     [fx  0 cx]
// K = [ 0 fy cy]
//     [ 0  0  1]
cv::Mat K;
std::chrono::high_resolution_clock::time_point start, end;

void printDetections(openpose_ros::PersonResponse_<std::allocator<void>>::_detections_type detections) {
    for (int i = 0; i < detections.size(); ++i) {
        ROS_INFO("Detection: %d, Confidence: %f", i, srv.response.detections[i].avgConfidence);
    }
}

cv::Mat blendRecognitionDepth(cv::Mat mat, cv::Mat cvMat);

void showImage(std::string name, cv::Mat image) {
    cv::imshow(name, image);
    cv::waitKey(1);
}

cv::Mat rosToCvMat(const sensor_msgs::ImageConstPtr &data) {
    cv::Mat result;
    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(data, data->encoding);
    }
    catch (cv_bridge::Exception &e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return result;
    }
    result = cv_ptr->image;
    return result;
}

cv::Mat rosToCvMat(openpose_ros::PersonResponse_<std::allocator<void> >::_detection_img_type &data) {
    cv::Mat result;
    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(data, data.encoding);
    }
    catch (cv_bridge::Exception &e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return result;
    }
    result = cv_ptr->image;
    return result;
}

void showIllustration(std::string name, const cv::Mat &depth, const cv::Mat &rgb) {
    double min;
    double max;
    cv::minMaxIdx(depth, &min, &max);
    //ROS_INFO("Min/Max: %f/%f", min, max);

    if (min < 0)
        min = 0;
    if (max > 255)
        max = 255;

    cv::Mat adjMap;
    depth.convertTo(adjMap, CV_8UC1, 255 / (max - min), -min);

    cv::Mat falseColorsMap;
    applyColorMap(adjMap, falseColorsMap, cv::COLORMAP_JET);

    // Alpha Blending
    double alpha = 0.5;
    double beta;
    beta = (1.0 - alpha);
    cv::Mat illustration;
    cv::addWeighted(falseColorsMap, alpha, rgb, beta, 0.0, illustration);

    cv::imshow(name, illustration);
    cv::waitKey(1);
}

pcl::PointCloud<pcl::PointXYZ>::Ptr
calculateWorldCoordinates(openpose_ros::PersonDetection_<std::allocator<void>>::_bodyparts_type bodyparts,
                          cv::Mat depth) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_result(new pcl::PointCloud<pcl::PointXYZ>);
    double xW, yW, zW;
    /*
    std::cout << bodyparts[1].name << ": \t " << bodyparts[1].confidence << " -- " << bodyparts[1].x << ","
              << bodyparts[1].y << " | " << depth.at<float>(bodyparts[1].y, bodyparts[1].x) << std::endl;
    */
    pcl::PointXYZ pt;
    int partCount = 0;
    // Iterate over all Bodyparts and exclude Parts below Threshold
    for (int i = 0; i < bodyparts.size(); ++i) {
        if (bodyparts[i].confidence > 0.08) {
            partCount++;
            zW = depth.at<float>(bodyparts[i].y, bodyparts[i].x);
            xW = (bodyparts[i].x - K.at<float>(0, 2)) * zW / K.at<float>(0, 0);
            yW = (bodyparts[i].y - K.at<float>(1, 2)) * zW / K.at<float>(1, 1);

            // Use this if you publish under "zed_depth_frame" as tf
            /*
            pt.x = xW;
            pt.y = yW;
            pt.z = zW;
            */

            // Use this if you publish under "ZED_left_camera" as tf
            // This is also used for topic "/zed/point_cloud/cloud_registered"
            pt.x = zW;
            pt.y = -xW;
            pt.z = -yW;

            cloud_result->push_back(pt);
            /*
            std::cout << bodyparts[i].name << ": \t " << bodyparts[i].confidence << " -- " << bodyparts[i].x << ","
                      << bodyparts[i].y << " | " << pt.x << ", " << pt.y << ", " << pt.z << std::endl;
            */
        }
    }
    ROS_INFO("%d valid Bodyparts detected", partCount);
    return cloud_result;
}

cv::Mat blendRecognitionWithDepth(const cv::Mat &depth, const cv::Mat &rgb) {
    double min;
    double max;
    cv::minMaxIdx(depth, &min, &max);
    //ROS_WARN("Min/Max: %f/%f", min, max);

    if (min < 0)
        min = 0;
    if (max > 255)
        max = 255;

    cv::Mat adjMap;
    depth.convertTo(adjMap, CV_8UC1, 255 / (max - min), -min);

    cv::Mat falseColorsMap;
    applyColorMap(adjMap, falseColorsMap, cv::COLORMAP_JET);

    // Alpha Blending
    double alpha = 0.5;
    double beta;
    beta = (1.0 - alpha);
    cv::Mat illustration;
    cv::addWeighted(falseColorsMap, alpha, rgb, beta, 0.0, illustration);

    return illustration;
}

void publishGroundCloudRGB(pcl::PointCloud<pcl::PointXYZRGB> cloud_in) {
    // Make sure to use right transformation in method calculateWorldCoordinates
    cloud_in.header.frame_id = "ZED_left_camera";
    //cloud_in.header.frame_id =  "zed_depth_frame"; x = -y, y = -z, z = x
    cloud_in.header.stamp = ros::Time::now().toNSec();
    groundPub.publish(cloud_in);
}

void publishDetectionCloudRGB(pcl::PointCloud<pcl::PointXYZRGB> cloud_in) {
    // Make sure to use right transformation in method calculateWorldCoordinates
    cloud_in.header.frame_id = "ZED_left_camera";
    //cloud_in.header.frame_id =  "zed_depth_frame"; x = -y, y = -z, z = x
    cloud_in.header.stamp = ros::Time::now().toNSec();
    detectionPub.publish(cloud_in);
}

void publishCloudXYZ(const sensor_msgs::PointCloud2ConstPtr &cloud_in) {
    origCloudPub.publish(cloud_in);
}

bool checkIfUbC(std::string input) {
    if (input == "Neck") {
        return true;
    } else if (input == "REar") {
        return true;
    } else if (input == "LEar") {
        return true;
    } else if (input == "REye") {
        return true;
    } else if (input == "LEye") {
        return true;
    } else if (input == "Nose") {
        return true;
    } else if (input == "RShoulder") {
        return true;
    } else if (input == "LShoulder") {
        return true;
    } else {
        return false;
    }
}

float
checkForUpperBodyCriticals(openpose_ros::PersonDetection_<std::allocator<void>>::_bodyparts_type bodyparts,
                           cv::Mat depth, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ground) {
    float result = 2.0;
    // Calculate 3D World Coordinate of Upper Body Criticals exclusively
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ubc(new pcl::PointCloud<pcl::PointXYZ>);
    double xW, yW, zW;
    pcl::PointXYZ pt;
    int partCount = 0;
    // Iterate over all Bodyparts and exclude Parts below Threshold
    for (int i = 0; i < bodyparts.size(); ++i) {
        if (bodyparts[i].confidence > 0.05 && checkIfUbC(bodyparts[i].name)) {
            partCount++;
            zW = depth.at<float>(bodyparts[i].y, bodyparts[i].x);
            xW = (bodyparts[i].x - K.at<float>(0, 2)) * zW / K.at<float>(0, 0);
            yW = (bodyparts[i].y - K.at<float>(1, 2)) * zW / K.at<float>(1, 1);

            // Use this if you publish under "zed_depth_frame" as tf
            /*
            pt.x = xW;
            pt.y = yW;
            pt.z = zW;
            */

            // Use this if you publish under "ZED_left_camera" as tf
            // This is also used for topic "/zed/point_cloud/cloud_registered"
            pt.x = zW;
            pt.y = -xW;
            pt.z = -yW;

            cloud_ubc->push_back(pt);
            /*
            std::cout << bodyparts[i].name << ": \t " << bodyparts[i].confidence << " -- " << bodyparts[i].x << ","
                      << bodyparts[i].y << " | " << pt.x << ", " << pt.y << ", " << pt.z << std::endl;
            */
        }
    }

    pcl::_PointXYZRGB centroid;
    centroid = util->calculateCentroid(cloud_ubc, util->getColor(4));
    float shortestDistance;
    shortestDistance = util->calculateShortestDistance(cloud_ground, centroid);

    return shortestDistance;
}

cv::Mat overlayDetection(const cv::Mat &rgb,
                         openpose_ros::PersonDetection_<std::allocator<void>>::_bodyparts_type bparts, bool cog,
                         bool ubc) {
    // Determine color to draw (BGR)
    cv::Scalar color = cv::Scalar(0, 255, 0); // green
    if(ubc & cog){
        color = cv::Scalar(0, 0, 255); // red
    }
    else if(ubc){
        color = cv::Scalar(51, 153, 255); // orange
    }
    else if(cog){
        color = cv::Scalar(51, 255, 255); // yellow
    }
    cv::Point min_pt, max_pt;
    cv::Mat result_image = rgb;
    // Find min/max and draw bounding box
    int x_min = 10000, y_min= 10000, x_max = 0, y_max = 0;
    for (int i = 0; i < bparts.size(); ++i) {
        if(bparts[i].confidence < 0.3)
            continue;

        if(bparts[i].x < x_min){
            x_min = bparts[i].x;
        }
        if(bparts[i].x > x_max){
            x_max = bparts[i].x;
        }
        if(bparts[i].y < y_min){
            y_min = bparts[i].y;
        }
        if(bparts[i].y > y_max){
            y_max = bparts[i].y;
        }
    }
    min_pt.x = x_min;
    min_pt.y = y_min;
    max_pt.x = x_max;
    max_pt.y = y_max;
    cv::rectangle(result_image, min_pt, max_pt, color, 7);

    return result_image;
}

/* Subscribe to ZED stereocamera stream (RGB & Depth) */
void sensingCallback(const sensor_msgs::ImageConstPtr &rgb, const sensor_msgs::ImageConstPtr &depth,
                     const sensor_msgs::PointCloud2ConstPtr &cloud, const sensor_msgs::CameraInfoConstPtr &depth_info) {
    start = std::chrono::high_resolution_clock::now();
    bool cog = false; // Center of Gravity | true if it is below a certain threshold
    bool ubc = false; // Upper Body criticals | true if nose, ears, eyes, shoulders below a certain threshold

    ROS_INFO("#######################################");
    // Set Intrinsic Camera Parameters
    if (!camParamSet) {
        K.at<float>(0, 0) = depth_info->K[0]; // fx
        K.at<float>(0, 2) = depth_info->K[2]; // cx
        K.at<float>(1, 1) = depth_info->K[4]; // fy
        K.at<float>(1, 2) = depth_info->K[5]; // cy
        K.at<float>(2, 2) = 1.0;
        camParamSet = true;
    }
    if (count < -1) {
        count++;
        return;
    }
    count = 0;
    ROS_INFO("[service call] openpose_ros");

    srv.request.image = *rgb;

    if (!srv_client.call(srv)) {
        ROS_ERROR("[service call] failed");
        return;
    }

    // Setting up some variables
    cv::Mat result_img = rosToCvMat(rgb);
    cv::Mat vis_img = rosToCvMat(depth);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudGroundRGB(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudDetectionRGB(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudGround(new pcl::PointCloud<pcl::PointXYZ>);

    ROS_INFO("[service call] Detection(s) %d", srv.response.detections.size());
    if(srv.response.detections.size() < 1){
        return;
    }
    bool publish = false;
    bool floorProcessed = false;
// #################################################################################################################
// Main loop - iterating over all detections
// #################################################################################################################
    for (int i = 0; i < srv.response.detections.size() ; ++i) {
        ROS_INFO("#### Processing Detection %d", i+1);

        // Check for average confidence
        if (srv.response.detections[i].avgConfidence < 0.35) {
            ROS_WARN("Average confidence too low");
            continue;
        }
        // Calculate Depth of Pose Keypoints and generate point cloud
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloudDetection(new pcl::PointCloud<pcl::PointXYZ>);
        cloudDetection = calculateWorldCoordinates(srv.response.detections[i].bodyparts, rosToCvMat(depth));
        if (cloudDetection->size() < 4) {
            ROS_WARN("Not enough valid bodyparts. return.");
            continue;
        }
        publish = true;
        // Estimate convex hull to get rid of outliers
        std::vector<double> cloud_stats;
        cloud_stats = util->calculateAreaVolume(cloudDetection);
        double vol_min = 0.05;
        double are_min = 1.0; // BSA (Body Surface Area)
        // normalize it with number of valid bodypart detections
        vol_min /= cloudDetection->size();
        are_min /= cloudDetection->size();
        // cloud_stats.at(0) = area in m^2 | cloud_stats.at(1) = volume in m^3
        if ((cloud_stats.at(0) < are_min) || (cloud_stats.at(1) < vol_min)) {
            ROS_WARN("Area %f or Volume %f of detection not sufficient. return.", cloud_stats.at(0), cloud_stats.at(1));
            continue;
        }

        // Only process the ground floor
        if(!floorProcessed) {
            // Detect ground floor
            pcl::fromROSMsg(*cloud, *cloudGround);
            cloudGround = gff->extractGroundFloor(cloudGround, false);
            pcl::copyPointCloud(*cloudGround, *cloudGroundRGB);
            util->colorCloud(cloudGroundRGB, util->getColor(0));
            floorProcessed = true;
        }
        // Transform ground and detection cloud to RGB clouds
        pcl::copyPointCloud(*cloudDetection, *cloudDetectionRGB);
        util->colorCloud(cloudDetectionRGB, util->getColor(1));
        float shortestDistance;
// #####################################################################################################################
// Metric ONE: Determine if center of gravity (CoG) is close to ground plane
// #####################################################################################################################
        //Calculate centroid and add to projected Cloud for illustration
        pcl::_PointXYZRGB centroid;
        centroid = util->calculateCentroidWAugmentation(cloudDetectionRGB, cloudDetection, util->getColor(4));
        shortestDistance = util->calculateShortestDistance(cloudGround, centroid);
        ROS_INFO("CoG distance to ground: %fm", shortestDistance);
        if (shortestDistance < 0.3) { // if CoG closer than 0.3m to the ground we have detected a fallen person
            cog = true;
            ROS_ERROR("!!!!!! Center of Gravity Critical !!!!!!");
            ROS_ERROR("!!!!!!!! FALLEN PERSON DETECTED !!!!!!!!");
        }
// #####################################################################################################################
// Metric TWO: Determine if upper body critical (UbC) are close to ground
// #####################################################################################################################
        // UbC: Eyes, Nose, Ears, Neck, Shoulders
        shortestDistance = checkForUpperBodyCriticals(srv.response.detections[0].bodyparts, rosToCvMat(depth),
                                                      cloudGround);
        ROS_INFO("UbC distance to ground: %fm", shortestDistance);
        if (shortestDistance < 0.45) { // if UbC closer than 0.9m to the ground we have detected a fallen person
            ubc = true;
            ROS_ERROR("!!!!!!!!! Upper Body Critical !!!!!!!!!!");
            ROS_ERROR("!!!!!!!! FALLEN PERSON DETECTED !!!!!!!!");
        }

        // Prepare results for publication
        // Input RGB image overlaid with green for 'OK', red for 'fallen', orange for 'uncertain'
        result_img = overlayDetection(result_img, srv.response.detections[i].bodyparts, cog, ubc);
        vis_img = blendRecognitionWithDepth(vis_img, rosToCvMat(srv.response.detection_img));

        // Show Images for Debugging
        //showIllustration("Recognition + Depth Blending", rosToCvMat(depth), rosToCvMat(srv.response.detection_img));
        //showImage("Depth", rosToCvMat(depth));
        //showImage("OpenPose", rosToCvMat(srv.response.detection_img));
    }

    // Make sure there is something to publish, otherwise skip
    if(publish) {
        // Publishing Clouds
        publishDetectionCloudRGB(*cloudDetectionRGB); // publish result cloud containing ground floor and detections
        publishGroundCloudRGB(*cloudGroundRGB); // publish result cloud containing ground floor and detections
        publishCloudXYZ(cloud); // publish original cloud from ZED
        //util->displayCloud(cloudProject, "Projected Pose");
        sensor_msgs::Image visMsg = *cv_bridge::CvImage(std_msgs::Header(), "bgr8", vis_img).toImageMsg();
        sensor_msgs::Image detectMsg = *cv_bridge::CvImage(std_msgs::Header(), "bgr8", result_img).toImageMsg();
        fallenVisPub.publish(visMsg);
        fallenDetectPub.publish(detectMsg);
        openposeImagePub.publish(srv.response.detection_img);
        openposeDepthPub.publish(depth);
    }
    end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
    ROS_INFO("Processing time: %dms", duration);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "fallen_person_detector");
    ros::NodeHandle n;
    camParamSet = false;
    K = cv::Mat(cv::Size(3, 3), CV_32F); // Initialize K to be a 3 x 3 float matrix
    K.setTo(cv::Scalar(0.0));

    ROS_INFO("Ready to call openpose_ros.");

    /* RCNN Service */
    srv_client = n.serviceClient<openpose_ros::Person>("detect_poses");
    image_transport::ImageTransport it(n);
    message_filters::Subscriber<sensor_msgs::Image> rgb_sub(n, "/zed/left/image_raw_color", 1);
    message_filters::Subscriber<sensor_msgs::Image> depth_sub(n, "zed/depth/depth_registered", 1);
    message_filters::Subscriber<sensor_msgs::PointCloud2> cloud_sub(n, "/zed/point_cloud/cloud_registered", 1);
    message_filters::Subscriber<sensor_msgs::CameraInfo> depth_info_sub(n, "/zed/depth/camera_info", 1);
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image,
            sensor_msgs::PointCloud2, sensor_msgs::CameraInfo> MySyncPolicy;
    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), rgb_sub, depth_sub, cloud_sub, depth_info_sub);
    sync.registerCallback(boost::bind(&sensingCallback, _1, _2, _3, _4));

    // For advertising Images
    openposeImagePub = it.advertise("/fallen/PoseImage", 1); // Just the Pose in an RGB image (straight from openpose)
    openposeDepthPub = it.advertise("/fallen/DepthImage", 1); // The use depth image as used for 3D reconstruction
    fallenVisPub = it.advertise("/fallen/VisImage", 1); // Combination of Pose image with depth image
    fallenDetectPub = it.advertise("/fallen/DetectionImage", 1); // Result Image: green overlay: OK; red overlay: fallen
    // For advertising Clouds
    groundPub = n.advertise<pcl::PointCloud<pcl::PointXYZRGB>>("/fallen/GroundCloudRGB", 1);
    detectionPub = n.advertise<pcl::PointCloud<pcl::PointXYZRGB>>("/fallen/DetectionCloudRGB", 1);
    origCloudPub = n.advertise<pcl::PointCloud<pcl::PointXYZRGB>>("/fallen/OrigCloudRGB", 1);

    gff = new ground_floor_finder();
    util = new pcl_util(); // some point cloud utility functions

    ros::spin();

    return 0;
}