//---------------------------------------------------------------------------------------------------------------------
//  flow
//---------------------------------------------------------------------------------------------------------------------
//  Copyright 2018 Pablo Ramon Soria (a.k.a. Bardo91) pabramsor@gmail.com
//---------------------------------------------------------------------------------------------------------------------
//  Permission is hereby granted, free of charge, to any person obtaining a copy of this software
//  and associated documentation files (the "Software"), to deal in the Software without restriction,
//  including without limitation the rights to use, copy, modify, merge, publish, distribute,
//  sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
//  furnished to do so, subject to the following conditions:
//
//  The above copyright notice and this permission notice shall be included in all copies or substantial
//  portions of the Software.
//
//  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING
//  BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
//  NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES
//  OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
//  CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
//---------------------------------------------------------------------------------------------------------------------

#include <flow/visual/blocks/publishers/ros/ROSPublishers.h>


namespace flow{

    // Declaration of Trait structs
	#ifdef FLOW_USE_ROS

        //-------------------------------------------------------------------------------------------------------------
        std::string TraitPoseStampedPublisher::blockName_ = "ROS Publisher Pose";
        std::pair<std::string, std::string> TraitPoseStampedPublisher::input_("Pose", "mat44");
        geometry_msgs::PoseStamped TraitPoseStampedPublisher::conversion_(DataFlow _data){

            auto pose = _data.get<Eigen::Matrix4f>("Pose");
            geometry_msgs::PoseStamped ROSpose;

            Eigen::Affine3d poseAffine;
            poseAffine.matrix() = pose.cast<double>();
            ROSpose.pose   = tf2::toMsg(poseAffine);
            ROSpose.header.stamp    = ros::Time::now();
            ROSpose.header.frame_id = "map"; 
            
            return ROSpose;
        }

        //-------------------------------------------------------------------------------------------------------------
        std::string TraitPointCloudPublisher::blockName_ = "ROS Publisher PointCloud";
        std::pair<std::string, std::string> TraitPointCloudPublisher::input_("Point Cloud", "cloud");
        sensor_msgs::PointCloud2 TraitPointCloudPublisher::conversion_(DataFlow _data){

            auto cloud = _data.get<pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr>("Cloud");
            sensor_msgs::PointCloud2 ROScloud;
            
    		pcl::toROSMsg(*cloud, ROScloud);
            ROScloud.header.stamp    = ros::Time::now();
            ROScloud.header.frame_id = "map"; 

            return ROScloud;
        }

    #endif
}