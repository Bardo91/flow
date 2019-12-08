//---------------------------------------------------------------------------------------------------------------------
//  flow
//---------------------------------------------------------------------------------------------------------------------
//  Copyright 2019 Pablo Ramon Soria (a.k.a. Bardo91) pabramsor@gmail.com
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

#ifndef FLOW_BLOCKS_PUBLISHERS_ROS_ROSPUBLISHERS_H_
#define FLOW_BLOCKS_PUBLISHERS_ROS_ROSPUBLISHERS_H_

#include <flow/visual/blocks/publishers/ros/BlockROSPublisher.h>
#include <Eigen/Eigen>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <opencv2/opencv.hpp>

#ifdef FLOW_USE_ROS
	#include <tf2_eigen/tf2_eigen.h>
	#include <geometry_msgs/PoseStamped.h>
	#include <sensor_msgs/Image.h>
	#include <pcl_conversions/pcl_conversions.h>
	#include <cv_bridge/cv_bridge.h>
#endif

namespace flow{
	#ifdef FLOW_USE_ROS
		template<typename _T>
		struct TraitRosPublisher{
	    	static std::string blockName_;
	    	static std::map<std::string, std::string> input_;
	    	static std::any conversion_(DataFlow _data);
	    	typedef _T ROSType_;
		};

		typedef TraitRosPublisher<geometry_msgs::PoseStamped> TraitPoseStampedPublisher;
		typedef TraitRosPublisher<sensor_msgs::PointCloud2> TraitPointCloudPublisher;
		typedef TraitRosPublisher<sensor_msgs::Image> TraitImagePublisher;

		typedef BlockROSPublisher< TraitPoseStampedPublisher > BlockROSPublisherPoseStamped;
		typedef BlockROSPublisher< TraitPointCloudPublisher  > BlockROSPublisherPointCloud;
		typedef BlockROSPublisher< TraitImagePublisher       > BlockROSPublisherImage;
    #endif
}
#endif