/******************************************************************************
Copyright (c) 2016, Intel Corporation
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice,
this list of conditions and the following disclaimer in the documentation
and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors
may be used to endorse or promote products derived from this software without
specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*******************************************************************************/


#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <image_transport/image_transport.h>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>

#include <pcl-1.7/pcl/conversions.h>
#include <pcl-1.7/pcl/point_cloud.h>
#include <pcl-1.7/pcl/point_types.h>
#include <pcl-1.7/pcl/PCLPointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/PointStamped.h>

#include <visualization_msgs/Marker.h>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32MultiArray.h"
#include <vector>

#include "follower_nodelet.h"

//Nodelet dependencies
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(realsense_depth_follower::DepthFollowerNodelet, nodelet::Nodelet)

namespace realsense_depth_follower
{

	//******************************
	// Public Methods
	//******************************

	DepthFollowerNodelet::~DepthFollowerNodelet()
	{
		ROS_INFO_STREAM("Done - DepthFollowerNodelet");
	}

	void DepthFollowerNodelet::onInit()
	{
		ROS_INFO_STREAM("Starting Follower node");

		fillConfigMap();

		ros::NodeHandle& nh = getNodeHandle();

		//subscribe
		mSub = nh.subscribe("camera/depth/points", 1, &DepthFollowerNodelet::pointcloudCallback, this);

		//advertise
		mMarkerPub = nh.advertise<visualization_msgs::Marker>("depth_follower/marker",1);
		mGoalPub = nh.advertise<geometry_msgs::PointStamped>("depth_follower/goal", 1);

		// Initialize dynamic reconfigure
		mReconfigureServer.reset(new dynamic_reconfigure::Server<realsense_depth_follower::depth_followerConfig>(getPrivateNodeHandle()));
		mReconfigureServer->setCallback(boost::bind(&DepthFollowerNodelet::ConfigureCallback, this, _1, _2));

	}

	//===================================
	//	Member Functions
	//===================================

	void DepthFollowerNodelet::fillConfigMap()
	{
		std::vector<std::string> args = getMyArgv();
		while (args.size() > 1)
		{
			mConfig[args[0]] = args[1];
			args.erase(args.begin());
			args.erase(args.begin());
		}


		mEnabled = true;
		mMinY = 0;
		mMaxY=20000;
		mMinX=0;
		mMaxX=20000;
		mMinZ=0;
		mMaxZ=20000;
		mMinBlobSize = 4000;

		char* key = (char*)"MinY";
		if (mConfig.find(key) != mConfig.end())
		{
			ROS_INFO("Setting %s to %s", key, mConfig.at(key).c_str());
			mMinY = atof(mConfig.at(key).c_str());
		}

		key = (char*)"MaxY";
		if (mConfig.find(key) != mConfig.end())
		{
			ROS_INFO("Setting %s to %s", key, mConfig.at(key).c_str());
			mMaxY = atof(mConfig.at(key).c_str());
		}

		key = (char*)"MinX";
		if (mConfig.find(key) != mConfig.end())
		{
			ROS_INFO("Setting %s to %s", key, mConfig.at(key).c_str());
			mMinX = atof(mConfig.at(key).c_str());
		}

		key = (char*)"MaxX";
		if (mConfig.find(key) != mConfig.end())
		{
			ROS_INFO("Setting %s to %s", key, mConfig.at(key).c_str());
			mMaxX = atof(mConfig.at(key).c_str());
		}

		key = (char*)"MaxZ";
		if (mConfig.find(key) != mConfig.end())
		{
			ROS_INFO("Setting %s to %s", key, mConfig.at(key).c_str());
			mMaxZ = atof(mConfig.at(key).c_str());
		}

		key = (char*)"MinZ";
		if (mConfig.find(key) != mConfig.end())
		{
			ROS_INFO("Setting %s to %s", key, mConfig.at(key).c_str());
			mMinZ = atof(mConfig.at(key).c_str());
		}

		key = (char*)"MinBlobSize";
		if (mConfig.find(key) != mConfig.end())
		{
			ROS_INFO("Setting %s to %s", key, mConfig.at(key).c_str());
			mMinBlobSize = atof(mConfig.at(key).c_str());
		}



	}

	pcl::PointXYZ DepthFollowerNodelet::getCentroidFromPointCloud(const pcl::PointCloud<pcl::PointXYZ>*  cloud)
	{
		pcl::PointXYZ point;
		point.x = point.y = point.z = -1;

		//X,Y,Z of the centroid
		float x = 0.0;
		float y = 0.0;
		float z = 0.0;//1e6;

		//Number of points observed
		unsigned int n = 0;
		//Iterate through all the points in the region and find the average of the position
		BOOST_FOREACH (const pcl::PointXYZ& pt, cloud->points)
		{
		  if (!std::isnan(x) && !std::isnan(y) && !std::isnan(z))
		  {
			//Test to ensure the point is within the acceptable box.
			if (-pt.y > mMinY   && -pt.y < mMaxY  && pt.x < mMaxX && pt.x > mMinX && pt.z < mMaxZ&& pt.z > mMinZ)
			{
			  //Add the point to the totals
			  x += pt.x;
			  y += pt.y;
			  z += pt.z;//= std::min(z, pt.z);
			  n++;
			}
		  }
					}
		//If there are enough points, find the centroid and calculate the command goal.
		//If there are no points, return default point
		if (n>mMinBlobSize)
		{
			point.x = x/n;
			point.y = y/n;
			point.z = z/n;
		}

		return point;
	}

	void DepthFollowerNodelet::process(const pcl::PointCloud<pcl::PointXYZ>*  cloud)
	{

		//X,Y,Z of the centroid
		float x = 0.0;
		float y = 0.0;
		float z = 0;

		pcl::PointXYZ point = getCentroidFromPointCloud(cloud);

		//if the point is not valid
		if (point.x != -1 || point.y !=-1 || point.z != -1)
		{
			x = point.x;
			y = point.y;
			z = point.z;

			publishMarker(x, y, z);
			publishGoal(x, y, z);
		}

	  }


	//***********************************
	// Debug/Visualize functions
	//***********************************

	void DepthFollowerNodelet::publishGoal(double x, double y, double z)
	{
		geometry_msgs::PointStamped goal;
		goal.header.frame_id = mFrame;
		goal.header.stamp = ros::Time();
		goal.point.x = x;
		goal.point.y = y;
		goal.point.z = z;
		mGoalPub.publish(goal);
	}

	void DepthFollowerNodelet::publishMarker(double x,double y,double z)
	  {
	    visualization_msgs::Marker marker;
	    marker.header.frame_id = mFrame;
	    marker.header.stamp = ros::Time();
	    marker.ns = "my_namespace";
	    marker.id = 0;
	    marker.type = visualization_msgs::Marker::SPHERE;
	    marker.action = visualization_msgs::Marker::ADD;
	    marker.pose.position.x = x;
	    marker.pose.position.y = y;
	    marker.pose.position.z = z;
	    marker.pose.orientation.x = 0.0;
	    marker.pose.orientation.y = 0.0;
	    marker.pose.orientation.z = 0.0;
	    marker.pose.orientation.w = 1.0;
	    marker.scale.x = 0.1;
	    marker.scale.y = 0.1;
	    marker.scale.z = 0.1;
	    marker.color.a = 1.0;
	    marker.color.r = 1.0;
	    marker.color.g = 0.0;
	    marker.color.b = 0.0;
	    mMarkerPub.publish( marker );

	 }

	//***********************************
	// Callback functions
	//***********************************

	void DepthFollowerNodelet::pointcloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg_pointcloud)
	{
		if (mEnabled && (mMarkerPub.getNumSubscribers() > 0 || mGoalPub.getNumSubscribers() > 0))
		{
			pcl::PointCloud<pcl::PointXYZ> cloud;
			pcl::fromROSMsg(*msg_pointcloud, cloud);
			int size = cloud.points.size();
			mFrame = msg_pointcloud->header.frame_id;
			process(&cloud);
		}
	}

	void DepthFollowerNodelet::ConfigureCallback(realsense_depth_follower::depth_followerConfig &config, uint32_t level)
	{
		mMinY = config.mMinY;
		mMaxY = config.mMaxY;
		mMinX = config.mMinX;
		mMaxX = config.mMaxX;
		mMinZ = config.mMinZ;
		mMaxZ = config.mMaxZ;


		mMinBlobSize = config.mMinBlobSize;

		mEnabled = config.mEnabled;

	}

	//***********************************
	// Utils functions
	//***********************************

	bool DepthFollowerNodelet::to_bool(std::string str)
	{
		std::transform(str.begin(), str.end(), str.begin(), ::tolower);
		std::istringstream is(str);

		bool b;

		is >> std::boolalpha >> b;

		return b;
	}

}

