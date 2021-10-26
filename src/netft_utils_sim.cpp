
/*
Copyright (c) 2016, Los Alamos National Security, LLC
All rights reserved.
Copyright 2016. Los Alamos National Security, LLC. This software was produced under U.S. Government contract DE-AC52-06NA25396 for Los Alamos National Laboratory (LANL), which is operated by Los Alamos National Security, LLC for the U.S. Department of Energy. The U.S. Government has rights to use, reproduce, and distribute this software.  NEITHER THE GOVERNMENT NOR LOS ALAMOS NATIONAL SECURITY, LLC MAKES ANY WARRANTY, EXPRESS OR IMPLIED, OR ASSUMES ANY LIABILITY FOR THE USE OF THIS SOFTWARE.  If software is modified to produce derivative works, such modified software should be clearly marked, so as not to confuse it with the version available from LANL.

Additionally, redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer. 
2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution. 
3. Neither the name of Los Alamos National Security, LLC, Los Alamos National Laboratory, LANL, the U.S. Government, nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission. 

THIS SOFTWARE IS PROVIDED BY LOS ALAMOS NATIONAL SECURITY, LLC AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL LOS ALAMOS NATIONAL SECURITY, LLC OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

Author: Alex von Sternberg
*/

#include "ros/ros.h"
#include "geometry_msgs/WrenchStamped.h"
#include "netft_utils/StartSim.h"
#include "netft_utils/StopSim.h"
#include "tf/transform_listener.h"
#include <math.h>
#include <visualization_msgs/Marker.h>

/**
 * This program simulates the force torque data coming from a FT sensor
 */
std::string world_frame = "world";
std::string reference_frame = "yumi_base_link";
std::string ft_frame = "ftsensor_l";
std::string tool_tip_l_frame = "yumi_tool_tip_l";

geometry_msgs::WrenchStamped simWrench;        // Wrench containing the simulated data
bool toSim = false;                            // True if we should output simulated data
double ropeLength = 0.3;                       // minimal length of the rope in meters
double springConstant = 0.1;                   // SpringContant in Newton/Meter
double ropeWeight = 0.01;                      // Rope weight in kg

ros::Publisher marker_pub;

tf::StampedTransform transform;
geometry_msgs::PointStamped fixpoint_reference_frame;
geometry_msgs::PointStamped fixpoint_tool_tip_l_frame;
geometry_msgs::PointStamped fixpoint_ft_frame;

void setWrench(double x, double y, double z, double rx, double ry, double rz)
{
  simWrench.header.frame_id = ft_frame;
  simWrench.header.stamp = ros::Time::now();
  simWrench.wrench.force.x = x;
  simWrench.wrench.force.y = y;
  simWrench.wrench.force.z = z;
  simWrench.wrench.torque.x = rx;
  simWrench.wrench.torque.y = ry;
  simWrench.wrench.torque.z = rz;
}
                  
bool startSim(netft_utils::StartSim::Request &req, netft_utils::StartSim::Response &res)
{
  
  ropeLength = req.ropeLength;
  springConstant = req.springConstant;
  ropeWeight = req.ropeWeight;

  reference_frame = req.frame;
  fixpoint_reference_frame.header.frame_id = reference_frame;
  fixpoint_reference_frame.point.x = req.x;
  fixpoint_reference_frame.point.y = req.y;
  fixpoint_reference_frame.point.z = req.z;

  toSim = true;

  
  visualization_msgs::Marker marker;
  marker.header.frame_id = "/world";
  marker.header.stamp = ros::Time::now();
  marker.ns = "basic_shapes";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::SPHERE;
  marker.action = visualization_msgs::Marker::ADD;
  marker.scale.x = 0.02;
  marker.scale.y = 0.02;
  marker.scale.z = 0.02;
  marker.color.a = 1.0;
  marker.color.r = 0.0;
  marker.color.g = 1.0;
  marker.color.b = 0.0;
  marker.pose.position.x = req.x;
  marker.pose.position.y = req.y;
  marker.pose.position.z = req.z;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;

  marker.lifetime = ros::Duration();
  marker_pub.publish(marker);


  return true;
}  

bool stopSim(netft_utils::StopSim::Request &req, netft_utils::StopSim::Response &res)
{ 
  toSim = false;                
  return true;    
}  
    
int main(int argc, char **argv)
{                 
  //Node name     
  ros::init(argc, argv, "netft_utils_sim");
  
  //Access main ROS system
  ros::NodeHandle n;

  marker_pub = n.advertise<visualization_msgs::Marker>("groundTruth_marker", 1);
                  
  //Publish on the /netft_transformed_data topic. Queue up to 100000 data points
  ros::Publisher netft_data_pub = n.advertise<geometry_msgs::WrenchStamped>("netft_data", 100000);

  //Listen to the transfomation from the ft sensor to world frame.
  tf::TransformListener listener;
  
  //Advertise bias and threshold services
  ros::ServiceServer start_service = n.advertiseService("start_sim", startSim);
  ros::ServiceServer stop_service = n.advertiseService("stop_sim", stopSim);
  
  ros::Rate loop_rate(400);

  //Initialize variables
  fixpoint_reference_frame.header.frame_id = reference_frame;
  fixpoint_reference_frame.header.stamp  = ros::Time(0); //ros::Time::now();
  fixpoint_reference_frame.point.x = 0.0;
  fixpoint_reference_frame.point.y = 0.0;
  fixpoint_reference_frame.point.z = 0.0;
    
  while ( ros::ok() )
  {

    

      try{
        // wait until transform is avalible 
        ros::Time now = ros::Time::now();
        listener.waitForTransform(reference_frame, tool_tip_l_frame, now, ros::Duration(1.0));
        listener.waitForTransform(tool_tip_l_frame, ft_frame, now, ros::Duration(1.0));
        
        // get the reference point in the tool tip frame
        listener.transformPoint(tool_tip_l_frame, fixpoint_reference_frame,	fixpoint_tool_tip_l_frame);
        

        //shift the point to sensor position
        listener.waitForTransform(tool_tip_l_frame, ft_frame, ros::Time(0), ros::Duration(1.0));
        listener.lookupTransform(tool_tip_l_frame, ft_frame, ros::Time(0), transform);
        fixpoint_tool_tip_l_frame.point.x += transform.getOrigin().x();
        fixpoint_tool_tip_l_frame.point.y += transform.getOrigin().y();
        fixpoint_tool_tip_l_frame.point.z += transform.getOrigin().z();

        // transform the point in the sensor frame
        listener.transformPoint(ft_frame, fixpoint_tool_tip_l_frame,	fixpoint_ft_frame);

        // TODO: Finn add cable weight
        // TODO: add noise


        //ROS_INFO_STREAM(testpoint.point);
            
      }
      catch (tf::TransformException ex){
        ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep();
      }

      double scaling = 0.0;
      double length = sqrt(pow(fixpoint_ft_frame.point.x,2.0) + pow(fixpoint_ft_frame.point.y,2.0) + pow(fixpoint_ft_frame.point.z,2.0));

      // calculate force that results from tensioning 
      if(length <= ropeLength){
        scaling = 0.0; // not tensiones
      }
      else{
        scaling = springConstant * (length - ropeLength);
      }

      setWrench(fixpoint_ft_frame.point.x*scaling/length, fixpoint_ft_frame.point.y*scaling/length, fixpoint_ft_frame.point.z*scaling/length, 0.0, 0.0, 0.0);
    
    if(!toSim)
    {
      setWrench(0.0001, 0.0001, 0.0001, 0.0, 0.0, 0.0);
    }

    // Publish transformed dat
    //ROS_INFO_STREAM(simWrench);
    netft_data_pub.publish( simWrench );
  
    loop_rate.sleep();		
    ros::spinOnce();
  }
  
  return 0;
}
