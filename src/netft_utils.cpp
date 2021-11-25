
/*
Copyright (c) 2016, Los Alamos National Security, LLC
All rights reserved.
Copyright 2016. Los Alamos National Security, LLC. This software was produced under U.S. Government contract DE-AC52-06NA25396 for Los Alamos National Laboratory (LANL), which is operated by Los Alamos National Security, LLC for the U.S. Department of Energy. The U.S. Government has rights to use, reproduce, and distribute this software.  NEITHER THE GOVERNMENT NOR LOS ALAMOS NATIONAL SECURITY, LLC MAKES ANY WARRANTY, EXPRESS OR IMPLIED, OR ASSUMES ANY LIABILITY FOR THE USE OF THIS SOFTWARE.  If software is modified to produce derivative works, such modified software should be clearly marked, so as not to confuse it with the version available from LANL.

Additionally, redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer. 
2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution. 
3. Neither the name of Los Alamos National Security, LLC, Los Alamos National Laboratory, LANL, the U.S. Government, nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission. 

THIS SOFTWARE IS PROVIDED BY LOS ALAMOS NATIONAL SECURITY, LLC AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL LOS ALAMOS NATIONAL SECURITY, LLC OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

Authors: Alex von Sternberg, Andy Zelenak
*/

#include "netft_utils.h"
#include <controller/Trajectory_msg.h>
#include <controller/Trajectory_point.h>
#include <math.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <cmath> 

int main(int argc, char **argv)
{
  // Initialize the ros netft_utils_node
  ros::init(argc, argv, "netft_utils_node");

  // Instantiate utils class
  ros::NodeHandle n;
  NetftUtils utils(n);
  ros::AsyncSpinner spinner(1);
  spinner.start();


  // Initialize utils
  utils.initialize();

  // Set up user input
  std::string world_frame = "world";
  std::string ft_frame = "ftsensor_l";

  if(argc<3)
  {
    ROS_FATAL("You must pass in at least the world and ft frame as command line arguments.");
    return 1;
  }
  else if(argc>=4)
  {
    ROS_FATAL("Too many arguments for netft_utils");
  }
  else
  {
    world_frame = argv[1];
    ft_frame = argv[2];
  }
  utils.setUserInput(world_frame, ft_frame);

  // Main ros loop
  ros::Rate loop_rate(500);
  //ros::Time last;
  while(ros::ok())
  {
    utils.update();
    loop_rate.sleep();
    //ros::Time curr = ros::Time::now();
    //ROS_INFO_STREAM("Loop time: " <<  curr.toSec()-last.toSec());
    //last = curr;
  }

  return 0;
}

NetftUtils::NetftUtils(ros::NodeHandle nh) :
  n(nh),
  isFilterOn(false),
  deltaTFilter(0.0),
  cutoffFrequency(0.0),
  newFilter(false),
  isBiased(false),
  isGravityBiased(false),
  isDifferentToolFrame(false),
  isNewBias(false),
  isNewGravityBias(false),
  payloadWeight(0.2),
  payloadLeverArm(0.2)
{
}

NetftUtils::~NetftUtils()
{
  delete listener;
  delete lp;
}

void NetftUtils::initialize()
{
  //lp = new LPFilter(0.002,200,6);
  
  //Zero out the zero wrench
  zero_wrench.wrench.force.x = 0.0;
  zero_wrench.wrench.force.y = 0.0;
  zero_wrench.wrench.force.z = 0.0;
  zero_wrench.wrench.torque.x = 0.0;
  zero_wrench.wrench.torque.y = 0.0;
  zero_wrench.wrench.torque.z = 0.0;

  

  //Initialize cancel message
  //cancel_msg.toCancel = false

  //Listen to the transfomation from the ft sensor to world frame.
  listener = new tf::TransformListener(ros::Duration(300));

  //Subscribe to the NetFT topic.
  raw_data_sub = n.subscribe("netft_data",100, &NetftUtils::netftCallback, this);

  //Publish on the /netft_transformed_data topic. Queue up to 100000 data points
  //netft_raw_world_data_pub = n.advertise<geometry_msgs::WrenchStamped>("raw_world", 100000);
  //netft_world_data_pub = n.advertise<geometry_msgs::WrenchStamped>("transformed_world", 100000);
  netft_tool_data_pub = n.advertise<geometry_msgs::WrenchStamped>("tool_tip_force_l", 100);
  netft_world_data_pub = n.advertise<geometry_msgs::WrenchStamped>("world_tip_force_l", 100);
  //netft_cancel_pub = n.advertise<netft_utils::Cancel>("cancel", 100000);
  // Initialize YuMi trajectory publischer
  //trajectory_pub = n.advertise<controller::Trajectory_msg>("/Trajectroy", 1);

  //Advertise bias and threshold services
  bias_service = n.advertiseService("bias", &NetftUtils::fixedOrientationBias, this);
  set_tool_tip_frame_service = n.advertiseService("set_tool_tip_frame", &NetftUtils::setToolTipFrame, this);
  gravity_comp_service = n.advertiseService("gravity_comp", &NetftUtils::compensateForGravity, this);
  set_tool_data = n.advertiseService("set_tool_data", &NetftUtils::settooldata, this);
  set_bias_data = n.advertiseService("set_bias_data", &NetftUtils::setbiasdata, this);
  find_tool_params = n.advertiseService("find_tool_params", &NetftUtils::findToolParams, this);
  weight_bias_service = n.advertiseService("set_weight_bias", &NetftUtils::setWeightBias, this);
  filter_service = n.advertiseService("filter", &NetftUtils::setFilter, this);
}

void NetftUtils::setUserInput(std::string world, std::string ft)
{
  world_frame = world;
  ft_frame = ft;
}

void NetftUtils::update()
{
  /*
  
  // Check for a filter
  if(newFilter)
  {   
    //delete lp;
    lp = new LPFilter(deltaTFilter,cutoffFrequency,6);   
    newFilter = false;
  }
  */
  
  // Look up transform from ft to world frame
  tf::StampedTransform tempTransform;
  try
  {
    listener->waitForTransform(world_frame, ft_frame, ros::Time(0), ros::Duration(1.0));
    listener->lookupTransform(world_frame, ft_frame, ros::Time(0), tempTransform);
  }
  catch (tf::TransformException ex)
  {
    ROS_ERROR("%s",ex.what());
  }

  // Set translation to zero before updating value
  tempTransform.setOrigin(tf::Vector3(0.0,0.0,0.0));
  ft_to_world = tempTransform;

  // Transform to world Frame
  transformFrame(raw_data_tool, raw_data_world, 'w');

  if (isBiased) // Apply the bias for a static sensor frame
  {
    // Get tool bias in world frame
    geometry_msgs::WrenchStamped world_bias;  // Wrench containing the current bias data in world frame
    transformFrame(tool_bias, world_bias, 'w');

    // Add bias and apply threshold to get transformed data
    copyWrench(raw_data_world, tf_data_world, world_bias);
    copyWrench(raw_data_tool, tf_data_tool, tool_bias);
  }
  else // Just pass the data straight through
  {
    copyWrench(raw_data_world, tf_data_world, zero_wrench);
    copyWrench(raw_data_tool, tf_data_tool, zero_wrench);
  }

  
  if (isGravityBiased) // Compensate for gravity. Assumes world Z-axis is up
  {
      // Gravity moment = (payloadLeverArm) cross (payload force)  <== all in the sensor frame. Need to convert to world later
      // Since it's assumed that the CoM of the payload is on the sensor's central axis, this calculation is simplified.
      double gravMomentX = -payloadLeverArm*tf_data_tool.wrench.force.y;
      double gravMomentY = payloadLeverArm*tf_data_tool.wrench.force.x;
    
      // Subtract the gravity torques from the previously-calculated wrench in the tool frame
      tf_data_tool.wrench.torque.x = tf_data_tool.wrench.torque.x - gravMomentX;
      tf_data_tool.wrench.torque.y = tf_data_tool.wrench.torque.y - gravMomentY;
    
      // Convert to world to account for the gravity force. Assumes world-Z is up.
      //ROS_INFO_STREAM("payloadWeight: "<< payloadWeight<<" payloadLeverArm: "<<payloadLeverArm<<" tf_data_world.wrench.force.z: "<<tf_data_world.wrench.force.z);
      transformFrame(tf_data_tool, tf_data_world, 'w');
      tf_data_world.wrench.force.z = tf_data_world.wrench.force.z - payloadWeight;

    
      // tf_data_world now accounts for gravity completely. Convert back to the tool frame to make that data available, too
      transformFrame(tf_data_world, tf_data_tool, 't');
  }
 

  if (isDifferentToolFrame) // Shift the measurement to the tool tip frame
  {
    
    // TODO: consider torque difference.
    tf::Vector3 tempF;
    tf::Vector3 tempT;
    tempF.setX(tf_data_tool.wrench.force.x);
    tempF.setY(tf_data_tool.wrench.force.y);
    tempF.setZ(tf_data_tool.wrench.force.z);
    tempT.setX(tf_data_tool.wrench.torque.x);
    tempT.setY(tf_data_tool.wrench.torque.y);
    tempT.setZ(tf_data_tool.wrench.torque.z);
    
    tf_data_tool_tip.header.frame_id = tool_tip_frame;

    tempF = toolTipTransform * tempF;
    tempT = toolTipTransform * tempT;

    tf_data_tool_tip.header.stamp = tf_data_tool.header.stamp;
    tf_data_tool_tip.wrench.force.x = tempF.getX();
    tf_data_tool_tip.wrench.force.y = tempF.getY();
    tf_data_tool_tip.wrench.force.z = tempF.getZ();
    tf_data_tool_tip.wrench.torque.x = tempT.getX();
    tf_data_tool_tip.wrench.torque.y = tempT.getY();
    tf_data_tool_tip.wrench.torque.z = tempT.getZ();


    double degrees = 0.0;
    n.getParam("ftSensorRotationOffset", degrees);

    tf_data_tool_tip.wrench.force.x = tempF.getX() * cos(degrees) - tempF.getY() * sin(degrees);
    tf_data_tool_tip.wrench.force.y = tempF.getX() * sin(degrees) + tempF.getY() * cos(degrees);

    
    

  }else // Just pass the data straight through
  {
    copyWrench(tf_data_tool, tf_data_tool_tip, zero_wrench);
  }

  
  transformFrame(tf_data_tool, tf_data_world, 'w');

  // Publish transformed dat
  netft_tool_data_pub.publish( tf_data_tool_tip );
  netft_world_data_pub.publish( tf_data_world );

  ros::spinOnce();
}

void NetftUtils::copyWrench(geometry_msgs::WrenchStamped &in, geometry_msgs::WrenchStamped &out, geometry_msgs::WrenchStamped &bias)
{
  out.header.stamp = in.header.stamp;
  out.header.frame_id = in.header.frame_id;
  out.wrench.force.x = in.wrench.force.x - bias.wrench.force.x;
  out.wrench.force.y = in.wrench.force.y - bias.wrench.force.y;
  out.wrench.force.z = in.wrench.force.z - bias.wrench.force.z;
  out.wrench.torque.x = in.wrench.torque.x - bias.wrench.torque.x;
  out.wrench.torque.y = in.wrench.torque.y - bias.wrench.torque.y;
  out.wrench.torque.z = in.wrench.torque.z - bias.wrench.torque.z;
}


void NetftUtils::transformFrame(geometry_msgs::WrenchStamped in_data, geometry_msgs::WrenchStamped &out_data, char target_frame)
{
  tf::Vector3 tempF;
  tf::Vector3 tempT;
  tempF.setX(in_data.wrench.force.x);
  tempF.setY(in_data.wrench.force.y);
  tempF.setZ(in_data.wrench.force.z);
  tempT.setX(in_data.wrench.torque.x);
  tempT.setY(in_data.wrench.torque.y);
  tempT.setZ(in_data.wrench.torque.z);
  if(target_frame == 'w')
  {
      out_data.header.frame_id = world_frame;
      tempF = ft_to_world * tempF;
      tempT = ft_to_world * tempT;
  }
  else if(target_frame == 't')
  {
      out_data.header.frame_id = ft_frame;
      tempF = ft_to_world.inverse() * tempF;
      tempT = ft_to_world.inverse() * tempT;
  }	
  out_data.header.stamp = in_data.header.stamp;
  out_data.wrench.force.x = tempF.getX();
  out_data.wrench.force.y = tempF.getY();
  out_data.wrench.force.z = tempF.getZ();
  out_data.wrench.torque.x = tempT.getX();
  out_data.wrench.torque.y = tempT.getY();
  out_data.wrench.torque.z = tempT.getZ();
}

// Runs when a new datapoint comes in
void NetftUtils::netftCallback(const geometry_msgs::WrenchStamped::ConstPtr& data)
{
  // Copy tool frame data.
  raw_data_tool.header.stamp = data->header.stamp;
  raw_data_tool.header.frame_id = ft_frame;
  raw_data_tool.wrench.force.x = data->wrench.force.x;
  raw_data_tool.wrench.force.y = data->wrench.force.y;
  raw_data_tool.wrench.force.z = data->wrench.force.z;
  raw_data_tool.wrench.torque.x = data->wrench.torque.x;
  raw_data_tool.wrench.torque.y = data->wrench.torque.y;
  raw_data_tool.wrench.torque.z = data->wrench.torque.z;
}               

// -------------------------------------------------------------------------------------------------------------------
// ---------------------------------- Services -----------------------------------------------------------------------
// -------------------------------------------------------------------------------------------------------------------

// Set the readings from the sensor to zero at this instant and continue to apply the bias on future readings.
// This doesn't account for gravity.
// Useful when the sensor's orientation won't change.
// Run this method when the sensor is stationary to avoid inertial effects.
// Cannot bias the sensor if gravity compensation has already been applied.
bool NetftUtils::fixedOrientationBias(netft_utils::SetBias::Request &req, netft_utils::SetBias::Response &res)
{                 
  if(req.toBias)  
  { 

    if(isBiased)
    {
      copyWrench(zero_wrench, tool_bias, zero_wrench);
      isBiased = false;
      ROS_INFO_STREAM("Delete old bias.");
    }

    ROS_INFO_STREAM("wait 5 sec for Bias calibaration...");

    double fx = 0;
    double fy = 0;
    double fz = 0;
    double tx = 0;
    double ty = 0;
    double tz = 0;

    int numMeasurments = 200;

    for (int i = 0; i <= numMeasurments; i++)
    {
      fx = fx + tf_data_tool.wrench.force.x;
      fy = fy + tf_data_tool.wrench.force.y;
      fz = fz + tf_data_tool.wrench.force.z;
      tx = tx + tf_data_tool.wrench.torque.x;
      ty = ty + tf_data_tool.wrench.torque.y;
      tz = tz + tf_data_tool.wrench.torque.z;

      ros::Duration(0.01).sleep();
    }

    tool_bias.header.stamp = tf_data_tool.header.stamp;
    tool_bias.header.frame_id = tf_data_tool.header.frame_id;
    tool_bias.wrench.force.x = fx/numMeasurments;
    tool_bias.wrench.force.y = fy/numMeasurments;
    tool_bias.wrench.force.z = fz/numMeasurments;
    tool_bias.wrench.torque.x = tx/numMeasurments;
    tool_bias.wrench.torque.y = ty/numMeasurments;
    tool_bias.wrench.torque.z = tz/numMeasurments;
    
    isBiased = true;

    ROS_INFO_STREAM("Bias calibaration done! Bias: \n"  << tool_bias.wrench );
  }               
  else            
  {               
    copyWrench(zero_wrench, tool_bias, zero_wrench); // Clear the stored bias if the argument was false
  }               

  res.success = true;
                  
  return true;    
}




// Set Tool Tip frame differen from Sensor origin
bool NetftUtils::setToolTipFrame(netft_utils::SetToolTipFrame::Request &req, netft_utils::SetToolTipFrame::Response &res)
{

  if(req.setToolTip)
  {  
    
    // Look up transform from ft to world frame
    tf::StampedTransform tempTransform;
    try
    {
      listener->waitForTransform(req.toolTipFrame, ft_frame, ros::Time(0), ros::Duration(1.0));
      listener->lookupTransform(req.toolTipFrame, ft_frame, ros::Time(0), toolTipTransform);

      toolTipTransform.setOrigin(tf::Vector3(0.0,0.0,0.0));

      tool_tip_frame = req.toolTipFrame;

      isDifferentToolFrame = true;

    }
    catch (tf::TransformException ex)
    {
      ROS_ERROR("%s",ex.what());
    }
  } else {
    isDifferentToolFrame = false;
  }
  
  res.success = true;     
  return true;
}


// Calculate the payload's mass and center of mass so gravity can be compensated for, even as the sensor changes orientation.
// It's assumed that the payload's center of mass is located on the sensor's central access.
// Run this method when the sensor is stationary to avoid inertial effects.
// Cannot do gravity compensation if sensor has already been biased.
bool NetftUtils::compensateForGravity(netft_utils::SetBias::Request &req, netft_utils::SetBias::Response &res)
{

  if(req.toBias)
  {  
    if (isBiased)
    {
      ROS_ERROR("Cannot compensate for gravity if the sensor has already been biased, i.e. useful data was wiped out");
      res.success = false;
      return false;  
    }
    else  // Cannot compensate for gravity if the sensor has already been biased, i.e. useful data was wiped out 
    {
      // Get the weight of the payload. Assumes the world Z axis is up.
      payloadWeight = raw_data_world.wrench.force.z;
  
      // Calculate the z-coordinate of the payload's center of mass, in the sensor frame.
      // It's assumed that the x- and y-coordinates are zero.
      // This is a lever arm.
      payloadLeverArm = raw_data_tool.wrench.torque.y/raw_data_tool.wrench.force.x;
    
      isNewGravityBias = true;
      isGravityBiased = true;
    }
  }
  
  res.success = true;     
  return true;
}


// Set the tooldata if known
bool NetftUtils::settooldata(netft_utils::SetToolData::Request &req, netft_utils::SetToolData::Response &res)
{

  if(req.setNew)
  {
    // Get the weight of the payload.
    payloadWeight = req.mass;
    
    // This is a lever arm.
    payloadLeverArm = req.COM;
      
    isNewGravityBias = true;
    isGravityBiased = true;
  } 
  else
  {
    isGravityBiased = false;
  }
  
  res.success = true;     
  return true;
}

// Set the bias data if known
bool NetftUtils::setbiasdata(netft_utils::SetBiasData::Request &req, netft_utils::SetBiasData::Response &res)
{

  if(req.setNew)
  {

    tool_bias.wrench.force.x = req.fx;
    tool_bias.wrench.force.y = req.fy;
    tool_bias.wrench.force.z = req.fz;
    tool_bias.wrench.torque.x = req.tx;
    tool_bias.wrench.torque.y = req.ty;
    tool_bias.wrench.torque.z = req.tz;
      
    isNewBias = true;
    isBiased = true;
  } 
  else
  {
    isBiased = false;
  }
  
  res.success = true;     
  return true;
}





bool NetftUtils::setFilter(netft_utils::SetFilter::Request &req, netft_utils::SetFilter::Response &res)
{            
  if(req.toFilter)  
  {
    
    newFilter = true;
    isFilterOn = true;
    deltaTFilter = req.deltaT;
    cutoffFrequency = req.cutoffFrequency;
  }               
  else            
  {               
    isFilterOn = false;
  }               
  
  return true;    
}  



bool NetftUtils::setWeightBias(netft_utils::SetBias::Request &req, netft_utils::SetBias::Response &res)
{           
  ROS_INFO_STREAM("request to bias");      
  if(req.toBias)  
  {               
    copyWrench(raw_data_tool, weight_bias, zero_wrench);
  }               
  else            
  {               
    copyWrench(zero_wrench, weight_bias, zero_wrench);
  }               
  res.success = true;
                  
  return true;    
}  
       


// procces to find exact tool data for YuMi use controller https://github.com/CRIS-Chalmers/yumi.git
bool NetftUtils::findToolParams(netft_utils::FindToolParams::Request &req, netft_utils::FindToolParams::Response &res)
{                 
  if(req.toDrive)  
  { 

    if(isBiased)
    {
      copyWrench(zero_wrench, tool_bias, zero_wrench);
      isBiased = false;
      ROS_INFO_STREAM("Delete old bias.");
    }

    if(isGravityBiased)
    { 
      payloadWeight = 0;
      payloadLeverArm = 0;
      copyWrench(zero_wrench, tool_bias, zero_wrench);
      isGravityBiased = false;
      ROS_INFO_STREAM("Delete old tool values.");
    }

    ROS_INFO_STREAM("YuMi will drive. Wait for calibaration...");
    double fz = 0;
    double m = 0;

    int numMeasurments = 500;

    controller::Trajectory_msg trajectory_msg;
    controller::Trajectory_point trajectory_point;

    
    trajectory_msg.mode = "individual"; // control mode

    // 1. Z down -----------------------------------------------------------------------------
    trajectory_msg.header.stamp = ros::Time::now();
    trajectory_point.positionRight = {0.50, -0.4, 0.3};  // poition right arm [m], yumi_base_link is the origin 
    trajectory_point.positionLeft = {0.50, 0.4, 0.3};  // poition left arm [m]
    trajectory_point.orientationLeft = {1, 0, 0, 0};// orientation left arm, quaterniorns [x, y, z, w]
    trajectory_point.orientationRight = {1, 0, 0, 0}; // orientation right arm
    trajectory_point.pointTime = 10.0; // time to get to this point [s]
    trajectory_msg.trajectory = {trajectory_point};
    trajectory_pub.publish(trajectory_msg);

    ros::Duration(20.0).sleep(); // sleep

    for (int i = 0; i <= numMeasurments; i++)
    {
      fz = fz + tf_data_world.wrench.force.z;
    }

    ROS_INFO_STREAM("Done z down 1/6");

    // 2. X down -----------------------------------------------------------------------------

    trajectory_msg.header.stamp = ros::Time::now();
    trajectory_point.positionRight = {0.60, -0.4, 0.4};  // poition right arm [m], yumi_base_link is the origin 
    trajectory_point.positionLeft = {0.60, 0.4, 0.4};  // poition left arm [m]
    trajectory_point.orientationLeft = {0.0, 0.707, 0.0 , 0.707};// orientation left arm, quaterniorns [x, y, z, w]
    trajectory_point.orientationRight = {0.0, 0.707, 0.0 , 0.707}; // orientation right arm
    trajectory_point.pointTime = 10.0; // time to get to this point [s]
    trajectory_msg.trajectory = {trajectory_point};
    trajectory_pub.publish(trajectory_msg);

    ros::Duration(20.0).sleep(); // sleep

    for (int i = 0; i <= numMeasurments; i++)
    {
      fz = fz + tf_data_world.wrench.force.z;
      m = m + tf_data_world.wrench.torque.y;
    }
    ROS_INFO_STREAM("Done x down 2/6");

    // 3. Y down -----------------------------------------------------------------------------
    trajectory_msg.header.stamp = ros::Time::now();
    trajectory_point.positionRight = {0.60, -0.4, 0.4};  // poition right arm [m], yumi_base_link is the origin 
    trajectory_point.positionLeft = {0.60, 0.4, 0.4};  // poition left arm [m]
    trajectory_point.orientationLeft = {-0.5, 0.5, -0.5, 0.5};// orientation left arm, quaterniorns [x, y, z, w]
    trajectory_point.orientationRight = {-0.5, 0.5, -0.5, 0.5}; // orientation right arm
    trajectory_point.pointTime = 10.0; // time to get to this point [s]
    trajectory_msg.trajectory = {trajectory_point};
    trajectory_pub.publish(trajectory_msg);

    ros::Duration(20.0).sleep(); // sleep

    for (int i = 0; i <= numMeasurments; i++)
    {
      fz = fz + tf_data_world.wrench.force.z;
      m = m + tf_data_world.wrench.torque.y;
    }
    
    ROS_INFO_STREAM("Done y down 3/6");

    // 4. X up -----------------------------------------------------------------------------
    trajectory_msg.header.stamp = ros::Time::now();
    trajectory_point.positionRight = {0.60, -0.4, 0.4};  // poition right arm [m], yumi_base_link is the origin 
    trajectory_point.positionLeft = {0.60, 0.4, 0.4};  // poition left arm [m]
    trajectory_point.orientationLeft = {-0.707, 0.0, -0.707, 0.0};// orientation left arm, quaterniorns [x, y, z, w]
    trajectory_point.orientationRight = {-0.707, 0.0, -0.707, 0.0}; // orientation right arm
    trajectory_point.pointTime = 10.0; // time to get to this point [s]
    trajectory_msg.trajectory = {trajectory_point};
    trajectory_pub.publish(trajectory_msg);

    ros::Duration(20.0).sleep(); // sleep

    for (int i = 0; i <= numMeasurments; i++)
    {
      fz = fz + tf_data_world.wrench.force.z;
      m = m + tf_data_world.wrench.torque.y;
    }

    ROS_INFO_STREAM("Done x up 4/6");

    // 5. Y up -----------------------------------------------------------------------------
    trajectory_msg.header.stamp = ros::Time::now();
    trajectory_point.positionRight = {0.60, -0.4, 0.4};  // poition right arm [m], yumi_base_link is the origin 
    trajectory_point.positionLeft = {0.60, 0.4, 0.4};  // poition left arm [m]
    trajectory_point.orientationLeft = {0.5, 0.5, 0.5, 0.5};// orientation left arm, quaterniorns [x, y, z, w]
    trajectory_point.orientationRight = {0.5, 0.5, 0.5, 0.5}; // orientation right arm
    trajectory_point.pointTime = 10.0; // time to get to this point [s]
    trajectory_msg.trajectory = {trajectory_point};
    trajectory_pub.publish(trajectory_msg);

    ros::Duration(20.0).sleep(); // sleep

    for (int i = 0; i <= numMeasurments; i++)
    {
      fz = fz + tf_data_world.wrench.force.z;
      m = m + tf_data_world.wrench.torque.y;
    }
    ROS_INFO_STREAM("Done y up 5/6");

    // 6. Z up -----------------------------------------------------------------------------
    trajectory_msg.header.stamp = ros::Time::now();

    trajectory_point.positionRight = {0.50, -0.3, 0.5};  // poition right arm [m], yumi_base_link is the origin 
    trajectory_point.positionLeft = {0.50, 0.3, 0.5};  // poition left arm [m]
    trajectory_point.orientationLeft = {0, 0, -1, 0};// orientation left arm, quaterniorns [x, y, z, w]
    trajectory_point.orientationRight = {0, 0, -1, 0}; // orientation right arm
    trajectory_point.pointTime = 10.0; // time to get to this point [s]
    trajectory_msg.trajectory = {trajectory_point};
    trajectory_pub.publish(trajectory_msg);

    ros::Duration(20.0).sleep(); // sleep 

    for (int i = 0; i <= numMeasurments; i++)
    {
      fz = fz + tf_data_world.wrench.force.z;
    }
    ROS_INFO_STREAM("Done z up 6/6");

    
    double force = std::abs(fz/(6*numMeasurments));
    double leverarm = m/(4*numMeasurments*force);

    ROS_INFO_STREAM("Force in N: "<<force<<"  Lever arm in mm: "<<leverarm);
    payloadWeight = force;
    payloadLeverArm = leverarm;
    
  }
  res.success = true;

                  
  return true;    
}