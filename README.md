The netft_utils package provides a C++ interface and a ROS node for an ATI force/torque sensor that's connected to an ATI Netbox. There are some handy features like:

1. specify the rate of data acquistion,

2. set the World tf frame and the Sensor tf frame so forces are automatically transformed as a robot moves,

3. bias/threshold the sensor,

4. compensate for gravity forces on the mass of the end-effector as the end-effector changes orientation

This package builds on the original package by Derek King, written in 2008 but not updated since ROS groovy. http://wiki.ros.org/netft_rdt_driver|netft_rdt_driver


# Installing the package
It's best to install this package from source. Create a catkin workspace for the package:


`$ mkdir -p ~/Desktop/catkin_ws/src`\
`$ cd catkin_ws`\
`$ catkin_make`

Clone the package


`$ cd src`\
`$ git clone https://github.com/Finn-Sueberkrueb/netft_utils.git`\
`$ cd ..`

Source the new files


`$ source devel/setup.bash`

(You may have to do this every time you open a new terminal window, or add this line to your bashrc: `source ~/Desktop/catkin_ws/devel/setup.bash`) Build the package


`$ catkin_make`

Check that it runs


`$ roscore`\
`$ rosrun netft_utils netft_utils_sim`\
`$ rostopic echo /netft_data`

This handy node is simulating a F/T sensor - good for debugging.

# Launch a F/T Sensor node

`$ rosrun netft_utils netft_node 192.168.125.2`\
`$ rostopic echo /netft_data`

The IP address should be the IP address of your sensor. You should see a steady stream of WrenchStamped data.

Optional arguments:

* address 192.168.125.2 Specify the ip address
* wrench Publish the older Wrench data type instead of WrenchStamped
* rate Set the publish rate in Hz

After launching this netft_node, you can launch a netft_utils node to interact with the netft_node in the usual ROS ways. Pass the tf frame for the world and the tf frame for the sensor on the robot and you can easily convert between them. If you don't care about tf frames, just pass it dummy arguments. The sensor tf frame should be set up to match the Y and Z-axis markings on the sensor.


`$ rosrun netft_utils netft_utils yumi_base_link ftsensor_l`\
`$ rostopic echo /raw_world`  (raw data from the sensor, no bias applied, transformed to world frame)\
`$ rostopic echo /transformed_tool `  (if you biased the sensor, it has an effect here)\
`$ rostopic echo /transformed_world ` (if you biased the sensor, it has an effect here)


Bias the sensor, i.e. set its values to zero:

`$ rosservice call /bias true 10 5`

where 10 is the max force measurement and 5 is the max torque measurement you expect. These arguments are useful for getting better resolution from the sensor.

# rosservice
Find the tool parameters by rotating the tool. This is special for the ABB Yumi robot and the following controller: https://github.com/CRIS-Chalmers/yumi.git \
`$ rosservice call FindToolParams true`

`$ rosservice call set_tool_data -- true -2.714805 0.04597385` \
The first value is the force exerted by the tool weight in N. The second value is the distance between the tools CoM and the sensor reference point.



`$ rosservice call bias true 10 5`\
`$ rosservice call SetBiasData true fx fy fz tx ty tz`\
`$ rosservice call start_sim 0.1 2.0 0.1 "world" 0.4 0 0`\
`$ rosservice call start_sim ropeLength springConstant ropeWeight "frame" x_mounting_point y_mounting_point z_mounting_point`\
`$ rosservice call stop_sim`



# Gravity compensation
If the sensor hasn't been biased, you can enable gravity compensation to cancel out the force/torque readings due to the mass of the payload. The code makes two assumptions:

1. The World Z-axis is defined upward, i.e. anti-parallel with gravity.

2. The center of mass of the payload is located on the central Z-axis of the force/torque sensor. Usually payloads are fairly symmetric so this is mostly true. This assumption is necessary because a cross-product can't be inverted and solved, i.e. there are infinite solutions.

To test gravity compensation, do

`$ rosrun netft_utils netft_node 192.168.125.2`\
`$ rosrun netft_utils netft_utils yumi_base_link ftsensor_l`\
`$ rostopic echo /transformed_world`\
`$ rosservice call /bias true 10 5`

You should see the World-Z component of force disappear along with any moments due to gravity. Gravity compensation will continue to be applied as the end-effector moves. After you enable gravity compensation, you can bias the sensor to zero out the remaining forces/torques on the other axes.

# C++ Interface
The following code covers the gist of how to declare and use a netft object with C++. Do {{{$rosrun netft_utils netft_utils_cpp_test}}} to see it in action (you will have to adjust the IP address of the sensor in the source code, as it's hardcoded to 192.168.1.84.

<pre><code>
  ros::init(argc, argv, "netft_utils_cpp_test");
  ros::NodeHandle n;
  ros::AsyncSpinner* spinner;
  spinner = new ros::AsyncSpinner(3);
  spinner->start();
  double ftSleep; // controls the data acquisition rate

  // Subscribe to the F/T topic
  ros::Subscriber ftSub = n.subscribe("/netft/netft_data", 1, netftCallback);

  // Connect and bias the ft sensor
  NetftUtilsLean* fti = new NetftUtilsLean(&n);
  fti->setFTAddress("192.168.1.84");
  // Adjust the data acquisition rate and set the World and sensor frames, respectively
  fti->initialize(1/ftSleep, "base_link", "sensor_frame");

  // Set max and min force/torque readings
  fti->setMax(80.0, 8.0, 60.0, 6.0);
  std::future<bool> ftThread;
  ftThread = std::async(std::launch::async, &NetftUtilsLean::run, fti);
  fti->biasSensor(1);
</code></pre>
The asynchronous spinner is highly recommended.

To get data in the tool frame, do
<pre><code>
  geometry_msgs::WrenchStamped wrench;
  fti->getToolData(wrench);
</code></pre>

And in the world frame:
<pre><code>
  fti->getWorldData(wrench);
</code></pre>

# Web Interface
It's easy to forget the force/torque sensor has a web interface. Just open a browser and enter its IP address, e.g. 192.168.125.2. Here you can see the status of the sensor, change units from metric to imperial, adjust its DAQ rate, etc.

# Troubleshooting
If the program Seg Faults immediately, you may have specified an incorrect IP address for the sensor. Follow this [procedure](http://superuser.com/questions/261818/how-can-i-list-all-ips-in-the-connected-network-through-terminal-preferably) to see a list of IP addresses on your network and find the right one.

If the network latency increases significantly after launching the F/T sensor, it is probably pushing too much data. You can reduce its data transmission rate through the Netbox's web interface. Simply navigate to the sensor's IP in a browser, e.g.: http://192.168.125.2 Reduce the data transmission rate downward via "RDT Output Rate."

Note that the built-in axes on the ATI sensor don't follow the right-hand convention. We negate the X-component of the measured wrench so it does follow the right-hand convention.