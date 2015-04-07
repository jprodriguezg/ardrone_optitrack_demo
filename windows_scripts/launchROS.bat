set ROS_MASTER_URI=http://192.168.201.240:11311
set ROS_IP=192.168.201.240
set ROS_HOSTNAME=192.168.201.240


call C:\Users\install\work\catkin_workspace\devel\setup.bat

rosparam set /optitrack_node/my_ip 127.0.0.1
rosparam set /optitrack_node/optitrack_pc_ip 127.0.0.1
rosparam set /optitrack_node/use_vicon_coord_sys true
rosparam set /optitrack_node/restamp_message true

start roscore
start rosrun optitrack_client optitrack_node