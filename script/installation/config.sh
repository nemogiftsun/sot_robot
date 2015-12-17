export DEVEL_CONFIG=ros
export DEVEL_DIR=~/RobotSoftware/laas/devel/$DEVEL_CONFIG
export PATH=$DEVEL_DIR/install/bin:$DEVEL_DIR/install/sbin:$PATH
export PKG_CONFIG_PATH=$DEVEL_DIR/install/lib/pkgconfig:/opt/ros/hydro/lib/pkgconfig/:/usr/lib/pkgconfig/:/usr/local/lib/pkgconfig/
export PYTHONPATH=$DEVEL_DIR/install/lib/python2.7/site-packages:$DEVEL_DIR/install/lib/python2.7/dist-packages:/opt/ros/hydro/lib/python2.7/dist-packages:$PYTHONPATH
export LD_LIBRARY_PATH=$DEVEL_DIR/install/lib:/opt/ros/hydro/lib:usr/local/lib:usr/install/lib:$LD_LIBRARY_PATH

source /opt/ros/hydro/setup.bash
source $DEVEL_DIR/install/setup.bash 
export ROS_PACKAGE_PATH=$DEVEL_DIR/src:$DEVEL_DIR/install/lib:$ROS_PACKAGE_PATH
