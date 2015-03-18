export DEVEL_CONFIG=ros
export DEVEL_DIR=~/laasinstall/devel/$DEVEL_CONFIG
export PATH=$DEVEL_DIR/install/bin:$DEVEL_DIR/install/sbin:$PATH
export PKG_CONFIG_PATH=$DEVEL_DIR/install/lib/pkgconfig:/opt/ros/hydro/lib/pkgconfig/:/usr/lib/pkgconfig/
export PYTHONPATH=$DEVEL_DIR/install/lib/python2.7/site-packages:$DEVEL_DIR/install/lib/python2.7/dist-packages:/opt/ros/hydro/lib/python2.7/dist-packages:$PYTHONPATH
export LD_LIBRARY_PATH=$DEVEL_DIR/install/lib:/opt/ros/hydro/lib:usr/install/lib:$LD_LIBRARY_PATH

source /opt/ros/hydro/setup.bash


