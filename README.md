# sot_robot

Author: Nirmal Giftsun

##Description
sot_robot is a ROS based real time controller which loads 'Stack of Tasks' to perform Hierarchical Task Function based 
Control
##Installation on ubuntu-12.04 with ros-hydro

To install all the packages on ubuntu 12.04 LTS, you should do the following
steps:

  1. install by apt-get (see http://wiki.ros.org/hydro/Installation/Ubuntu)
    - ros-hydro-desktop-full,
    - ros-hydro-pr2-robot,
    - ros-hydro-libccd,
    - ros-hydro-srdfdom,
    - urdfdom
    - urdfdom_headers
    - ros-hydro-robot_model
    - ros-hydro-pr2-controllers
    - ros-hydro-ros-control
    - asciidoc,
    - source-highlight,
    - git
        

  2. Choose a directory on you file system and define the environment
     variable DEVEL_DIR with the full path to this directory.
     - the packages will be cloned into $DEVEL_DIR/src,
     - the packages will be installed in $DEVEL_DIR/install.
     It is recommanded to set variable DEVEL_DIR in your .bashrc for future use.

  3. Copy Config and Makefile from scripts/installation
      - copy the config.sh file to $DEVEL_DIR
          wget -O $DEVEL_DIR/config.sh https://raw.githubusercontent.com/nemogiftsun/sot_robot/hydro/script/installation/config.sh
      - copy the Makefile to src
          $DEVEL_DIR/src/Makefile https://raw.githubusercontent.com/nemogiftsun/sot_robot/hydro/script/installation/Makefile
  
  4. cd into $DEVEL_DIR and type

    source config.sh

  5. cd into $DEVEL_DIR/src and type

    make all
