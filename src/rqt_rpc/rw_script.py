import xml.etree.ElementTree as ET
file = '/home/nemogiftsun/laasinstall/devel/ros/src/sot_robot/src/rqt_rpc/rpc_config.xml'
# Parse the configuration file
tree = ET.parse(file)
root = tree.getroot()
if (root.tag == "Trajectories"):
    trajectories = root.findall('trajectory')
    count = len(trajectories)







