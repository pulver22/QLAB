# Source this file to set environmental variable and set route

MY_WLAN=192.168.131.221  # Wireless address of my laptop
KF_WLAN=192.168.131.203 #230 # Wireless address of Kingfisher
KF_ETH=192.168.1.11 # Wired for Kingfisher, where ROS master lives
#sudo route add -net ${KF_ETH} netmask 255.255.255.255 gw ${KF_WLAN}
export ROS_IP=${MY_WLAN} # My laptop
export ROS_MASTER_URI=http://${KF_ETH}:11311
export ROS_HOSTNAME=${MY_WLAN}
# Do this last - if you do it twice it throws an error and stops
sudo route add -host ${KF_ETH} gw ${KF_WLAN} dev wlan0

#ssh administrator@192.168.131.230
#eth0      Link encap:Ethernet  HWaddr 50:7b:9d:b4:82:08  
#          inet addr:172.20.89.21  Bcast:172.20.91.255  Mask:255.255.252.0

# This works
#sudo route add -host 192.168.1.11 gw 192.168.131.230 dev wlan0

# Both work
#rostopic pub -r 10 /cmd_drive kingfisher_msgs/Drive '{left: 0.1, right: 0.0}'
#rostopic pub -r 10 /cmd_drive kingfisher_msgs/Drive -- 0.05 0.0
