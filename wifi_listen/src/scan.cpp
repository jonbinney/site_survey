#include <pcap.h>
#include <stdio.h>
#include <ros/ros.h>
#include <string>

using namespace std;

int main(int argc, char ** argv) {
  char errstr[PCAP_ERRBUF_SIZE];

  // List interfaces
  /*
  pcap_if_t *interfaces = NULL;
  int ret = pcap_findalldevs(&interfaces, errstr);

  pcap_if_t * cur_if = interfaces;
  while( cur_if ) {
    printf("%s\n", cur_if->name);
    cur_if = cur_if->next;
  }
  */

  ros::init(argc, argv, "wifi_scan");
  ros::NodeHandle nh("~");

  string iface;
  nh.param<string>("capture_dev", iface, "wlan0");

  pcap_t * cap;
  cap = pcap_create(iface.c_str(), errstr);

  if( ! cap ) {
    ROS_FATAL("Failed to create capture device %s", errstr);
    return -1;
  }

  // set rfmon mode
  if( pcap_can_set_rfmon(cap) ) {
    ROS_INFO("Setting %s to rfmon mode", iface.c_str());
    if( pcap_set_rfmon(cap, 1) ) {
      ROS_FATAL("Failed to set rfmon mode on %s", iface.c_str());
      return -1;
    }
  } else {
    ROS_FATAL("Can't set rfmon mode on %s", iface.c_str());
    return -1;
  }

  while( ros::ok() ) {
  }

  pcap_close(cap);

  return 0;
}
