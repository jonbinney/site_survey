#include <pcap.h>
#include <stdio.h>
#include <ros/ros.h>
#include <string>

using namespace std;

int main(int argc, char ** argv) {
  char errstr[PCAP_ERRBUF_SIZE];
  int ret = 0;

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

  // parameters
  
  // capture device
  string iface;
  nh.param<string>("capture_dev", iface, "wlan0");

  // enable rfmon mode
  bool rfmon;
  nh.param<bool>("rfmon", rfmon, false);

  // frequency
  int hz = 1;

  // end parameters

  // pcap setup
  pcap_t * cap;
  cap = pcap_create(iface.c_str(), errstr);
  if( ! cap ) {
    ROS_FATAL("Failed to create capture device %s", errstr);
    return -1;
  }

  // set 1MB capture buffer
  if( pcap_set_buffer_size(cap, 1024*1024) ) {
    ROS_ERROR("Failed to set buffer size on %s", iface.c_str());
  }

  // set promiscuous mode
  if( pcap_set_promisc(cap, 1) ) {
    ROS_ERROR("Failed to set promiscuous mode on %s", iface.c_str());
  }

  // set snapshot length
  if( pcap_set_snaplen(cap, 65536) ) {
    ROS_ERROR("Failed to set snapshot length on %s", iface.c_str());
  }

  // set timeout
  if( pcap_set_timeout(cap, 1000/hz) ) {
    ROS_ERROR("Failed to set capture timeout on %s", iface.c_str());
  }

  if( rfmon ) {
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
  }

  // activate capture device
  switch( (ret = pcap_activate(cap)) ) {
    case 0:
      ROS_INFO("Started capture on %s", iface.c_str());
      break;
    case PCAP_WARNING_PROMISC_NOTSUP:
      ROS_WARN("Promiscuous mode not supported on %s", iface.c_str());
      break;
    case PCAP_WARNING:
      ROS_WARN("pcap warning when activating %s", iface.c_str());
      break;
    case PCAP_ERROR_ACTIVATED:
      ROS_ERROR("%s has already been activated for capturing", iface.c_str());
      break;
    case PCAP_ERROR_NO_SUCH_DEVICE:
      ROS_ERROR("No such capture device: %s", iface.c_str());
      break;
    case PCAP_ERROR_PERM_DENIED:
      ROS_ERROR("Permission denied on %s", iface.c_str());
      break;
    case PCAP_ERROR_RFMON_NOTSUP:
      ROS_ERROR("RFMON mode not supported on %s", iface.c_str());
      break;
    case PCAP_ERROR_IFACE_NOT_UP:
      ROS_ERROR("%s not set up for capturing", iface.c_str());
      break;
    case PCAP_ERROR:
      ROS_ERROR("pcap error when activating %s", iface.c_str());
      break;
    default:
      ROS_WARN("Unknown return code from pcap_activate: %d", ret);
      break;
  }

  // get data-link type(s)
  int * dlink_buffer;
  int dlink_sz;
  if( (dlink_sz = pcap_list_datalinks(cap, &dlink_buffer)) > 0 ) {
    ROS_INFO("%d datalink types for %s", dlink_sz, iface.c_str());
    for( int i=0; i<dlink_sz; ++i ) {
      ROS_INFO("Datalink %s(%d) for %s",
          pcap_datalink_val_to_name(dlink_buffer[i]), dlink_buffer[i],
          iface.c_str());
    }
    pcap_free_datalinks(dlink_buffer);
    dlink_buffer = NULL;
  } else {
    ROS_ERROR("Failed to get list of datalinks for %s", iface.c_str());
  }

  // get current data-link type
  ret = pcap_datalink(cap);
  ROS_INFO("Datalink type %s(%d) on %s", pcap_datalink_val_to_name(ret), ret,
      iface.c_str());

  // end pcap setup

  // ROS Setup
  ros::Rate loop_rate(hz);

  // end ROS Setup

  // main loop
  while( ros::ok() ) {
    pcap_pkthdr * header;
    const u_char * data;
    // From the man page: "The struct pcap_pkthdr and the packet data are not
    //   to be freed by the caller, and are not guaranteed to be valid after
    //   the next call to pcap_next_ex"
    ret = pcap_next_ex(cap, &header, &data);
    if( ret > 0 ) {
      ROS_INFO("Captured packet with length %d(%d)", header->len, 
          header->caplen);
    }
    if( ret < 0 ) {
      ROS_WARN("Error when fetching incoming packet %d", ret);
    }
    ros::spinOnce();
  }

  ROS_INFO("Capture done; exiting");

  pcap_close(cap);

  return 0;
}
