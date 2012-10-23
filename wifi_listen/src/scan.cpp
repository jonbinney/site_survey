#include <pcap.h>

#include <stdint.h>
#include <ros/ros.h>
#include <string>

using namespace std;

struct ieee80211_radiotap_header {
  u_int8_t        it_version;     /* set to 0 */
  u_int8_t        it_pad;
  u_int16_t       it_len;         /* entire length */
  u_int32_t       it_present;     /* fields present */
} __attribute__((__packed__));

int radiotap_field_sz(int id) {
  switch(id) {
    case 29:
      return 0;
    case 1:
    case 2:
    case 5:
    case 6:
    case 10:
    case 11:
    case 12:
    case 13:
      return 1;
    case 4:
    case 7:
    case 8:
    case 9:
    case 14:
      return 2;
    case 19:
      return 3;
    case 3:
      return 4;
    case 30:
      return 6;
    case 0:
    case 20:
      return 8;
    case 21:
      return 12;
    default: return 0;
  }
}

void print_radiotap_field(int id, const u_char * data) {
  uint8_t u8_a;
  uint16_t u16_a;
  switch(id) {
    case 0:
      // TSFT
      u16_a = *((uint16_t*)data);
      ROS_INFO("Received at timestamp %ld", u16_a);
      break;
    case 1:
      // Flags
      u8_a = *((uint8_t*)data);
      ROS_INFO("Frame flags %X", u8_a);
      if( u8_a & 0x1 ) {
        ROS_INFO("Frame sent/received during CFP");
      }
      if( u8_a & 0x2 ) {
        ROS_INFO("Frame had short preamble");
      }
      if( u8_a & 0x4 ) {
        ROS_INFO("Frame had WEP encryption");
      }
      if( u8_a & 0x8 ) {
        ROS_INFO("Frame was fragmented");
      }
      if( u8_a & 0x10 ) {
        ROS_INFO("Frame includes FCS");
      }
      if( u8_a & 0x20 ) {
        ROS_INFO("Frame has padding");
      }
      if( u8_a & 0x40 ) {
        ROS_INFO("Frame failed FCS check");
      }
      break;
    case 2:
      // Rate ( x 500kbps)
      break;
    case 3:
      // Channel
      break;
    case 4:
      // FHSS
      break;
    case 5:
      // Antenna signal
      break;
    case 6:
      // Antenna noise
      break;
    case 7:
      // Lock quality
      break;
    case 8:
      // TX Attenuation
      break;
    case 9:
      // dB TX Attenuation
      break;
    case 10:
      // dBm TX power
      break;
    case 11:
      // Antenna index
      break;
    case 12:
      // dB antenna signal
      break;
    case 13:
      // dB antenna noise
      break;
    case 14:
      // RX flags
      break;
    case 19:
      // MCS
      break;
    case 20:
      // A-MPDU status
      break;
    case 21:
      // VHT
      break;
    case 29:
      // radiotap namespace
      break;
    case 30:
      // Vendor namespace
      break;
    default:
      ROS_INFO("Got unknown radiotap field %d", id);
      break;
  }
}

void radiotap_parse(const u_char * raw_data, int len) {
  ieee80211_radiotap_header * header = (ieee80211_radiotap_header*)raw_data;

  if( header->it_version != 0 ) {
    ROS_WARN("New version of radiotap header detected. Aborting");
    return;
  }
  // pointer to header data
  const u_char * header_data = raw_data + sizeof(ieee80211_radiotap_header);
  // pointer to packet data
  const u_char * data = raw_data + header->it_len;
  ROS_INFO("Radiotap header is %d bytes long (paylod %ld)", header->it_len,
      header->it_len - sizeof(ieee80211_radiotap_header));
  ROS_INFO("Radiotap header flags %X", header->it_present);

  for( int i=0; i < 32; ++i ) {
    int j = 1 << i;
    if( j & header->it_present ) {
      ROS_INFO("Field %d present in radiotap header", i);
      int sz = radiotap_field_sz(i);
      if( header_data + sz <= data ) {
        print_radiotap_field(i, header_data);
        header_data += radiotap_field_sz(i);
      } else {
        ROS_WARN("Header fields overflow available data");
      }
    }
  }

  if( header_data < data ) {
    ROS_WARN("Header field size underflow");
  }
  if( header_data > data ) {
    ROS_WARN("Header field size overflow");
  }
}

int main(int argc, char ** argv) {
  char errstr[PCAP_ERRBUF_SIZE];
  int ret = 0;

  // ROS nodehandle init
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
      return -1;
    case PCAP_ERROR_NO_SUCH_DEVICE:
      ROS_ERROR("No such capture device: %s", iface.c_str());
      return -1;
    case PCAP_ERROR_PERM_DENIED:
      ROS_ERROR("Permission denied on %s", iface.c_str());
      return -1;
    case PCAP_ERROR_RFMON_NOTSUP:
      ROS_ERROR("RFMON mode not supported on %s", iface.c_str());
      return -1;
    case PCAP_ERROR_IFACE_NOT_UP:
      ROS_ERROR("%s is not up", iface.c_str());
      return -1;
    case PCAP_ERROR:
      ROS_ERROR("pcap error when activating %s", iface.c_str());
      return -1;
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
      radiotap_parse(data, header->caplen);
    }
    if( ret < 0 ) {
//      ROS_WARN("Error when fetching incoming packet %d", ret);
    }
    ros::spinOnce();
  }

  ROS_INFO("Capture done; exiting");

  pcap_close(cap);

  return 0;
}
