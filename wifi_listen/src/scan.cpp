#include <pcap.h>
#include <stdio.h>

int main(int argc, char ** argv) {

  pcap_if_t *interfaces;
  char errstr[PCAP_ERRBUF_SIZE];
  int ret = pcap_findalldevs(&interfaces, errstr);

  pcap_if_t * cur_if = interfaces;
  while( cur_if ) {
    printf("%s\n", cur_if->name);
    cur_if = cur_if->next;
  }
  return 0;
}
