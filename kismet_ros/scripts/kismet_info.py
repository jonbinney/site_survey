#!/usr/bin/env python

import roslib; roslib.load_manifest('kismet_ros')
import rospy

import socket
import threading

class KismetClient(threading.Thread):
  def __init__(self, host='localhost', port=2501):
    threading.Thread.__init__(self)
    self._client = socket.socket(socket.AF_INET)
    remote = socket.getaddrinfo('localhost', 2501, socket.AF_INET, socket.SOCK_STREAM)
    self._client.connect(remote[0][4])
    self._client.settimeout(0.1)
    self._done = False
    self._last_time = None
    self._cmd_num = 1

    self._callbacks = {}

    self._protocol_fields = {
      'KISMET': ['version', 'starttime', 'servername', 'dumpfiles', 'uid'],
      'CAPABILITY': ['protocol', 'capabilities'],
      'TIME': ['timesec'], 
      'PROTOCOLS': ['protocols'],
      'ERROR': ['cmdid', 'text'],
      'ACK': ['cmdid', 'text']
    }

    self._callbacks['KISMET'] = []
    self._callbacks['TIME'] = [self.time_cb]
    self._callbacks['PROTOCOLS'] = [self.prot_cb]
    self._callbacks['ERROR'] = [self.err_cb]
    self._callbacks['ACK'] = [self.ack_cb]
    self._callbacks['CAPABILITY'] = [self.cap_cb]

    self.start()

  def send_command(self, command):
    self._client.send('!%d %s\n'%(self._cmd_num, command))
    self._cmd_num += 1

  def enable(self, protocol):
    self.send_command('enable %s *'%protocol)

  def add_callback(self, name, callback):
    if name in self._callbacks:
      self._callbacks[name].append(callback)

  def subscribe(self, name, callback):
    self.add_callback(name, callback)
    self.enable(name)

  def prot_cb(self, parts):
    # Add protocol
    parts = parts['protocols'].split(',')
    for p in parts:
      if not p in self._callbacks:
        self._callbacks[p] = []
      if not p in self._protocol_fields:
        self.send_command('CAPABILITY %s'%p)

  def cap_cb(self, parts):
    fields = parts['capabilities'].split(',')
    if parts['protocol'] in self._protocol_fields:
      print "Already had capabilities for %s; replacing"%(parts['protocol'])
    self._protocol_fields[parts['protocol']] = fields

  def time_cb(self, parts):
    self._last_time = parts['timesec']

  def err_cb(self, parts):
    print "Got Error %s for message %s"%(parts['text'], parts['cmdid'])

  def ack_cb(self, parts):
    pass
    #print "Got acknowledgement: %s"%parts[1]

  def _split_parts(self,line):
    parts = []
    # strings in a line are quoted with \001 ; fields are otherwise separated
    # by spaces
    string_mode = False
    part = ''
    for c in line:
      if string_mode:
        if c == '\001':
          string_mode = False
        else:
          part += c
      else:
        if c == ' ':
          parts.append(part)
          part = ''
        elif c == '\001':
          string_mode = True
        else:
          part += c
    if len(part) > 0:
      parts.append(part)
    return parts
 
  def run(self):
    try:
      data = self._client.recv(4096)
      got_data = 1
    except:
      data = ''
      got_data = 0
    while not self._done and data != None:
      if got_data:
        l = data.rfind('\n')
        lines = data[0:l]
        data = data[l+1:]
        #if len(data) > 0:
        #  print "Remaining data in buffer: %s"%data
        for line in lines.split('\n'):
          t = line[1:line.find(':')]
          if not t in self._callbacks:
            print "protocol %s isn't registered"%(t)
          elif not t in self._protocol_fields:
            print "no fields registered for protocol %s"%(t)
          else:
            parts = self._split_parts(line)[1:]
            fields = self._protocol_fields[t]
            if len(parts) != len(fields):
              print "Field length mismatch for %s. Got %d, expected %d"%(
                  t, len(parts), len(fields))
              print line
              print parts
              print fields
            else:
              for c in self._callbacks[t]:
                c(dict(zip(fields,parts)))
            if len(self._callbacks[t]) == 0:
              print "No callbacks for %s"%t
      try:
        buf = self._client.recv(4096)
        if buf:
          data += buf
        else:
          data = None
        got_data = 1
      except Exception as e:
        got_data = 0
    print "Kismet receive thread exiting"
          
  def stop(self):
    self._done = True

def bssid_cb(parts):
  print "Got BSSID %s channel %s retries %s"%(parts['bssid'], 
      parts['channel'],
      parts['retries'])

def ssid_cb(parts):
  print "Got SSID %s MAC %s"%(parts['ssid'], parts['mac'])

def source_cb(parts):
  print parts

def main():

  client = KismetClient()
  rospy.init_node('kismet_ros')
  
# BSSID detections (aggregated)
#  client.subscribe('BSSID', bssid_cb)

# SSID detections (aggregated)
#  client.subscribe('SSID', ssid_cb)

# Capture sources
#  client.subscribe('SOURCE', source_cb)

  #client.enable('CLIENT')

  rospy.spin()

  client.stop()

if __name__ == '__main__':
  main()
