#!/usr/bin/env python

import roslib; roslib.load_manifest('kismet_ros')
import rospy

import socket
import threading

def recvlines(sock):
  try:
    data = sock.recv(4096)
  except:
    data = None
  while data:
    for line in data.split('\n'):
      yield line
    try:
      data = sock.recv(4096)
    except:
      data = None

class KismetClient(threading.Thread):
  def __init__(self, host='localhost', port=2501):
    threading.Thread.__init__(self)
    self._client = socket.socket(socket.AF_INET)
    remote = socket.getaddrinfo('localhost', 2501, socket.AF_INET, socket.SOCK_STREAM)
    self._client.connect(remote[0][4])
    self._client.settimeout(0.1)
    self._done = False
    self._last_time = None

    self._callbacks = {}

    self._callbacks['TIME'] = [self.time_cb]
    self._callbacks['PROTOCOLS'] = [self.prot_cb]
    self._callbacks['ERROR'] = [self.err_cb]
    self._callbacks['ACK'] = [self.ack_cb]

    self.start()

  def send_command(self, command):
    self._client.send('!0 %s\n'%(command))

  def enable(self, protocol):
    self.send_command('enable %s *'%protocol)

  def add_callback(self, name, callback):
    if name in self._callbacks:
      self._callbacks[name].append(callback)

  def prot_cb(self, line):
    # Add protocol
    parts = line.split(' ')[1].split(',')
    for p in parts:
      if not p in self._callbacks:
        print "Adding callback array for protocol %s"%p
        self._callbacks[p] = []

  def time_cb(self, line):
    parts = line.split(' ')
    self._last_time = parts[1]
    print 'Got server timestamp %s'%(parts[1])

  def err_cb(self, line):
    print "Got Error: %s"%line

  def ack_cb(self, line):
    print "Got acknowledgement: %s"%line
 
  def run(self):
    try:
      data = self._client.recv(4096)
    except:
      data = None
    while not self._done:
      if data:
        for line in data.split('\n'):
          if len(line) > 0:
            t = line[1:line.find(':')]
            if t in self._callbacks:
              for c in self._callbacks[t]:
                c(line)
              if len(self._callbacks[t]) == 0:
                print "No callbacks for %s"%t
            else:
              print "protocol %s isn't registered"%t
      try:
        data = self._client.recv(4096)
      except:
        data = None
    print "Kismet receive thread exiting"
          
  def stop(self):
    self._done = True


def main():

  client = KismetClient()
  rospy.init_node('kismet_ros')
  
  client.enable('packet')

  rospy.spin()

  client.stop()

if __name__ == '__main__':
  main()
