#!/usr/bin/env python

import roslib; roslib.load_manifest('survey_driver')
import rospy

from geometry_msgs.msg import PoseStamped
from interactive_markers.interactive_marker_server import *

class GoalManager:
  def __init__(self):
    self._sub = rospy.Subscriber('goal', PoseStamped, self.goal_cb)
    self._goals = []
    self._server = InteractiveMarkerServer('goals')

  def goal_cb(self, goal):
    self._goals.append(goal.pose)
    marker = InteractiveMarker()
    marker.header.frame_id = '/map'
    marker.pose = goal.pose
    marker.name = str(len(self._goals))

    control = InteractiveMarkerControl()
    control.always_visible = True
    control.orientation_mode = InteractiveMarkerControl.ROTATE_AXIS
    control.interaction_mode = InteractiveMarkerControl.MOVE_PLANE
    control.independent_marker_orientation = True

    m = Marker()
 
    m.type = Marker.ARROW
    m.scale.x = 2.0
    m.scale.y = 2.0
    m.scale.z = 0.5

    m.color.r = 1.0
    m.color.g = 1.0
    m.color.b = 0.0
    m.color.a = 1.0

    control.markers.append(m)

    marker.controls.append(control)

    self._server.insert(marker)
    self._server.applyChanges()

  def feedback_cb(self, feedback):
    pass

def main():
  rospy.init_node('survey_planner')

  goal_manager = GoalManager()

  rospy.loginfo('survey_planner ready')
  rospy.spin()

if __name__ == '__main__':
  main()
