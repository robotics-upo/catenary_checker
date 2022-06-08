#! /bin/bash
sleep ${5-1}
rostopic pub /pose geometry_msgs/PoseStamped "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: '${4-map}'
pose:
  position:
    x: ${1-0.0}
    y: ${2-0.0}
    z: ${3-0.0}
  orientation:
    x: 0.0
    y: 0.0
    z: 0.0
    w: 1.0"

