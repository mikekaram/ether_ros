#! /bin/bash 
rosservice call /modify_output_bit $'slave: 0\nindex: 0\nsubindex: 0\nvalue: true'
rosservice call /modify_output_bit $'slave: 0\nindex: 0\nsubindex: 4\nvalue: true'
rosservice call /modify_output_sint16 $'slave: 0\nindex: 24\nvalue: 0'
rosservice call /modify_output_sint16 $'slave: 0\nindex: 26\nvalue: 590'  
rosservice call /modify_output_sint16 $'slave: 0\nindex: 28\nvalue: 0' 
rosservice call /modify_output_sint16 $'slave: 0\nindex: 30\nvalue: 3'
rosservice call /modify_output_sint16 $'slave: 0\nindex: 32\nvalue: 100' 
rosservice call /modify_output_sint16 $'slave: 0\nindex: 34\nvalue: 0' 
rosservice call /modify_output_sint16 $'slave: 0\nindex: 36\nvalue: 0'
rosservice call /ethercat_communicatord "mode: 'start'"