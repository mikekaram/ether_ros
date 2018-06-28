#! /bin/bash 
rosservice call /modify_output_bit $'slave: 0\nindex: 0\nsubindex: 0\nvalue: true' # state_machine_id
rosservice call /modify_output_bit $'slave: 0\nindex: 0\nsubindex: 4\nvalue: true' # blue_led_id
rosservice call /modify_output_sint16 $'slave: 0\nindex: 24\nvalue: 0' # x_cntr_traj_id
rosservice call /modify_output_sint16 $'slave: 0\nindex: 26\nvalue: 590'   # y_cntr_traj_id
rosservice call /modify_output_sint16 $'slave: 0\nindex: 28\nvalue: 0' # a_ellipse_id
rosservice call /modify_output_sint16 $'slave: 0\nindex: 30\nvalue: 3' # b_ellipse_id
rosservice call /modify_output_sint16 $'slave: 0\nindex: 32\nvalue: 100' # traj_freq_id
rosservice call /modify_output_sint16 $'slave: 0\nindex: 34\nvalue: 0' # phase_deg_id
rosservice call /modify_output_sint16 $'slave: 0\nindex: 36\nvalue: 0' # flatness_param_id

rosservice call /modify_output_bit $'slave: 1\nindex: 0\nsubindex: 0\nvalue: true'
rosservice call /modify_output_bit $'slave: 1\nindex: 0\nsubindex: 4\nvalue: true'
rosservice call /modify_output_sint16 $'slave: 1\nindex: 24\nvalue: 0'
rosservice call /modify_output_sint16 $'slave: 1\nindex: 26\nvalue: 590'  
rosservice call /modify_output_sint16 $'slave: 1\nindex: 28\nvalue: 0' 
rosservice call /modify_output_sint16 $'slave: 1\nindex: 30\nvalue: 3'
rosservice call /modify_output_sint16 $'slave: 1\nindex: 32\nvalue: 100' 
rosservice call /modify_output_sint16 $'slave: 1\nindex: 34\nvalue: 0' 
rosservice call /modify_output_sint16 $'slave: 1\nindex: 36\nvalue: 0'

rosservice call /modify_output_bit $'slave: 2\nindex: 0\nsubindex: 0\nvalue: true'
rosservice call /modify_output_bit $'slave: 2\nindex: 0\nsubindex: 4\nvalue: true'
rosservice call /modify_output_sint16 $'slave: 2\nindex: 24\nvalue: 0'
rosservice call /modify_output_sint16 $'slave: 2\nindex: 26\nvalue: 590'  
rosservice call /modify_output_sint16 $'slave: 2\nindex: 28\nvalue: 0' 
rosservice call /modify_output_sint16 $'slave: 2\nindex: 30\nvalue: 3'
rosservice call /modify_output_sint16 $'slave: 2\nindex: 32\nvalue: 100' 
rosservice call /modify_output_sint16 $'slave: 2\nindex: 34\nvalue: 0' 
rosservice call /modify_output_sint16 $'slave: 2\nindex: 36\nvalue: 0'

rosservice call /modify_output_bit $'slave: 3\nindex: 0\nsubindex: 0\nvalue: true'
rosservice call /modify_output_bit $'slave: 3\nindex: 0\nsubindex: 4\nvalue: true'
rosservice call /modify_output_sint16 $'slave: 3\nindex: 24\nvalue: 0'
rosservice call /modify_output_sint16 $'slave: 3\nindex: 26\nvalue: 590'  
rosservice call /modify_output_sint16 $'slave: 3\nindex: 28\nvalue: 0' 
rosservice call /modify_output_sint16 $'slave: 3\nindex: 30\nvalue: 3'
rosservice call /modify_output_sint16 $'slave: 3\nindex: 32\nvalue: 100' 
rosservice call /modify_output_sint16 $'slave: 3\nindex: 34\nvalue: 0' 
rosservice call /modify_output_sint16 $'slave: 3\nindex: 36\nvalue: 0'

rosservice call /ethercat_communicatord "mode: 'start'"