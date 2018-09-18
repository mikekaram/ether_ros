#! /bin/bash
rostopic pub -1 /pdo_listener $'slave_id: 255\nindex: 0\nsubindex: 0\nvalue: 1\ntype: bool'# state_machine_id
rostopic pub -1 /pdo_listener $'slave_id: 255\nindex: 0\nsubindex: 4\nvalue: 1\ntype: bool'# blue_led_id
rostopic pub -1 /pdo_listener $'slave_id: 255\nindex: 1\nsubindex: 0\nvalue: 2\ntype: int8'# TransitionTime
rostopic pub -1 /pdo_listener $'slave_id: 255\nindex: 6\nsubindex: 0\nvalue: 20\ntype: uint16' # FilterBandwidth
rostopic pub -1 /pdo_listener $'slave_id: 255\nindex: 12\nsubindex: 0\nvalue: 100\ntype: int16' # Kp100_knee
rostopic pub -1 /pdo_listener $'slave_id: 255\nindex: 14\nsubindex: 0\nvalue: 0\ntype: int16' # Kd1000_knee
rostopic pub -1 /pdo_listener $'slave_id: 255\nindex: 16\nsubindex: 0\nvalue: 0\ntype: int16' # Ki100_knee
rostopic pub -1 /pdo_listener $'slave_id: 255\nindex: 18\nsubindex: 0\nvalue: 100\ntype: int16' # Kp100_hip
rostopic pub -1 /pdo_listener $'slave_id: 255\nindex: 20\nsubindex: 0\nvalue: 0\ntype: int16' # Kd1000_hip
rostopic pub -1 /pdo_listener $'slave_id: 255\nindex: 22\nsubindex: 0\nvalue: 0\ntype: int16' # Ki100_hip
rostopic pub -1 /pdo_listener $'slave_id: 255\nindex: 24\nsubindex: 0\nvalue: 0\ntype: int16' # x_cntr_traj_id
rostopic pub -1 /pdo_listener $'slave_id: 255\nindex: 26\nsubindex: 0\nvalue: 590\ntype: int16' # y_cntr_traj_id
rostopic pub -1 /pdo_listener $'slave_id: 255\nindex: 28\nsubindex: 0\nvalue: 3\ntype: int16' # a_ellipse_id
rostopic pub -1 /pdo_listener $'slave_id: 255\nindex: 30\nsubindex: 0\nvalue: 0\ntype: int16' # b_ellipse_id
rostopic pub -1 /pdo_listener $'slave_id: 255\nindex: 32\nsubindex: 0\nvalue: 30\ntype: int16' # traj_freq_id
rostopic pub -1 /pdo_listener $'slave_id: 255\nindex: 34\nsubindex: 0\nvalue: 0\ntype: int16' # phase_deg_id
rostopic pub -1 /pdo_listener $'slave_id: 255\nindex: 36\nsubindex: 0\nvalue: 0\ntype: int16' # flatness_param_id

rosservice call /ethercat_communicatord "mode: 'start'"
