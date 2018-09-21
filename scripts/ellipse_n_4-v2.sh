#! /bin/bash
rostopic pub -1 /pdo_listener ighm_ros/ModifyPDOVariables '{slave_id: 255, index: 0, subindex: 0, value: [1], type: bool}'# state_machine_id
rostopic pub -1 /pdo_listener ighm_ros/ModifyPDOVariables '{slave_id: 255, index: 0, subindex: 4, value: [1], type: bool}'# blue_led_id
rostopic pub -1 /pdo_listener ighm_ros/ModifyPDOVariables '{slave_id: 255, index: 1, subindex: 0, value: [2], type: int8}'# TransitionTime
rostopic pub -1 /pdo_listener ighm_ros/ModifyPDOVariables '{slave_id: 255, index: 6, subindex: 0, value: [20], type: uint16}' # FilterBandwidth
rostopic pub -1 /pdo_listener ighm_ros/ModifyPDOVariables '{slave_id: 255, index: 12, subindex: 0, value: [100], type: int16}' # Kp100_knee
rostopic pub -1 /pdo_listener ighm_ros/ModifyPDOVariables '{slave_id: 255, index: 14, subindex: 0, value: [0], type: int16}' # Kd1000_knee
rostopic pub -1 /pdo_listener ighm_ros/ModifyPDOVariables '{slave_id: 255, index: 16, subindex: 0, value: [0], type: int16}' # Ki100_knee
rostopic pub -1 /pdo_listener ighm_ros/ModifyPDOVariables '{slave_id: 255, index: 18, subindex: 0, value: [100], type: int16}' # Kp100_hip
rostopic pub -1 /pdo_listener ighm_ros/ModifyPDOVariables '{slave_id: 255, index: 20, subindex: 0, value: [0], type: int16}' # Kd1000_hip
rostopic pub -1 /pdo_listener ighm_ros/ModifyPDOVariables '{slave_id: 255, index: 22, subindex: 0, value: [0], type: int16}' # Ki100_hip
rostopic pub -1 /pdo_listener ighm_ros/ModifyPDOVariables '{slave_id: 255, index: 24, subindex: 0, value: [0], type: int16}' # x_cntr_traj_id
rostopic pub -1 /pdo_listener ighm_ros/ModifyPDOVariables '{slave_id: 255, index: 26, subindex: 0, value: [590], type: int16}' # y_cntr_traj_id
rostopic pub -1 /pdo_listener ighm_ros/ModifyPDOVariables '{slave_id: 255, index: 28, subindex: 0, value: [3], type: int16}' # a_ellipse_id
rostopic pub -1 /pdo_listener ighm_ros/ModifyPDOVariables '{slave_id: 255, index: 30, subindex: 0, value: [0], type: int16}' # b_ellipse_id
rostopic pub -1 /pdo_listener ighm_ros/ModifyPDOVariables '{slave_id: 255, index: 32, subindex: 0, value: [30], type: int16}' # traj_freq_id
rostopic pub -1 /pdo_listener ighm_ros/ModifyPDOVariables '{slave_id: 255, index: 34, subindex: 0, value: [0], type: int16}' # phase_deg_id
rostopic pub -1 /pdo_listener ighm_ros/ModifyPDOVariables '{slave_id: 255, index: 36, subindex: 0, value: [0], type: int16}' # flatness_param_id

rosservice call /ethercat_communicatord "mode: 'start'"
