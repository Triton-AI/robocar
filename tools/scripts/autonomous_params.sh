ros2 param set /race_decision_engine_node use_params false
ros2 param set /race_decision_engine_node use_perception false
# ros2 param set /basestation_race_control_node track_flag "fcy"
ros2 param set /basestation_race_control_node track_flag "green"
ros2 param set /basestation_race_control_node track_flag "g80"
ros2 param set /basestation_race_control_node vehicle_flags ["attacker"]
# ros2 param set /publish_dummy_joystick_control limit_auto_throttle false
# ros2 param set /publish_dummy_joystick_control use_manual_cmd false
# ros2 param set /race_decision_engine_node speed_limit.green 40.0 # Just an example of how to set speed
ros2 param set /race_decision_engine_node ttls.race_ttl_index 15

#  GHOST CAR PARAMS - TODO:  I have rather load this from a param file, will work on that later
ros2 param set /ghost_car_node target_speed 60.0
