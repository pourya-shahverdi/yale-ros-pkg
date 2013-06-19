#!/bin/bash

cd /media/archive/expeditions_year1_data

rosbag record --size=1024 --split \
\
/Blink_Server/cancel \
/Blink_Server/feedback \
/Blink_Server/goal \
/Blink_Server/result \
/Blink_Server/status \
/ExpressionMotion_Server/cancel \
/ExpressionMotion_Server/feedback \
/ExpressionMotion_Server/goal \
/ExpressionMotion_Server/result \
/ExpressionMotion_Server/status \
/IK_Server/cancel \
/IK_Server/feedback \
/IK_Server/goal \
/IK_Server/result \
/IK_Server/status \
/Lookat_Server/cancel \
/Lookat_Server/feedback \
/Lookat_Server/goal \
/Lookat_Server/result \
/Lookat_Server/status \
/SpeechPlay_Server/cancel \
/SpeechPlay_Server/feedback \
/SpeechPlay_Server/goal \
/SpeechPlay_Server/result \
/SpeechPlay_Server/status \
/Track_Server/cancel \
/Track_Server/feedback \
/Track_Server/goal \
/Track_Server/result \
/Track_Server/status \
/Viseme_Server/cancel \
/Viseme_Server/feedback \
/Viseme_Server/goal \
/Viseme_Server/result \
/Viseme_Server/status \
/audio \
/camera_info \
/change_topic \
/diagnostics \
/dragon_GUI/sleep \
/dragon_status \
/dragonbot_blink \
/dragonbot_expression \
/dragonbot_ik \
/dragonbot_lookat \
/dragonbot_viseme \
/image_raw/compressed \
/image_raw/compressed/parameter_descriptions \
/image_raw/compressed/parameter_updates \
/image_raw/theora \
/image_raw/theora/parameter_descriptions \
/image_raw/theora/parameter_updates \
/joint_position_publisher/pose_array \
/robotsound \
/rosout \
/rosout_agg \
/tf

#/image_raw
#/image_raw/compressedDepth \
#/image_raw/compressedDepth/parameter_descriptions \
#/image_raw/compressedDepth/parameter_updates \
