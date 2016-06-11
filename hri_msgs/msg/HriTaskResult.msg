# Describes an Hri Task Result
#  success is true if task completed, with or without event trigger
#  even_triggered and event_id tells if and which event triggered the result
#  error_code

bool success

bool event_triggered
int32 event_id

uint8 error_code
uint8 NO_ERROR=0
uint8 TIMED_OUT=1
uint8 INVALID_GOAL=2
uint8 INVALID_TASK=3
uint8 INVALID_EVENT=4
uint8 MOTION_ERROR=5

uint32 last_seq
hri_msgs/HriTaskPhase last_phase


