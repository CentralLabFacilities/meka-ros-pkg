# Describes the type of awaited HriInteractionEvent
#  any event matching event_id
#  a contact event that is above the given threshold
#  a physic event that is above the physic threshold
#  a voice trigger with matching string (a word or a sentence)
#  a visual trigger with matching string (facedetected, color detected, etc...)
# either ANY or ALL condition should be filled
# UNTIL : action should start immediatly and continue UNTIL event matching
# IF: action should start IF event matches and end normally
# WHILE: action should start when event matches and end when event does not match anymore
# timeout is how long to wait for this event (0 means no timeout)

uint8 type
uint8 ID=1
uint8 CONTACT=2
uint8 PHYSIC=4
uint8 VOICE=8
uint8 VISUAL=16

uint8 condition
uint8 ANY=0
uint8 ALL=1

uint8 do
uint8 UNTIL=0
uint8 IF=1
uint8 WHILE=2

duration timeout

int32 event_id
geometry_msgs/Wrench physic_threshold_value
std_msgs/Float64 contact_threshold_value
string voice_trigger_text
string visual_trigger_text



