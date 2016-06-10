# Describes an Hri Interaction Event
#  id as the simplest description
#  type for easier filtering of events
#  data fields containing the event description

int32 id
uint8 type
uint8 ID=1
uint8 CONTACT=2
uint8 PHYSIC=4
uint8 VOICE=8
uint8 VISUAL=16

geometry_msgs/Wrench physic_event_value
std_msgs/Float64 contact_event_value
string voice_event_text
string visual_event_text



