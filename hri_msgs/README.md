### HRI messages

An HRI action contains a sequence of motion or waiting HRI Tasks

Each HriTask can be associated to an HriInteraction describing the interaction type and conditions
for which the interaction occurs based on HriEvents.
The HriTask starts either immediately or when the HriInteraction is met
The HriTask ends either with end of motion, or HriInteraction met, or HriInteraction not met anymore, 
or timeout occured while still waiting for a condition.

HriInteraction can be of several types, with one or more threshold to be reached or value to match.
The types can be selected and combined in a bitwise fashion (= summing up the different type id values)

 * Matching ID: If event_id matches Event id, the condition is valid
 * Matching Voice: If voice_event_text matches voice_trigger_text, the condition is valid
 * Matching Visual: If visual_event_text matches visual_trigger_text, the condition is valid
 * Reached Physic threshold: If one of the physic_event_value is higher than the corresponding physic_threshold_value, the condition is valid
 * Reached Contact threshold: If the contact_event_value is higher than the contact_threshold_value, the condition is valid

HriEvents contain Interaction status relevant for potentially triggering the HriInteraction
The type of the HriEvent permits to easy filter HriEvents for only awaited types (binary check on the type uint8 bits)
Several HriEvents might be needed to trigger the HriInteraction, depending on if ALL or ANY condition must be met







