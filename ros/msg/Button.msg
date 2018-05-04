string tracker                      # Serial of tracker that produced the event
uint32 mask                         # Button type
uint32 MASK_TRIGGER_TOUCH    = 1    # Primary trigger touched
uint32 MASK_TRIGGER_CLICK    = 2    # Primary trigger clicked
uint32 MASK_GRIP_CLICK       = 4    # Side grip click
uint32 MASK_MENU_CLICK       = 8    # Menu button click
uint32 MASK_PAD_TOUCH        = 32   # Pad touched
uint32 MASK_PAD_CLICK        = 16   # Pad click
uint16 trigger_val                  # Current trigger value
int16 pad_x                         # Pad X value (horizontal)
int16 pad_y                         # Pad Y value (vertical)