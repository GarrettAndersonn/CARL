std_msgs/Header header

uint8 STATE_INIT=0
uint8 STATE_IDLE=1
uint8 STATE_DRIVING=2
uint8 STATE_SAFE_STOP=3
uint8 STATE_ESTOP=4

uint8 FAULT_HEARTBEAT_LOST=1
uint8 FAULT_ESTOP=2

uint8 state
uint8 faults
float32 applied_left
float32 applied_right
