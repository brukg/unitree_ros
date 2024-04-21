
uint8[2] head
uint8 levelflag
uint8 framereserve

uint32[2] sn
uint32[2] version
uint16 bandwidth
unitree_legged_msgs/Imu imu
unitree_legged_msgs/MotorState[20] motorstate
unitree_legged_msgs/BmsState bms
int16[4] footforce	
int16[4] footforceest	
uint32 tick						
uint8[40] wirelessremote 
uint32 reserve
uint32 crc

# old version aliengo does not have:
unitree_legged_msgs/Cartesian[4] eeforceraw
unitree_legged_msgs/Cartesian[4] eeforce          #it's a 1-dof force infact, but we use 3-dof here just for visualization 
unitree_legged_msgs/Cartesian position            # will delete
unitree_legged_msgs/Cartesian velocity            # will delete
unitree_legged_msgs/Cartesian velocity_w            # will delete

