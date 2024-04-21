
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
uint8 mode
float32 progress
uint8 gaittype		   
float32 footraiseheight		  
float32[3] position 
float32 bodyheight			  
float32[3] velocity 
float32 yawspeed				   
float32[4] rangeobstacle
unitree_legged_msgs/Cartesian[4] footposition2body 
unitree_legged_msgs/Cartesian[4] footspeed2body	
uint8[40] wirelessremote
uint32 reserve

uint32 crc