uint8[2] head
uint8 levelflag
uint8 framereserve

uint32[2] sn
uint32[2] version
uint16 bandwidth
uint8 mode 

uint8 gaittype		   
uint8 speedlevel			   
float32 footraiseheight		   
float32 bodyheight	   
float32[2] position 
float32[3] euler	   
float32[2] velocity 
float32 yawspeed				   
unitree_legged_msgs/BmsCmd bms
unitree_legged_msgs/Led[4] led
uint8[40] wirelessremote
uint32 reserve

uint32 crc