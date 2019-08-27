#ifndef IXI_API_TYPES_H_
#define IXI_API_TYPES_H_

typedef enum 
{
	IXI_USM1 = 0,		
	IXI_USM2 = 1,
	IXI_USM3 = 2,
	IXI_USM4 = 3,
	IXI_NUM_MANIPS

} IXI_MANIP_INDEX;

#define IXI_NUM_USM_JOINTS  10    // There are nine joints in the PSM three of the oint values are equal to each other. 
#define IXI_NUM_USM_JOINT_VALS 8  // This is the number of joint values we get from API. Look Xi Kinematics to understand.

#endif // !IXI_API_TYPES_H_
