#ifndef MPU60X0_TYPES_H
#define MPU60X0_TYPES_H

#define GRAVITY			9.8f

#define MPU60X0_ADDR0		0x68
#define MPU60X0_ADDR1		0x69

#define PWR_MGMT_CYCLE_MODE	0x20
#define PWR_MGMT_SLEEP_MODE	0x40
#define PWR_MGMT_DISABLE_TEMP	0x08
#define PWR_MGMT_DISABLE_GYRO	0x07
#define PWR_MGMT_DISABLE_ACC	0x38
#define PWR_MGMT_WK_FREQ_1_25	0x0
#define PWR_MGMT_WK_FREQ_5	0x40
#define PWR_MGMT_WK_FREQ_20	0xC0
#define PWR_MGMT_WK_FREQ_40	0x80

#define FS_SEL_250		0x00
#define FS_SEL_500		0x08
#define FS_SEL_1000		0x10
#define FS_SEL_2000		0x18

#define FS_SEL_2		0x00
#define FS_SEL_4		0x08
#define FS_SEL_8		0x10
#define FS_SEL_16		0x18

#define REG_ACCEL_CONFIG	0x1C
#define REG_ACCEL_XOUT_H	0x3b

#define REG_GYRO_CONFIG		0x1B
#define REG_GYRO_XOUT_H		0x43

#define REG_TEMP_OUT_H          0x41

#define REG_PWR_MGMT_1		0x6B
#define REG_PWR_MGMT_2		0x6C



#endif  // MPU60X0_TYPES_H
