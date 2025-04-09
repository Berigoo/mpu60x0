#ifndef MPU60X0_TYPES_H
#define MPU60X0_TYPES_H

#define GRAVITY 9.8f

#define MPU60X0_ADDR0 0x68
#define MPU60X0_ADDR1 0x69

#define FS_SEL_250 0x00
#define FS_SEL_500 0x08
#define FS_SEL_1000 0x10
#define FS_SEL_2000 0x18

#define FS_SEL_2 0x00
#define FS_SEL_4 0x08
#define FS_SEL_8 0x10
#define FS_SEL_16 0x18

#define REG_ACCEL_CONFIG 0x1C
#define REG_ACCEL_XOUT_H 0x3b
#define REG_ACCEL_XOUT_L 0x3c

#define REG_GYRO_CONFIG 0x1B
#define REG_GYRO_XOUT_H 0x43
#define REG_GYRO_XOUT_L 0x44

#define REG_PWR_MGMT_1 0x6B



#endif  // MPU60X0_TYPES_H
