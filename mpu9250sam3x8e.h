/*
project_Tinnakon_Rover 32 bit Arduino Due
Tinnakon_Rover
by: tinnakon kheowree 
0860540582
tinnakon_za@hotmail.com
tinnakonza@gmail.com
http://quad3d-tin.lnwshop.com/
https://www.facebook.com/tinnakonza
*/
#define XAXIS 0
#define YAXIS 1
#define ZAXIS 2
float gyroScaleFactor = radians(2000.0/32668.0);//32768.0 32730 , +-32756  32768.0
float accelScaleFactor = 9.80665;//9.80665/4100.62 9.81/8192.09 9.81 / 8205  
uint8_t gyroSamples = 0;
uint8_t gyroSamples2 = 0;
uint8_t accSamples = 0;

float gyroSum[3];
float accelSum[3];
float AccXm,AccYm,AccZm;
float AccX,AccY,AccZ;
float AccXf,AccYf,AccZf;
float AccX2,AccY2,AccZ2;
float GyroXf,GyroYf,GyroZf;
float gyro_offsetX,gyro_offsetY,gyro_offsetZ,acc_offsetX,acc_offsetY,acc_offsetZ;
float acc_offsetZ2 = 9.81;
float GyroX,GyroY,GyroZ,GyroTemp;
float GyroX2,GyroY2,GyroZ2;
float Accel[3] = {0.0,0.0,0.0};
float filteredAccel[3] = {0.0,0.0,0.0};

#define HMC5883_Address 0x1E // I2C adress: 0x3C (8bit)   0x1E (7bit)
int16_t MagX1,MagY1,MagZ1;
float MagXf,MagYf,MagZf;
float c_magnetom_x;
float c_magnetom_y;
float c_magnetom_z;

//sensor MPU6500 -------------------------------------
#define NCS_PIN 50
#define N_BYTES 14 // number of bytes in data packet
const int nCS = NCS_PIN;
int16_t accelRaw[3];
int16_t gyroRaw[3];
int16_t Tempgyro;
unsigned short temp = 0;
unsigned short n_samples = 0;
unsigned short fifo_count = 0;
unsigned short max_fifo_count = 0;
unsigned short max_fifo_count2 = 0;
unsigned short fifo_err = 0;
unsigned char bytes[ N_BYTES ];

#define MPU6500_ADDRESS_AD0_LOW     0x68 // address pin low (GND), default for InvenSense evaluation board
#define MPU6500_ADDRESS_AD0_HIGH    0x69 // address pin high (VCC)
#define MPU6500_DEFAULT_ADDRESS     MPU6500_ADDRESS_AD0_LOW

#define MPU6500_RA_XG_OFFS_TC       0x00 //[7] PWR_MODE, [6:1] XG_OFFS_TC, [0] OTP_BNK_VLD
#define MPU6500_RA_YG_OFFS_TC       0x01 //[7] PWR_MODE, [6:1] YG_OFFS_TC, [0] OTP_BNK_VLD
#define MPU6500_RA_ZG_OFFS_TC       0x02 //[7] PWR_MODE, [6:1] ZG_OFFS_TC, [0] OTP_BNK_VLD
#define MPU6500_RA_X_FINE_GAIN      0x03 //[7:0] X_FINE_GAIN
#define MPU6500_RA_Y_FINE_GAIN      0x04 //[7:0] Y_FINE_GAIN
#define MPU6500_RA_Z_FINE_GAIN      0x05 //[7:0] Z_FINE_GAIN
#define MPU6500_RA_XA_OFFS_H        0x06 //[15:0] XA_OFFS
#define MPU6500_RA_XA_OFFS_L_TC     0x07
#define MPU6500_RA_YA_OFFS_H        0x08 //[15:0] YA_OFFS
#define MPU6500_RA_YA_OFFS_L_TC     0x09
#define MPU6500_RA_ZA_OFFS_H        0x0A //[15:0] ZA_OFFS
#define MPU6500_RA_ZA_OFFS_L_TC     0x0B
#define MPU6500_RA_XG_OFFS_USRH     0x13 //[15:0] XG_OFFS_USR
#define MPU6500_RA_XG_OFFS_USRL     0x14
#define MPU6500_RA_YG_OFFS_USRH     0x15 //[15:0] YG_OFFS_USR
#define MPU6500_RA_YG_OFFS_USRL     0x16
#define MPU6500_RA_ZG_OFFS_USRH     0x17 //[15:0] ZG_OFFS_USR
#define MPU6500_RA_ZG_OFFS_USRL     0x18
#define MPU6500_RA_SMPLRT_DIV       0x19
#define MPU6500_RA_CONFIG           0x1A
#define MPU6500_RA_GYRO_CONFIG      0x1B
#define MPU6500_RA_ACCEL_CONFIG     0x1C
#define MPU6500_RA_FF_THR           0x1D
#define MPU6500_RA_FF_DUR           0x1E
#define MPU6500_RA_MOT_THR          0x1F
#define MPU6500_RA_MOT_DUR          0x20
#define MPU6500_RA_ZRMOT_THR        0x21
#define MPU6500_RA_ZRMOT_DUR        0x22
#define MPU6500_RA_FIFO_EN          0x23
#define MPU6500_RA_I2C_MST_CTRL     0x24
#define MPU6500_RA_I2C_SLV0_ADDR    0x25
#define MPU6500_RA_I2C_SLV0_REG     0x26
#define MPU6500_RA_I2C_SLV0_CTRL    0x27
#define MPU6500_RA_I2C_SLV1_ADDR    0x28
#define MPU6500_RA_I2C_SLV1_REG     0x29
#define MPU6500_RA_I2C_SLV1_CTRL    0x2A
#define MPU6500_RA_I2C_SLV2_ADDR    0x2B
#define MPU6500_RA_I2C_SLV2_REG     0x2C
#define MPU6500_RA_I2C_SLV2_CTRL    0x2D
#define MPU6500_RA_I2C_SLV3_ADDR    0x2E
#define MPU6500_RA_I2C_SLV3_REG     0x2F
#define MPU6500_RA_I2C_SLV3_CTRL    0x30
#define MPU6500_RA_I2C_SLV4_ADDR    0x31
#define MPU6500_RA_I2C_SLV4_REG     0x32
#define MPU6500_RA_I2C_SLV4_DO      0x33
#define MPU6500_RA_I2C_SLV4_CTRL    0x34
#define MPU6500_RA_I2C_SLV4_DI      0x35
#define MPU6500_RA_I2C_MST_STATUS   0x36
#define MPU6500_RA_INT_PIN_CFG      0x37
#define MPU6500_RA_INT_ENABLE       0x38
#define MPU6500_RA_DMP_INT_STATUS   0x39
#define MPU6500_RA_INT_STATUS       0x3A
#define MPU6500_RA_ACCEL_XOUT_H     0x3B
#define MPU6500_RA_ACCEL_XOUT_L     0x3C
#define MPU6500_RA_ACCEL_YOUT_H     0x3D
#define MPU6500_RA_ACCEL_YOUT_L     0x3E
#define MPU6500_RA_ACCEL_ZOUT_H     0x3F
#define MPU6500_RA_ACCEL_ZOUT_L     0x40
#define MPU6500_RA_TEMP_OUT_H       0x41
#define MPU6500_RA_TEMP_OUT_L       0x42
#define MPU6500_RA_GYRO_XOUT_H      0x43
#define MPU6500_RA_GYRO_XOUT_L      0x44
#define MPU6500_RA_GYRO_YOUT_H      0x45
#define MPU6500_RA_GYRO_YOUT_L      0x46
#define MPU6500_RA_GYRO_ZOUT_H      0x47
#define MPU6500_RA_GYRO_ZOUT_L      0x48
#define MPU6500_RA_EXT_SENS_DATA_00 0x49
#define MPU6500_RA_EXT_SENS_DATA_01 0x4A
#define MPU6500_RA_EXT_SENS_DATA_02 0x4B
#define MPU6500_RA_EXT_SENS_DATA_03 0x4C
#define MPU6500_RA_EXT_SENS_DATA_04 0x4D
#define MPU6500_RA_EXT_SENS_DATA_05 0x4E
#define MPU6500_RA_EXT_SENS_DATA_06 0x4F
#define MPU6500_RA_EXT_SENS_DATA_07 0x50
#define MPU6500_RA_EXT_SENS_DATA_08 0x51
#define MPU6500_RA_EXT_SENS_DATA_09 0x52
#define MPU6500_RA_EXT_SENS_DATA_10 0x53
#define MPU6500_RA_EXT_SENS_DATA_11 0x54
#define MPU6500_RA_EXT_SENS_DATA_12 0x55
#define MPU6500_RA_EXT_SENS_DATA_13 0x56
#define MPU6500_RA_EXT_SENS_DATA_14 0x57
#define MPU6500_RA_EXT_SENS_DATA_15 0x58
#define MPU6500_RA_EXT_SENS_DATA_16 0x59
#define MPU6500_RA_EXT_SENS_DATA_17 0x5A
#define MPU6500_RA_EXT_SENS_DATA_18 0x5B
#define MPU6500_RA_EXT_SENS_DATA_19 0x5C
#define MPU6500_RA_EXT_SENS_DATA_20 0x5D
#define MPU6500_RA_EXT_SENS_DATA_21 0x5E
#define MPU6500_RA_EXT_SENS_DATA_22 0x5F
#define MPU6500_RA_EXT_SENS_DATA_23 0x60
#define MPU6500_RA_MOT_DETECT_STATUS    0x61
#define MPU6500_RA_I2C_SLV0_DO      0x63
#define MPU6500_RA_I2C_SLV1_DO      0x64
#define MPU6500_RA_I2C_SLV2_DO      0x65
#define MPU6500_RA_I2C_SLV3_DO      0x66
#define MPU6500_RA_I2C_MST_DELAY_CTRL   0x67
#define MPU6500_RA_SIGNAL_PATH_RESET    0x68
#define MPU6500_RA_MOT_DETECT_CTRL      0x69
#define MPU6500_RA_USER_CTRL        0x6A
#define MPU6500_RA_PWR_MGMT_1       0x6B
#define MPU6500_RA_PWR_MGMT_2       0x6C
#define MPU6500_RA_BANK_SEL         0x6D
#define MPU6500_RA_MEM_START_ADDR   0x6E
#define MPU6500_RA_MEM_R_W          0x6F
#define MPU6500_RA_DMP_CFG_1        0x70
#define MPU6500_RA_DMP_CFG_2        0x71
#define MPU6500_RA_FIFO_COUNTH      0x72
#define MPU6500_RA_FIFO_COUNTL      0x73
#define MPU6500_RA_FIFO_R_W         0x74
#define MPU6500_RA_WHO_AM_I         0x75
#define MPU6500_RA_SIGNAL_PATH_RESET 104
#define MPU6500_RA_PWR_MGMT_1       107
#define MPU6500_RA_PWR_MGMT_2       107
#define MPU6500_RA_FIFO_COUNT_H     114
#define MPU6500_RA_FIFO_COUNT_L     115

#define MPU6500_TC_PWR_MODE_BIT     7
#define MPU6500_TC_OFFSET_BIT       6
#define MPU6500_TC_OFFSET_LENGTH    6
#define MPU6500_TC_OTP_BNK_VLD_BIT  0

#define MPU6500_VDDIO_LEVEL_VLOGIC  0
#define MPU6500_VDDIO_LEVEL_VDD     1

#define MPU6500_CFG_EXT_SYNC_SET_BIT    5
#define MPU6500_CFG_EXT_SYNC_SET_LENGTH 3
#define MPU6500_CFG_DLPF_CFG_BIT    2
#define MPU6500_CFG_DLPF_CFG_LENGTH 3

#define MPU6500_EXT_SYNC_DISABLED       0x0
#define MPU6500_EXT_SYNC_TEMP_OUT_L     0x1
#define MPU6500_EXT_SYNC_GYRO_XOUT_L    0x2
#define MPU6500_EXT_SYNC_GYRO_YOUT_L    0x3
#define MPU6500_EXT_SYNC_GYRO_ZOUT_L    0x4
#define MPU6500_EXT_SYNC_ACCEL_XOUT_L   0x5
#define MPU6500_EXT_SYNC_ACCEL_YOUT_L   0x6
#define MPU6500_EXT_SYNC_ACCEL_ZOUT_L   0x7

#define MPU6500_DLPF_BW_256         0x00
#define MPU6500_DLPF_BW_188         0x01
#define MPU6500_DLPF_BW_98          0x02
#define MPU6500_DLPF_BW_42          0x03
#define MPU6500_DLPF_BW_20          0x04
#define MPU6500_DLPF_BW_10          0x05
#define MPU6500_DLPF_BW_5           0x06

#define MPU6500_GCONFIG_FS_SEL_BIT      4
#define MPU6500_GCONFIG_FS_SEL_LENGTH   2

#define MPU6500_GYRO_FS_250         0x00
#define MPU6500_GYRO_FS_500         0x01
#define MPU6500_GYRO_FS_1000        0x02
#define MPU6500_GYRO_FS_2000        0x03

#define MPU6500_ACONFIG_XA_ST_BIT           7
#define MPU6500_ACONFIG_YA_ST_BIT           6
#define MPU6500_ACONFIG_ZA_ST_BIT           5
#define MPU6500_ACONFIG_AFS_SEL_BIT         4
#define MPU6500_ACONFIG_AFS_SEL_LENGTH      2
#define MPU6500_ACONFIG_ACCEL_HPF_BIT       2
#define MPU6500_ACONFIG_ACCEL_HPF_LENGTH    3

#define MPU6500_ACCEL_FS_2          0x00
#define MPU6500_ACCEL_FS_4          0x01
#define MPU6500_ACCEL_FS_8          0x02
#define MPU6500_ACCEL_FS_16         0x03

#define MPU6500_DHPF_RESET          0x00
#define MPU6500_DHPF_5              0x01
#define MPU6500_DHPF_2P5            0x02
#define MPU6500_DHPF_1P25           0x03
#define MPU6500_DHPF_0P63           0x04
#define MPU6500_DHPF_HOLD           0x07

#define MPU6500_TEMP_FIFO_EN_BIT    7
#define MPU6500_XG_FIFO_EN_BIT      6
#define MPU6500_YG_FIFO_EN_BIT      5
#define MPU6500_ZG_FIFO_EN_BIT      4
#define MPU6500_ACCEL_FIFO_EN_BIT   3
#define MPU6500_SLV2_FIFO_EN_BIT    2
#define MPU6500_SLV1_FIFO_EN_BIT    1
#define MPU6500_SLV0_FIFO_EN_BIT    0

#define MPU6500_FIFO_EN_TEMP 0x80
#define MPU6500_FIFO_EN_GYROX 0x40
#define MPU6500_FIFO_EN_GYROY 0x20
#define MPU6500_FIFO_EN_GYROZ 0x10
#define MPU6500_FIFO_EN_GYRO 0x70
#define MPU6500_FIFO_EN_ACC 0x08

#define MPU6500_MULT_MST_EN_BIT     7
#define MPU6500_WAIT_FOR_ES_BIT     6
#define MPU6500_SLV_3_FIFO_EN_BIT   5
#define MPU6500_I2C_MST_P_NSR_BIT   4
#define MPU6500_I2C_MST_CLK_BIT     3
#define MPU6500_I2C_MST_CLK_LENGTH  4

#define MPU6500_CLOCK_DIV_348       0x0
#define MPU6500_CLOCK_DIV_333       0x1
#define MPU6500_CLOCK_DIV_320       0x2
#define MPU6500_CLOCK_DIV_308       0x3
#define MPU6500_CLOCK_DIV_296       0x4
#define MPU6500_CLOCK_DIV_286       0x5
#define MPU6500_CLOCK_DIV_276       0x6
#define MPU6500_CLOCK_DIV_267       0x7
#define MPU6500_CLOCK_DIV_258       0x8
#define MPU6500_CLOCK_DIV_500       0x9
#define MPU6500_CLOCK_DIV_471       0xA
#define MPU6500_CLOCK_DIV_444       0xB
#define MPU6500_CLOCK_DIV_421       0xC
#define MPU6500_CLOCK_DIV_400       0xD
#define MPU6500_CLOCK_DIV_381       0xE
#define MPU6500_CLOCK_DIV_364       0xF

#define MPU6500_I2C_SLV_RW_BIT      7
#define MPU6500_I2C_SLV_ADDR_BIT    6
#define MPU6500_I2C_SLV_ADDR_LENGTH 7
#define MPU6500_I2C_SLV_EN_BIT      7
#define MPU6500_I2C_SLV_BYTE_SW_BIT 6
#define MPU6500_I2C_SLV_REG_DIS_BIT 5
#define MPU6500_I2C_SLV_GRP_BIT     4
#define MPU6500_I2C_SLV_LEN_BIT     3
#define MPU6500_I2C_SLV_LEN_LENGTH  4

#define MPU6500_I2C_SLV4_RW_BIT         7
#define MPU6500_I2C_SLV4_ADDR_BIT       6
#define MPU6500_I2C_SLV4_ADDR_LENGTH    7
#define MPU6500_I2C_SLV4_EN_BIT         7
#define MPU6500_I2C_SLV4_INT_EN_BIT     6
#define MPU6500_I2C_SLV4_REG_DIS_BIT    5
#define MPU6500_I2C_SLV4_MST_DLY_BIT    4
#define MPU6500_I2C_SLV4_MST_DLY_LENGTH 5

#define MPU6500_MST_PASS_THROUGH_BIT    7
#define MPU6500_MST_I2C_SLV4_DONE_BIT   6
#define MPU6500_MST_I2C_LOST_ARB_BIT    5
#define MPU6500_MST_I2C_SLV4_NACK_BIT   4
#define MPU6500_MST_I2C_SLV3_NACK_BIT   3
#define MPU6500_MST_I2C_SLV2_NACK_BIT   2
#define MPU6500_MST_I2C_SLV1_NACK_BIT   1
#define MPU6500_MST_I2C_SLV0_NACK_BIT   0

#define MPU6500_INTCFG_INT_LEVEL_BIT        7
#define MPU6500_INTCFG_INT_OPEN_BIT         6
#define MPU6500_INTCFG_LATCH_INT_EN_BIT     5
#define MPU6500_INTCFG_INT_RD_CLEAR_BIT     4
#define MPU6500_INTCFG_FSYNC_INT_LEVEL_BIT  3
#define MPU6500_INTCFG_FSYNC_INT_EN_BIT     2
#define MPU6500_INTCFG_I2C_BYPASS_EN_BIT    1
#define MPU6500_INTCFG_CLKOUT_EN_BIT        0

#define MPU6500_INTMODE_ACTIVEHIGH  0x00
#define MPU6500_INTMODE_ACTIVELOW   0x01

#define MPU6500_INTDRV_PUSHPULL     0x00
#define MPU6500_INTDRV_OPENDRAIN    0x01

#define MPU6500_INTLATCH_50USPULSE  0x00
#define MPU6500_INTLATCH_WAITCLEAR  0x01

#define MPU6500_INTCLEAR_STATUSREAD 0x00
#define MPU6500_INTCLEAR_ANYREAD    0x01

#define MPU6500_INTERRUPT_FF_BIT            7
#define MPU6500_INTERRUPT_MOT_BIT           6
#define MPU6500_INTERRUPT_ZMOT_BIT          5
#define MPU6500_INTERRUPT_FIFO_OFLOW_BIT    4
#define MPU6500_INTERRUPT_I2C_MST_INT_BIT   3
#define MPU6500_INTERRUPT_PLL_RDY_INT_BIT   2
#define MPU6500_INTERRUPT_DMP_INT_BIT       1
#define MPU6500_INTERRUPT_DATA_RDY_BIT      0

// TODO: figure out what these actually do
// UMPL source code is not very obivous
#define MPU6500_DMPINT_5_BIT            5
#define MPU6500_DMPINT_4_BIT            4
#define MPU6500_DMPINT_3_BIT            3
#define MPU6500_DMPINT_2_BIT            2
#define MPU6500_DMPINT_1_BIT            1
#define MPU6500_DMPINT_0_BIT            0

#define MPU6500_MOTION_MOT_XNEG_BIT     7
#define MPU6500_MOTION_MOT_XPOS_BIT     6
#define MPU6500_MOTION_MOT_YNEG_BIT     5
#define MPU6500_MOTION_MOT_YPOS_BIT     4
#define MPU6500_MOTION_MOT_ZNEG_BIT     3
#define MPU6500_MOTION_MOT_ZPOS_BIT     2
#define MPU6500_MOTION_MOT_ZRMOT_BIT    0

#define MPU6500_DELAYCTRL_DELAY_ES_SHADOW_BIT   7
#define MPU6500_DELAYCTRL_I2C_SLV4_DLY_EN_BIT   4
#define MPU6500_DELAYCTRL_I2C_SLV3_DLY_EN_BIT   3
#define MPU6500_DELAYCTRL_I2C_SLV2_DLY_EN_BIT   2
#define MPU6500_DELAYCTRL_I2C_SLV1_DLY_EN_BIT   1
#define MPU6500_DELAYCTRL_I2C_SLV0_DLY_EN_BIT   0

#define MPU6500_PATHRESET_GYRO_RESET_BIT    2
#define MPU6500_PATHRESET_ACCEL_RESET_BIT   1
#define MPU6500_PATHRESET_TEMP_RESET_BIT    0

#define MPU6500_DETECT_ACCEL_ON_DELAY_BIT       5
#define MPU6500_DETECT_ACCEL_ON_DELAY_LENGTH    2
#define MPU6500_DETECT_FF_COUNT_BIT             3
#define MPU6500_DETECT_FF_COUNT_LENGTH          2
#define MPU6500_DETECT_MOT_COUNT_BIT            1
#define MPU6500_DETECT_MOT_COUNT_LENGTH         2

#define MPU6500_DETECT_DECREMENT_RESET  0x0
#define MPU6500_DETECT_DECREMENT_1      0x1
#define MPU6500_DETECT_DECREMENT_2      0x2
#define MPU6500_DETECT_DECREMENT_4      0x3

#define MPU6500_USERCTRL_DMP_EN_BIT             7
#define MPU6500_USERCTRL_FIFO_EN_BIT            6
#define MPU6500_USERCTRL_I2C_MST_EN_BIT         5
#define MPU6500_USERCTRL_I2C_IF_DIS_BIT         4
#define MPU6500_USERCTRL_DMP_RESET_BIT          3
#define MPU6500_USERCTRL_FIFO_RESET_BIT         2
#define MPU6500_USERCTRL_I2C_MST_RESET_BIT      1
#define MPU6500_USERCTRL_SIG_COND_RESET_BIT     0

#define MPU6500_PWR1_DEVICE_RESET_BIT   7
#define MPU6500_PWR1_SLEEP_BIT          6
#define MPU6500_PWR1_CYCLE_BIT          5
#define MPU6500_PWR1_TEMP_DIS_BIT       3
#define MPU6500_PWR1_CLKSEL_BIT         2
#define MPU6500_PWR1_CLKSEL_LENGTH      3

#define MPU6500_CLOCK_INTERNAL          0x00
#define MPU6500_CLOCK_PLL_XGYRO         0x01
#define MPU6500_CLOCK_PLL_YGYRO         0x02
#define MPU6500_CLOCK_PLL_ZGYRO         0x03
#define MPU6500_CLOCK_PLL_EXT32K        0x04
#define MPU6500_CLOCK_PLL_EXT19M        0x05
#define MPU6500_CLOCK_KEEP_RESET        0x07

#define MPU6500_PWR2_LP_WAKE_CTRL_BIT       7
#define MPU6500_PWR2_LP_WAKE_CTRL_LENGTH    2
#define MPU6500_PWR2_STBY_XA_BIT            5
#define MPU6500_PWR2_STBY_YA_BIT            4
#define MPU6500_PWR2_STBY_ZA_BIT            3
#define MPU6500_PWR2_STBY_XG_BIT            2
#define MPU6500_PWR2_STBY_YG_BIT            1
#define MPU6500_PWR2_STBY_ZG_BIT            0

#define MPU6500_WAKE_FREQ_1P25      0x0
#define MPU6500_WAKE_FREQ_2P5       0x1
#define MPU6500_WAKE_FREQ_5         0x2
#define MPU6500_WAKE_FREQ_10        0x3

#define MPU6500_BANKSEL_PRFTCH_EN_BIT       6
#define MPU6500_BANKSEL_CFG_USER_BANK_BIT   5
#define MPU6500_BANKSEL_MEM_SEL_BIT         4
#define MPU6500_BANKSEL_MEM_SEL_LENGTH      5

#define MPU6500_WHO_AM_I_BIT        6
#define MPU6500_WHO_AM_I_LENGTH     6

#define MPU6500_DMP_MEMORY_BANKS        8
#define MPU6500_DMP_MEMORY_BANK_SIZE    256
#define MPU6500_DMP_MEMORY_CHUNK_SIZE   16

//sensor ---------------------
#define applyDeadband(value, deadband)  \
  if(fabs(value) < deadband) {           \
    value = 0.0;                        \
  } else if(value > 0.0){               \
    value -= deadband;                  \
  } else if(value < 0.0){               \
    value += deadband;                  \
  }
  
//////////////////////////////////////////////////
byte SPI_read_register( byte reg )
{
  digitalWrite( NCS_PIN, LOW );
  SPI.transfer( nCS, reg | 0x80, SPI_CONTINUE ); // reg | 0x80 to denote read
  byte read_value = SPI.transfer( nCS, 0x00 ); // write 8-bits zero
// read the 8-bits coming back from reg at the same time.
  digitalWrite( NCS_PIN, HIGH );
  return read_value;
}
void SPI_write_register( byte reg, byte value )
{
  digitalWrite( NCS_PIN, LOW );
  SPI.transfer( nCS, reg, SPI_CONTINUE );
  SPI.transfer( nCS, value );
  digitalWrite( NCS_PIN, HIGH );
}
static int16_t spi_transfer_16(void)
{
    uint8_t byte_H, byte_L;
    byte_H = SPI.transfer(0);
    byte_L = SPI.transfer(0);
    return (((int16_t)byte_H)<<8) | byte_L;
}
void mpu6500_initialize()
{
  pinMode( NCS_PIN, OUTPUT );
  digitalWrite( NCS_PIN, HIGH );
  digitalWrite( NCS_PIN, LOW );
  digitalWrite( NCS_PIN, HIGH );
  SPI.begin( nCS );
  SPI.setDataMode( nCS, SPI_MODE0 );
  SPI.setBitOrder( nCS, MSBFIRST );
  SPI.setClockDivider( nCS, 84 ); // 1MHz
  digitalWrite( NCS_PIN, HIGH );
  digitalWrite( NCS_PIN, LOW );
  digitalWrite( NCS_PIN, HIGH );
  SPI_write_register( MPU6500_RA_PWR_MGMT_1, 0x80 );
  delay(100); // page 42 - delay 100ms
    // reset gyro, accel, temp 
  SPI_write_register( MPU6500_RA_SIGNAL_PATH_RESET, 0x07);//0x05
   delay(100); // page 42 - delay 100ms
  // set SPI mode by setting I2C_IF_DIS
  // reset DMP, FIFO, SIG
  //SPI_write_register( MPU6500_RA_USER_CTRL, 0x10 | 0x8 | 0x4 | 0x1 );
  SPI_write_register( MPU6500_RA_USER_CTRL, 0x03);
  delay(100);
  byte id;
  id = SPI_read_register( MPU6500_RA_WHO_AM_I );
  Serial.print( "id should be 0x70: " );
  Serial.println( id, HEX );
  // Set DLPF_CFG to 1: 1kHz Gyro sampling, 184Hz bandwidth
  //SPI_write_register( MPU6500_RA_CONFIG, 0x01 );
  // Default: 1kHz Accel sampling, 480Hz cutoff
    // set sample rate
    SPI_write_register(MPU6500_RA_SMPLRT_DIV, 0x01);
    delay(1);
    // set low pass filter
    SPI_write_register(MPU6500_RA_CONFIG, MPU6500_DLPF_BW_42);//5, 42, 98, 188
    delay(1);
    SPI_write_register(MPU6500_RA_GYRO_CONFIG, MPU6500_GYRO_FS_2000);  // Gyro scale 2000º/s
    delay(1);
    SPI_write_register(MPU6500_RA_ACCEL_CONFIG, 0x10);//Accel scale 8g (4096 LSB/g)
    delay(1);
    // configure interrupt to fire when new data arrives
    //SPI_write_register(MPU6500_RA_INT_ENABLE, BIT_RAW_RDY_EN);
    delay(1);
    // clear interrupt on any read
    //SPI_write_register(MPU6500_RA_INT_PIN_CFG, BIT_INT_RD_CLEAR);
    delay(1);
    //attachInterrupt(6,data_interrupt,RISING);
    
  // enable temperature, gyro, and accelerometer output
  //SPI_write_register( MPU6500_RA_FIFO_EN,MPU6500_FIFO_EN_ACC | MPU6500_FIFO_EN_TEMP | MPU6500_FIFO_EN_GYRO );
  // enable FIFO
  // set SPI mode by setting I2C_IF_DIS
  //SPI_write_register( MPU6500_RA_USER_CTRL, 0x40 | 0x10 );
  // speed up to take data
  //SPI.setClockDivider( nCS, 8 ); // 10.5MHz; 7 sometimes works
  Serial.println( "mpu6500_initialize_SPI" );
}

void MPU6500Read( void )
{
      digitalWrite( NCS_PIN, LOW );
      SPI.transfer(MPU6500_RA_ACCEL_XOUT_H  | 0x80);//0x40 // SPI read,
      accelRaw[XAXIS] = spi_transfer_16() * -1;
      accelRaw[YAXIS] = spi_transfer_16();
      accelRaw[ZAXIS] = spi_transfer_16();
      Tempgyro = spi_transfer_16();
      gyroRaw[XAXIS] = spi_transfer_16();
      gyroRaw[YAXIS] = spi_transfer_16() * -1;
      gyroRaw[ZAXIS] = spi_transfer_16() * -1;
      digitalWrite( NCS_PIN, HIGH );
}
void mpu6500_Gyro_Values()
{
     int i = 0;
     byte result[6];
  while(Wire.available())    
  { 
    result[i] = Wire.read(); 
    i++;
  }
  Wire.endTransmission();
    gyroRaw[XAXIS] = ((result[0] << 8) | result[1]);//-12     -3
    gyroRaw[YAXIS] = ((result[2] << 8) | result[3])*-1;//37      15
    gyroRaw[ZAXIS] = ((result[4] << 8) | result[5])*-1;//11    5
}	
void mpu6500_Accel_Values()
{
    int i = 0;
    byte result[6];
  while(Wire.available())    
  { 
    result[i] = Wire.read(); 
    i++;
  }
  Wire.endTransmission();
    accelRaw[XAXIS] = ((result[0] << 8) | result[1])*-1;//+ 105 + 100  max=4054.46, min=-4149.48
    accelRaw[YAXIS] = ((result[2] << 8) | result[3]);// - 45 - 35      max=4129.57, min=-4070.64
    accelRaw[ZAXIS] = ((result[4] << 8) | result[5]);// + 150 + 170    max=4014.73 , min=-4165.13
     // adjust for  acc axis offsets/sensitivity differences by scaling to +/-1 g range
  AccXm = ((float)(accelRaw[XAXIS] - A_X_MIN) / (A_X_MAX - A_X_MIN))*2.0 - 1.0;
  AccYm = ((float)(accelRaw[YAXIS] - A_Y_MIN) / (A_Y_MAX - A_Y_MIN))*2.0 - 1.0;
  AccZm = ((float)(accelRaw[ZAXIS] - A_Z_MIN) / (A_Z_MAX - A_Z_MIN))*2.0 - 1.0;
}
void MagHMC5883Int()
{
  Wire.beginTransmission(HMC5883_Address); //open communication with HMC5883
  Wire.write(0x00); //Configuration Register A
  Wire.write(0x70); //num samples: 8 ; output rate: 15Hz ; normal measurement mode
  Wire.endTransmission();
  delay(10);
  Wire.beginTransmission(HMC5883_Address); //open communication with HMC5883
  Wire.write(0x01); //Configuration Register B
  Wire.write(0x20); //configuration gain 1.3Ga
  Wire.endTransmission();
  delay(10);
  //Put the HMC5883 IC into the correct operating mode
  Wire.beginTransmission(HMC5883_Address); //open communication with HMC5883
  Wire.write(0x02); //select mode register
  Wire.write(0x00); //continuous measurement mode
  Wire.endTransmission();
  delay(10);
}

void Mag5883Read()
{
  Wire.beginTransmission(HMC5883_Address);  //Tell the HMC5883 where to begin reading data
  Wire.write(0x03); //select register 3, X MSB register
  Wire.endTransmission();//Read data from each axis, 2 registers per axis
  Wire.requestFrom(HMC5883_Address, 6);
  int i = 0;
  byte result[6];
  while(Wire.available())    
  { 
    result[i] = Wire.read(); 
    i++;
  }
  Wire.endTransmission();   
  MagY1 = ((result[0] << 8) | result[1]);//offset + 1.05
  MagZ1 = ((result[2] << 8) | result[3]);// + 0.05
  MagX1 = ((result[4] << 8) | result[5])*-1;// - 0.55
  MagXf = MagXf + (MagX1 - MagXf)*0.3521;//0.45
  MagYf = MagYf + (MagY1 - MagYf)*0.3521;
  MagZf = MagZf + (MagZ1 - MagZf)*0.3521;
 // adjust for  compass axis offsets/sensitivity differences by scaling to +/-5 range
  c_magnetom_x = ((float)(MagXf - M_X_MIN) / (M_X_MAX - M_X_MIN))*10.0 - 5.0;
  c_magnetom_y = ((float)(MagYf - M_Y_MIN) / (M_Y_MAX - M_Y_MIN))*10.0 - 5.0;
  c_magnetom_z = ((float)(MagZf - M_Z_MIN) / (M_Z_MAX - M_Z_MIN))*10.0 - 5.0;
}
void Mag_Calibrate()//Calibration_sensor Magnetometer
{
    // Output MIN/MAX values
    M_X_MIN = 0;
    M_Y_MIN = 0;
    M_Z_MIN = 0;
    M_X_MAX = 0;
    M_Y_MAX = 0;
    M_Z_MAX = 0; 
    Serial.print("magn x,y,z (min/max) = ");
    for (int i = 0; i < 600; i++) {//Calibration 30 s
      digitalWrite(13, HIGH);//30
      Mag5883Read();
      if (MagX1 < M_X_MIN) M_X_MIN = MagX1;
      if (MagX1 > M_X_MAX) M_X_MAX = MagX1;
      if (MagY1 < M_Y_MIN) M_Y_MIN = MagY1;
      if (MagY1 > M_Y_MAX) M_Y_MAX = MagY1;
      if (MagZ1 < M_Z_MIN) M_Z_MIN = MagZ1;
      if (MagZ1 > M_Z_MAX) M_Z_MAX = MagZ1;
      delay(25);
      digitalWrite(13, LOW);//30
      delay(25);
    }
      Serial.print(M_X_MIN);Serial.print("/");
      Serial.print(M_X_MAX);Serial.print("\t");
      Serial.print(M_Y_MIN);Serial.print("/");
      Serial.print(M_Y_MAX);Serial.print("\t");
      Serial.print(M_Z_MIN);Serial.print("/");
      Serial.print(M_Z_MAX);
      Serial.print("\n");
}
void mpu6500_readGyroSum() {
    mpu6500_Gyro_Values();
    gyroSum[XAXIS] += gyroRaw[XAXIS];
    gyroSum[YAXIS] += gyroRaw[YAXIS];
    gyroSum[ZAXIS] += gyroRaw[ZAXIS];
    gyroSamples++;
}
void mpu6500_Get_gyro()
{       
    if(gyroSamples == 0){
      gyroSamples = 1;
    }
    GyroX = (gyroSum[XAXIS] / gyroSamples)*gyroScaleFactor - gyro_offsetX;// Calculate average
    GyroY = (gyroSum[YAXIS] / gyroSamples)*gyroScaleFactor - gyro_offsetY;
    GyroZ = (gyroSum[ZAXIS] / gyroSamples)*gyroScaleFactor - gyro_offsetZ;            
    gyroSum[XAXIS] = 0.0;// Reset SUM variables
    gyroSum[YAXIS] = 0.0;
    gyroSum[ZAXIS] = 0.0;
    gyroSamples2 = gyroSamples;
    gyroSamples = 0;            
}
void mpu6500_readAccelSum() {
    mpu6500_Accel_Values();
    accelSum[XAXIS] += AccXm;
    accelSum[YAXIS] += AccYm;
    accelSum[ZAXIS] += AccZm;  
    accSamples++;
}
void mpu6500_Get_accel()
{
    if(accSamples == 0){
      accSamples = 1;
    }
    AccX = (accelSum[XAXIS] / accSamples)*accelScaleFactor - acc_offsetX;// Calculate average
    AccY = (accelSum[YAXIS] / accSamples)*accelScaleFactor - acc_offsetY;
    AccZ = (accelSum[ZAXIS] / accSamples)*accelScaleFactor;// Apply correct scaling (at this point accel reprensents +- 1g = 9.81 m/s^2)
    accelSum[XAXIS] = 0.0;    // Reset SUM variables
    accelSum[YAXIS] = 0.0;
    accelSum[ZAXIS] = 0.0; 
    accSamples = 0;   
}
void sensor_Calibrate()
{
    Serial.print("Sensor_Calibrate");Serial.println("\t");
    gyroSum[XAXIS] = 0.0;// Reset SUM variables
    gyroSum[YAXIS] = 0.0;
    gyroSum[ZAXIS] = 0.0;
    gyroSamples = 0;  
    accelSum[XAXIS] = 0.0;    // Reset SUM variables
    accelSum[YAXIS] = 0.0;
    accelSum[ZAXIS] = 0.0; 
    for (uint8_t i=0; i<45; i++) //Collect 60, 100 samples
    {
        Serial.print("- ");
        mpu6500_readGyroSum();
        mpu6500_readAccelSum();
        Mag5883Read();
        digitalWrite(13, HIGH);
        digitalWrite(Pin_LED_R, HIGH);
        delay(20);
        digitalWrite(13, LOW);
        digitalWrite(Pin_LED_R, LOW);
        delay(20);
    }
    Serial.println("- ");
    gyro_offsetX = (gyroSum[XAXIS]/gyroSamples)*gyroScaleFactor;
    gyro_offsetY = (gyroSum[YAXIS]/gyroSamples)*gyroScaleFactor;
    gyro_offsetZ = (gyroSum[ZAXIS]/gyroSamples)*gyroScaleFactor;
    acc_offsetX = (accelSum[XAXIS]/gyroSamples)*accelScaleFactor;
    acc_offsetY = (accelSum[YAXIS]/gyroSamples)*accelScaleFactor;
    acc_offsetZ = (accelSum[ZAXIS]/gyroSamples)*accelScaleFactor;
    acc_offsetZ2 = sqrt(acc_offsetX*acc_offsetX + acc_offsetY*acc_offsetY + acc_offsetZ*acc_offsetZ);
    AccZf = acc_offsetZ;//15.4
    MagXf = MagX1;
    MagYf = MagY1;
    MagZf = MagZ1;
    gyroSamples = 0;
    accSamples = 0;
    Serial.print("GYRO_Calibrate");Serial.print("\t");
    Serial.print(gyro_offsetX);Serial.print("\t");//-0.13
    Serial.print(gyro_offsetY);Serial.print("\t");//-0.10
    Serial.print(gyro_offsetZ);Serial.println("\t");//0.03 
    Serial.print("ACC_Calibrate");Serial.print("\t");
    Serial.print(acc_offsetX);Serial.print("\t");
    Serial.print(acc_offsetY);Serial.print("\t");
    Serial.print(acc_offsetZ);Serial.print("\t");
    Serial.print(acc_offsetZ2);Serial.println("\t");  
acc_offsetX = 0.04;//-0.18 0.11 -0.36  Trim PITCH CONTROL   -10.07	-10.55	-9.82
acc_offsetY = 0.06;//0.16 -0.14 0.18 Trim ROLL CONTROL     10.39	9.74	11
//acc_offsetZ = 0.0;//0.245 0.235 10.2
}
