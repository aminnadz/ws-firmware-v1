/* Hardware Related Definitions */
#define BT_CONFIN   6                     /* Bluetooth module configuration input pin */
#define BT_WAKE     7                     /* Bluetooth wakeup pin */
#define BT_RESET    8                     /* Bluetooth module reset pin */
#define MPU_INT     9                     /* MPU interrupt pin */
#define VBAT        0                     /* Battery Voltage pin */



/* Sensors enabled by default*/
#define ENABLE_TIME_LOG         false	
#define ENABLE_ACCEL_LOG        true
#define ENABLE_GYRO_LOG         true
#define ENABLE_MAG_LOG          false
#define ENABLE_QUAT_LOG         true
#define ENABLE_EULER_LOG        false
#define ENABLE_HEADING_LOG      false


/* IMU Default Configuration */
#define DMP_SAMPLE_RATE           100       /* DMP sample rate(4-200 Hz) */
#define IMU_COMPASS_SAMPLE_RATE   100       /* Compass sample rate (4-100 Hz) */
#define IMU_AG_SAMPLE_RATE        100       /* Accel/gyro sample rate (4Hz and 1kHz) */
#define IMU_GYRO_FSR              2000      /* Gyro full-scale range (250, 500, 1000, or 2000) */
#define IMU_ACCEL_FSR             2         /* Accel full-scale range (2, 4, 8, or 16) */
#define IMU_AG_LPF                5         /* Accel/Gyro LPF corner frequency (5, 10, 20, 42, 98, or 188 Hz) */
#define ENABLE_GYRO_CALIBRATION   true      /* Preform gyro calibration */
#define MPU9250_INT_ACTIVE        LOW       /* MPU9250 interrupt active low */

/* BT Module Related Configuration */
#define BTSerial Serial2                    /* Define Serial2 as BTSerial*/

/* State Machine related initialization */
#define IDLE        0
#define SAMPLE      1
#define SERIALIZE   2
#define SEND        3
