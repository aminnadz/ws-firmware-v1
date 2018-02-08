#include <Arduino.h>
#include "config.h"                                         /* config.h holds configuration parameters */
#include <SparkFunMPU9250-DMP.h>                            /* MPU-9250 Digital Motion Processing (DMP) Library */
#include <RN487x_BLE.h>                                     /* RN4871 BLE Module Library (by Microchip) */

MPU9250_DMP imu;                                            /* Create an instance of the MPU9250_DMP class */

/* MPU Configuration Globals (uses definition from config.h */
bool enableTimeLog = ENABLE_TIME_LOG;
bool enableAccel = ENABLE_ACCEL_LOG;
bool enableGyro = ENABLE_GYRO_LOG;
bool enableCompass = ENABLE_MAG_LOG;
bool enableQuat = ENABLE_QUAT_LOG;
bool enableEuler = ENABLE_EULER_LOG;
bool enableHeading = ENABLE_HEADING_LOG;
unsigned short accelFSR = IMU_ACCEL_FSR;
unsigned short gyroFSR = IMU_GYRO_FSR;
unsigned short fifoRate = DMP_SAMPLE_RATE;

/* Bluetooth Configuration Globals */
const char* myDeviceName = "ws-ino" ;                         /* Set custom Device name */

/* LED Blink Control */
uint32_t lastBlink = 0;
bool ledState = false;

/* State machine related variables */
uint8_t state;
String message = "";

void setup() {
  init_ws_hardware();                                         /* init the hardware */
  delay(100);

  /* Initialize the MPU-9250. Should return true on success: */
  if (!init_IMU())
  {
    SerialUSB.println("Error initializing MPU-9250");
    while (1) ;                                               /* Loop forever connection fails. LED will remain off in this state. */
  }

  /* Initialize the RN4871. Should return true on success: */
  if (!init_BT())
  {
    SerialUSB.println("Error initializing RN4871");
    while (1) ;                                               /* Loop forever connection fails. LED will remain off in this state. */
  }

  state = IDLE;                                               /* After config start from IDLE state */
  delay(500);
}

void loop()
{
  switch (state)
  {
    case IDLE:
      if (imu.fifoAvailable())                                /* If new data is available */
      {
        state = SAMPLE;                                       /* Change state to SAMPLE */
      }
      else
      {
        state = IDLE;                                         /* Otherwise stay in IDLE */
      }

      break;

    case SAMPLE:
      if (imu.dmpUpdateFifo() != INV_SUCCESS)                 /* If fifo can't be updated (sample fail) */
      {
        state = IDLE;                                         /* Return to IDLE state */
      }
      else
      {
        if ((enableCompass || enableHeading) && (imu.updateCompass() != INV_SUCCESS))           /* If compass enabled and can't read */
        {
          state = IDLE;                                       /* Return to IDLE state */
        }
        else
        {
          state = SERIALIZE;                                  /* Otherwise move to SERIALIZE */
        }
      }
      break;

    case SERIALIZE:
      message = Serialize();                                  /* Serialize sensor data */
      state = SEND;                                           /* Next state SEND */
      break;

    case SEND:
      BTSerial.print(message);                                /* Send serialized message to Bluetooth */
      SerialUSB.print(message);                               /* Send serialized message to USB */
      SerialUSB.print(".");                                   /* Send serialized message to USB */
      indicate();                                             /* Flash LED*/
      state = IDLE;                                           /* Go to IDLE state */
      break;

    default :
      SerialUSB.print("Invalid state");
  }
}


void init_ws_hardware(void)
{

  SerialUSB.begin(115200);                                    /* Initialize USB serial */
  while (!SerialUSB) {                                        /* Wait for initialization to end */
    ;
  }

  BTSerial.begin(115200);                                     /* Initialize Bluetooth Serial */
  while (!BTSerial) {                                         /* Wait for initialization to end */
    ;
  }

  pinMode(LED_BUILTIN, OUTPUT);                               /* Initialize pin for GREEN (D13) led */
  digitalWrite(LED_BUILTIN, LOW);                             /* Turn LED Off */

  pinMode(BT_CONFIN, OUTPUT);                                 /* BT System Configuration input at D6, configure as output*/
  digitalWrite(BT_CONFIN, HIGH);                              /* Put module in application mode (LOW for test mode) */

  pinMode(BT_RESET, OUTPUT);                                  /* BT Module Reset at D8, configure as output*/
  digitalWrite(BT_RESET, HIGH);                               /* High = not in reset */

  pinMode(MPU_INT, INPUT_PULLUP);                             /* MPU interrupt pin at D9, configure as input with pullup*/
}

bool init_IMU(void)
{
  /* imu.begin() should return 0 on success. Will initialize I2C bus, and reset MPU-9250 to defaults. */
  if (imu.begin() != INV_SUCCESS)
  {
    return false;
  }

  /* Set up MPU-9250 interrupt: */
  imu.enableInterrupt();                                                      /* Enable interrupt output */
  imu.setIntLevel(1);                                                         /* Set interrupt to active-low */
  imu.setIntLatched(1);                                                       /* Latch interrupt output */

  /*  Configure sensors: */
  imu.setGyroFSR(gyroFSR);
  imu.setAccelFSR(accelFSR);
  imu.setLPF(IMU_AG_LPF);
  imu.setSampleRate(IMU_AG_SAMPLE_RATE);                                      /* this value will be overridden by the DMP sample rate */
  imu.setCompassSampleRate(IMU_COMPASS_SAMPLE_RATE);


  /* Configure DMP. Use the FIFO to get data from the DMP. */
  unsigned short dmpFeatureMask = 0;

  if (ENABLE_GYRO_CALIBRATION)
  {
    dmpFeatureMask |= DMP_FEATURE_SEND_CAL_GYRO;                              /* Gyro calibration - re-calibrates the gyro after a set amount of no motion detected */
  }
  else
  {
    dmpFeatureMask |= DMP_FEATURE_SEND_RAW_GYRO;                              /* Otherwise add raw gyro readings to the DMP */
  }

  /* Add accel and quaternion's to the DMP */
  dmpFeatureMask |= DMP_FEATURE_SEND_RAW_ACCEL;
  dmpFeatureMask |= DMP_FEATURE_6X_LP_QUAT;

  /* Initialize the DMP, and set the FIFO's update rate: */
  imu.dmpBegin(dmpFeatureMask, fifoRate);

  return true; // Return success
}

bool init_BT(void)
{
  bool ret = false;

  rn487xBle.hwInit();                                                           /* Initialize the BLE hardware */
  rn487xBle.initBleStream(&BTSerial);                                           /* Assign the BLE serial port to the BLE library */

  /* Init the module process */
  if (rn487xBle.swInit())
    ret = true;
  else
    ret = false;

  /* Setup the module */
  rn487xBle.enterCommandMode();                                                 /* Fist, enter into command mode */
  rn487xBle.stopAdvertising();                                                  /* Stop advertising */
  rn487xBle.setAdvPower(2);                                                     /* Set the advertising output power (range: min = 5, max = 0) */
  rn487xBle.setSerializedName(myDeviceName);                                    /* Set the serialized device name */
  rn487xBle.setDefaultServices(UART_TRANSP_SERVICE | DEVICE_INFO_SERVICE);      /* Set device info service and transparent uart service */
  rn487xBle.startAdvertising();                                                 /* Start advertising */
  rn487xBle.reboot();                                                           /* Reset the module */

  return ret;                                                                   /* Return success or fail */
}

String Serialize(void)
{
  String packet = "";                                                           /* Create empty packet*/
  if (enableTimeLog)                                                            /* If time logging is enabled */
  {
    packet += String(imu.time) + ",";                                           /* Add time to packet */
  }

  if (enableAccel)                                                              /* If accelerometer logging is enabled */
  {
    packet += String(imu.calcAccel(imu.ax)) + ",";
    packet += String(imu.calcAccel(imu.ay)) + ",";
    packet += String(imu.calcAccel(imu.az)) + ",";
  }

  if (enableGyro)                                                               /* If gyroscope logging is enabled */
  {
    packet += String(imu.calcGyro(imu.gx)) + ",";
    packet += String(imu.calcGyro(imu.gy)) + ",";
    packet += String(imu.calcGyro(imu.gz)) + ",";
  }

  if (enableCompass)                                                            /* If magnetometer logging is enabled */
  {
    packet += String(imu.calcMag(imu.mx)) + ",";
    packet += String(imu.calcMag(imu.my)) + ",";
    packet += String(imu.calcMag(imu.mz)) + ",";
  }

  if (enableQuat)                                                               /* If quaternion logging is enabled */
  {
    packet += String(imu.calcQuat(imu.qw), 4) + ",";
    packet += String(imu.calcQuat(imu.qx), 4) + ",";
    packet += String(imu.calcQuat(imu.qy), 4) + ",";
    packet += String(imu.calcQuat(imu.qz), 4) + ",";
  }

  if (enableEuler)                                                              /* If Euler-angle logging is enabled */
  {
    imu.computeEulerAngles();
    packet += String(imu.pitch, 2) + ",";
    packet += String(imu.roll, 2) + ",";
    packet += String(imu.yaw, 2) + ",";
  }

  if (enableHeading)                                                            /* If heading logging is enabled */
  {
    packet += String(imu.computeCompassHeading(), 2) + ",";
  }

  /* Remove last comma/space: */
  packet.remove(packet.length() - 2, 2);
  packet += "\r\n"; // Add a new line

  return packet;                                                                /* return serialized packet string */
}

void indicate(void)
{
  /* Blink LED once every second*/
  if ( millis() > lastBlink + 1000 )
  {
    digitalWrite(LED_BUILTIN, ledState);
    ledState = !ledState;
    lastBlink = millis();
  }
}
