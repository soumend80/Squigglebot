//=========================================================================================================
/*
  This code reads data from MPU6050 IMU and WCS2702 current sensors and sends it to a local telent server.
  Be sure to update the correct SSID and PASSWORD before running to allow connection to your WiFi network.

  Please refer to the github repository https://github.com/soumend80/Squigglebot for more details.
*/ 
//=========================================================================================================
#include <Wire.h>
#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <ESP8266WebServer.h>
#include <WiFiUdp.h>
#include <ESP8266mDNS.h>
#include "ESP8266HTTPUpdateServerCustom.h"

//===================================================================================================
//                Replace with your network credentials (IMPORTANT!!!)
//===================================================================================================
const char* ssid = "********";
const char* password = "********";
const char* host = "esp8266-webupdate";
const char* update_path = "/update";
const char* update_username = "admin";
const char* update_password = "admin";
#define MAX_SRV_CLIENTS 3			// how many clients should be able to telnet to this ESP8266
ESP8266WebServer webserver(80);   	// start a webserver at port 80 (http port)
WiFiServer telnetserver(23);        // Telnet server at port 23
WiFiClient serverClients[MAX_SRV_CLIENTS];
ESP8266HTTPUpdateServerCustom httpUpdater;
String page = "";
double data;
unsigned long previousMicros = 0;        
unsigned long currentMicros = 0;
unsigned long elapsedMillis = 0;
unsigned long count = 0;
bool flag = false;
bool startflag = false;
int ledPin = 2;
bool ledState = LOW;
int wifi_search_count = 0;         // Counter to search for Wifi before restarting
float data_rate = 40000;           // data transmission rate in microseconds (40 millisecond)
#define NUM_CHIPS 20               // total number of Squigglebots

//=================================================================================================================
// Register definitions (see MPU-6000/MPU-6050 Register Map and Descriptions, Rev 4.2, 08/19/2013 for more details)
//=================================================================================================================
#define XGOFFS_TC        0x00                  
#define YGOFFS_TC        0x01
#define ZGOFFS_TC        0x02
#define X_FINE_GAIN      0x03 
#define Y_FINE_GAIN      0x04
#define Z_FINE_GAIN      0x05
#define XA_OFFSET_H      0x06 
#define XA_OFFSET_L_TC   0x07
#define YA_OFFSET_H      0x08
#define YA_OFFSET_L_TC   0x09
#define ZA_OFFSET_H      0x0A
#define ZA_OFFSET_L_TC   0x0B
#define SELF_TEST_X      0x0D
#define SELF_TEST_Y      0x0E
#define SELF_TEST_Z      0x0F
#define SELF_TEST_A      0x10
#define XG_OFFS_USRH     0x13  
#define XG_OFFS_USRL     0x14
#define YG_OFFS_USRH     0x15
#define YG_OFFS_USRL     0x16
#define ZG_OFFS_USRH     0x17
#define ZG_OFFS_USRL     0x18
#define SMPLRT_DIV       0x19
#define CONFIG           0x1A
#define GYRO_CONFIG      0x1B
#define ACCEL_CONFIG     0x1C
#define FF_THR           0x1D  
#define FF_DUR           0x1E  
#define MOT_THR          0x1F  
#define MOT_DUR          0x20  
#define ZMOT_THR         0x21  
#define ZRMOT_DUR        0x22  
#define FIFO_EN          0x23
#define I2C_MST_CTRL     0x24
#define I2C_SLV0_ADDR    0x25
#define I2C_SLV0_REG     0x26
#define I2C_SLV0_CTRL    0x27
#define I2C_SLV1_ADDR    0x28
#define I2C_SLV1_REG     0x29
#define I2C_SLV1_CTRL    0x2A
#define I2C_SLV2_ADDR    0x2B
#define I2C_SLV2_REG     0x2C
#define I2C_SLV2_CTRL    0x2D
#define I2C_SLV3_ADDR    0x2E
#define I2C_SLV3_REG     0x2F
#define I2C_SLV3_CTRL    0x30
#define I2C_SLV4_ADDR    0x31
#define I2C_SLV4_REG     0x32
#define I2C_SLV4_DO      0x33
#define I2C_SLV4_CTRL    0x34
#define I2C_SLV4_DI      0x35
#define I2C_MST_STATUS   0x36
#define INT_PIN_CFG      0x37
#define INT_ENABLE       0x38
#define DMP_INT_STATUS   0x39  
#define INT_STATUS       0x3A
#define ACCEL_XOUT_H     0x3B
#define ACCEL_XOUT_L     0x3C
#define ACCEL_YOUT_H     0x3D
#define ACCEL_YOUT_L     0x3E
#define ACCEL_ZOUT_H     0x3F
#define ACCEL_ZOUT_L     0x40
#define TEMP_OUT_H       0x41
#define TEMP_OUT_L       0x42
#define GYRO_XOUT_H      0x43
#define GYRO_XOUT_L      0x44
#define GYRO_YOUT_H      0x45
#define GYRO_YOUT_L      0x46
#define GYRO_ZOUT_H      0x47
#define GYRO_ZOUT_L      0x48
#define EXT_SENS_DATA_00 0x49
#define EXT_SENS_DATA_01 0x4A
#define EXT_SENS_DATA_02 0x4B
#define EXT_SENS_DATA_03 0x4C
#define EXT_SENS_DATA_04 0x4D
#define EXT_SENS_DATA_05 0x4E
#define EXT_SENS_DATA_06 0x4F
#define EXT_SENS_DATA_07 0x50
#define EXT_SENS_DATA_08 0x51
#define EXT_SENS_DATA_09 0x52
#define EXT_SENS_DATA_10 0x53
#define EXT_SENS_DATA_11 0x54
#define EXT_SENS_DATA_12 0x55
#define EXT_SENS_DATA_13 0x56
#define EXT_SENS_DATA_14 0x57
#define EXT_SENS_DATA_15 0x58
#define EXT_SENS_DATA_16 0x59
#define EXT_SENS_DATA_17 0x5A
#define EXT_SENS_DATA_18 0x5B
#define EXT_SENS_DATA_19 0x5C
#define EXT_SENS_DATA_20 0x5D
#define EXT_SENS_DATA_21 0x5E
#define EXT_SENS_DATA_22 0x5F
#define EXT_SENS_DATA_23 0x60
#define MOT_DETECT_STATUS 0x61
#define I2C_SLV0_DO      0x63
#define I2C_SLV1_DO      0x64
#define I2C_SLV2_DO      0x65
#define I2C_SLV3_DO      0x66
#define I2C_MST_DELAY_CTRL 0x67
#define SIGNAL_PATH_RESET  0x68
#define MOT_DETECT_CTRL   0x69
#define USER_CTRL        0x6A  
#define PWR_MGMT_1       0x6B  
#define PWR_MGMT_2       0x6C
#define DMP_BANK         0x6D  
#define DMP_RW_PNT       0x6E  
#define DMP_REG          0x6F  
#define DMP_REG_1        0x70
#define DMP_REG_2        0x71
#define FIFO_COUNTH      0x72
#define FIFO_COUNTL      0x73
#define FIFO_R_W         0x74
#define WHO_AM_I_MPU6050 0x75 

//===================================================================================================
// AD0 is set to 0 by grounding through a 4k7 resistor.
// Seven-bit device address is 110100 for AD0 = 0 and 110101 for AD0 = 1
//===================================================================================================
#define ADO 0
#if ADO
#define MPU6050_ADDRESS 0x69  // Device address when AD0 = 1
#else
#define MPU6050_ADDRESS 0x68  // Device address when AD0 = 0
#endif

// Set initial input parameters
enum Ascale {
  AFS_2G = 0,
  AFS_4G,
  AFS_8G,
  AFS_16G
};

enum Gscale {
  GFS_250DPS = 0,
  GFS_500DPS,
  GFS_1000DPS,
  GFS_2000DPS
};
//===================================================================================================
// 							Specify sensor full scale range
//===================================================================================================
int Gscale = GFS_2000DPS;
int Ascale = AFS_2G;
float aRes, gRes; 		// scale resolutions per LSB for the sensors


int16_t accelCount[3];           // Stores the 16-bit signed accelerometer sensor output
float ax, ay, az;                // Stores the real accel value in g's
int16_t gyroCount[3];            // Stores the 16-bit signed gyro sensor output
float gx, gy, gz;                // Stores the real gyro value in degrees per seconds
float gyroBias[3], accelBias[3]; // Bias corrections for gyro and accelerometer from calibration
int16_t tempCount;               // Stores the internal chip temperature sensor output
float temperature;               // Scaled temperature in degrees Celsius
float SelfTest[6];               // Gyro and accelerometer self-test sensor output
uint32_t delt_t = 0;

//===================================================================================================
// 						Offset gyro values obtained from calibration
//===================================================================================================
const float calibration_gyro[NUM_CHIPS][3] = {
  { -3.55, -4.26, -0.21},
  {0.28, -0.34, 0.21},
  { -3.66, -1.00, -1.89},
  { -1.89, -0.50, -0.83},
  { -1.46, -0.18, 0.35},
  { -4.73, 5.01, -1.50},
  { -1.73, 0.60, -0.11},
  {2.06, 1.44, -0.43},
  { -0.63, 2.36, -0.64},
  { -0.05, -0.28, 0.86},
  { -1.82, -0.71, -1.39},
  { -2.97, -0.30, -0.29},
  { -1.50, -1.51, -1.82},
  { -0.63, 0.85, -0.53},
  { -2.03, -2.74, -2.05},
  { -9.74, -0.59, -0.69},
  { -1.93, 2.76, -0.29},
  {0.11, 0.47, 1.13},
  {0.53, 1.65, -1.09},
  {1.18, 2.53, -1.66},
};
//===================================================================================================
//					Offset acceleration values obtatined from calibration
//===================================================================================================
const float calibration_accel[NUM_CHIPS][3] = {
  {0.05, -0.02, -0.03},
  {0.00, -0.03, 0.32},
  { -0.01, 0.01, 0.04},
  { -0.01, 0.01, 0.03},
  { -0.02, -0.00, 0.06},
  { -0.01, 0.02, -0.01},
  { -0.00, 0.01, 0.04},
  { -0.00, 0.00, 0.03},
  {0.01, -0.00, 0.17},
  {0.01, 0.01, 0.09},
  {0.04, -0.03, -0.01},
  { -0.01, 0.00, 0.01},
  { -0.01, 0.01, -0.03},
  {0.01, 0.03, -0.04},
  {0.01, -0.01, -0.06},
  { -0.00, 0.01, 0.64},
  {0.06, 0.00, -0.28},
  {0.02, 0.01, 0.07},
  {0.00, 0.02, 0.31},
  { -0.01, 0.00, 0.06},
};

//===================================================================================================
//					Assigned fixed ip addresses to all the robots
//===================================================================================================
const int ip_address_list[NUM_CHIPS] = {103, 115, 132, 155, 143, 125, 180, 174, 164, 111, 114, 192, 160, 197, 184, 113, 175, 126, 122, 172};
String my_ip;

//===================================================================================================
// Uncomment below piece of code to get the orientation of the sensor. Implemented following the algorithm in
// S. O. H. Madgwick, A. J. L. Harrison and R. Vaidyanathan, "Estimation of IMU and MARG orientation using a gradient descent algorithm," 
// 2011 IEEE International Conference on Rehabilitation Robotics, 2011, pp. 1-7, doi: 10.1109/ICORR.2011.5975346.
// It fuses acceleration and rotation rate to produce a quaternion-based estimate of relative
// device orientation, which can be converted to yaw, pitch, and roll.
// parameters for 6 DoF sensor fusion calculations
//===================================================================================================
// float GyroMeasError = PI * (40.0f / 180.0f);     // gyroscope measurement error in rads/s 
// float beta = sqrt(3.0f / 4.0f) * GyroMeasError;  // compute beta
// float GyroMeasDrift = PI * (2.0f / 180.0f);      // gyroscope measurement drift in rad/s/s
// float zeta = sqrt(3.0f / 4.0f) * GyroMeasDrift;  // compute zeta, the other free parameter in the Madgwick scheme usually set to a small or zero value
// float pitch, yaw, roll;
// float deltat = 0.0f;                              // integration interval for both filter schemes
// uint32_t lastUpdate = 0, firstUpdate = 0;         // used to calculate integration interval
// uint32_t Now = 0;                                 // used to calculate integration interval
// float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};            // vector to hold quaternion

//===================================================================================================
//                    Power on setup
//===================================================================================================
void setup()
{
  Wire.begin();
  Serial.begin(115200);
  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, ledState);
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);                   //begin WiFi connection
  Serial.println("");
  delay(30);

  // Wait for connection
  Serial.println("Connecting");
  while (WiFi.status() != WL_CONNECTED) {
    delay(250);
    Serial.print(".");

    wifi_search_count += 1;
    if (wifi_search_count > 80) {               // Wait for 20 seconds to connect to WiFi
      Serial.println("");
      Serial.println("Can't connect to WiFi... Restarting...");
      ESP.restart();
    }
  }
  Serial.println("");
  Serial.print("Connected to ");
  Serial.println(ssid);
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  telnetserver.begin();
  telnetserver.setNoDelay(true);

  Serial.print("Telnet server started! Use 'telnet ");
  Serial.print(WiFi.localIP());
  Serial.println(" 23' to connect");

  webserver.on("/1", turnOn);         			// Associate the handler function for turning the motor on
  webserver.on("/0", turnOff);        			// Associate the handler function for turning the motor off
  webserver.on("/toggle", toggle);    			// Associate the handler function for toggling the state of the motor
  webserver.on("/", SendIP_Mac);				// Associate the handler function for sending the IP & MAC address of ESP8266
  webserver.on("/start", StartMeasurement);		// Associate the handler function for starting the measurement
  webserver.on("/reboot", Reboot);				// Associate the handler function for rebooting ESP8266
  webserver.begin();
  Serial.println("HTTP server started!");
  MDNS.begin(host);
  httpUpdater.setup(&webserver, update_path);
  MDNS.addService("http", "tcp", 80);
  Serial.println("HTTPUpdateServer ready!");
  Serial.printf("Open http://%s.local%s or http://", host, update_path);
  Serial.print(WiFi.localIP());
  Serial.printf("%s in your browser\n", update_path);

  my_ip = WiFi.localIP().toString();
//  Serial.println(my_ip);
  my_ip.remove(0, 10);
//  Serial.println(my_ip);

  for (uint8_t i = 0; i < NUM_CHIPS; i++) {
    if (ip_address_list[i] == my_ip.toInt()) {
      for (uint8_t j = 0; j < 3; j++) {
        gyroBias[j] = calibration_gyro[i][j];
        accelBias[j] = calibration_accel[i][j];
      }
      break;
    }
  }

  // Read the WHO_AM_I register, this is a good test of communication
  uint8_t c = readByte(MPU6050_ADDRESS, WHO_AM_I_MPU6050);  // Read WHO_AM_I register for MPU-6050

  if (c == 0x68) // WHO_AM_I should always be 0x68
  {
    Serial.println("MPU6050 is online...");

    MPU6050SelfTest(SelfTest); // Start by performing self test and reporting values
    //    Serial.print("x-axis self test: acceleration trim within : "); Serial.print(SelfTest[0]); Serial.println("% of factory value");
    //    Serial.print("y-axis self test: acceleration trim within : "); Serial.print(SelfTest[1]); Serial.println("% of factory value");
    //    Serial.print("z-axis self test: acceleration trim within : "); Serial.print(SelfTest[2]); Serial.println("% of factory value");
    //    Serial.print("x-axis self test: gyration trim within : "); Serial.print(SelfTest[3]); Serial.println("% of factory value");
    //    Serial.print("y-axis self test: gyration trim within : "); Serial.print(SelfTest[4]); Serial.println("% of factory value");
    //    Serial.print("z-axis self test: gyration trim within : "); Serial.print(SelfTest[5]); Serial.println("% of factory value");

    if (SelfTest[0] < 1.0f && SelfTest[1] < 1.0f && SelfTest[2] < 1.0f && SelfTest[3] < 1.0f && SelfTest[4] < 1.0f && SelfTest[5] < 1.0f) {
      delay(1000);

      initMPU6050(); Serial.println("MPU6050 initialized for active data mode...."); // Initialize device for active mode read of acclerometer, gyroscope, and temperature
      Serial.print("gyroBiasX = "); Serial.println(gyroBias[0]);
      Serial.print("gyroBiasY = "); Serial.println(gyroBias[1]);
      Serial.print("gyroBiasZ = "); Serial.println(gyroBias[2]);
      Serial.print("accelBiasX = "); Serial.println(accelBias[0]);
      Serial.print("accelBiasY = "); Serial.println(accelBias[1]);
      Serial.print("accelBiasZ = "); Serial.println(accelBias[2]);
      Serial.println("");
    }
    else
    {
      Serial.print("Could not connect to MPU6050: 0x");
      Serial.println(c, HEX);
      //      while (1) ; // Loop forever if communication doesn't happen
    }

  }

  previousMicros = micros();
}

//===================================================================================================
//                    Main Program Loop
//===================================================================================================
void loop()
{
  webserver.handleClient();
  MDNS.update();

  if (startflag) {
    uint8_t i;
    //check if there are any new clients
    if (telnetserver.hasClient()) {
      flag = true;
      if (count == 0) {
        previousMicros = micros();
        count++;
      }
      for (i = 0; i < MAX_SRV_CLIENTS; i++) {
        //find free/disconnected spot
        if (!serverClients[i] || !serverClients[i].connected()) {
          if (serverClients[i]) serverClients[i].stop();
          serverClients[i] = telnetserver.available();
          Serial.print("New client: "); Serial.println(i + 1);
          break;
        }
      }
      //    }
      //no free/disconnected spot so reject
      if ( i == MAX_SRV_CLIENTS) {
        WiFiClient serverClient = telnetserver.available();
        serverClient.stop();
        //Serial.println("Connection rejected ");
      }
    }
    //Send data
    currentMicros = micros();
    if (flag && (currentMicros - previousMicros > data_rate)) {

      if (readByte(MPU6050_ADDRESS, INT_STATUS) & 0x01) { // check if data ready interrupt
        readAccelData(accelCount);  // Read the x/y/z adc values
        getAres();

        // Now we'll calculate the accleration value into actual g's
        ax = (float)accelCount[0] * aRes - accelBias[0]; // get actual g value, this depends on scale being set
        ay = (float)accelCount[1] * aRes - accelBias[1];
        az = (float)accelCount[2] * aRes - accelBias[2];

        readGyroData(gyroCount);  // Read the x/y/z adc values
        getGres();

        // Calculate the gyro value into actual degrees per second
        gx = (float)gyroCount[0] * gRes - gyroBias[0]; // get actual gyro value, this depends on scale being set
        gy = (float)gyroCount[1] * gRes - gyroBias[1];
        gz = (float)gyroCount[2] * gRes - gyroBias[2];

        // tempCount = readTempData();  // Read the x/y/z adc values
        // temperature = ((float) tempCount) / 340. + 36.53; // Temperature in degrees Celsius
      }
     
	  // Uncomment if orientation needs to be calculated
	  // Now = micros();
      // deltat = ((Now - lastUpdate) / 1000000.0f); // set integration time by time elapsed since last filter update
      // lastUpdate = Now;
      // MadgwickQuaternionUpdate(ax, ay, az, gx * PI / 180.0f, gy * PI / 180.0f, gz * PI / 180.0f);

      // Print acceleration values in g
      //    Serial.print("ax = "); Serial.print(ax); Serial.print(" g ");
      //    Serial.print("ay = "); Serial.print(ay); Serial.print(" g ");
      //    Serial.print("az = "); Serial.print(az); Serial.println(" g");

      // Print gyro values in degree/sec
      //    Serial.print("gx = "); Serial.print(gx); Serial.print(" degrees/sec ");
      //    Serial.print("gy = "); Serial.print(gy); Serial.print(" degrees/sec ");
      //    Serial.print("gz = "); Serial.print(gz); Serial.println(" degrees/sec");

      // Print temperature in degrees Celsius
      //    Serial.print("T = ");  Serial.print(temperature, 2);  Serial.println(" degrees C"); // Print T values to tenths of s degree C
      //    Serial.println("");

	  // Print quaternion values
      //    Serial.print(" q0 = "); Serial.print(q[0]); 
      //    Serial.print(" qx = "); Serial.print(q[1]); 
      //    Serial.print(" qy = "); Serial.print(q[2]); 
      //    Serial.print(" qz = "); Serial.println(q[3]);

      //    Serial.print(' ');

      // Calculation of yaw, pitch, and roll
      //    yaw   = atan2(2.0f * (q[1] * q[2] + q[0] * q[3]), q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3]);
      //    pitch = -asin(2.0f * (q[1] * q[3] - q[0] * q[2]));
      //    roll  = atan2(2.0f * (q[0] * q[1] + q[2] * q[3]), q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3]);
      //    pitch *= 180.0f / PI;
      //    yaw   *= 180.0f / PI;
      //    roll  *= 180.0f / PI;

      ////    Serial.print("Yaw, Pitch, Roll: ");
      //    Serial.print(yaw, 2); Serial.print('\t');
      ////    Serial.print(", ");
      //    Serial.print(pitch, 2); Serial.print('\t');
      ////    Serial.print(", ");
      //    Serial.println(roll, 2);
      ////    Serial.println("");


      elapsedMillis += (currentMicros - previousMicros) / 1000;
      String str = String(gx) + "\t" + String(gy) + "\t" + String(gz) + "\t" + String(ax) + "\t" + String(ay) + "\t" + String(az) + "\t" + String(analogRead(A0)) + "\t" + String(elapsedMillis) + "\t" + String(digitalRead(ledPin));
	  //str = str + "\t" + String(q[0]) + "\t" + String(q[1]) + "\t" + String(q[2]) + "\t" + String(q[3]);// + "\n";
      previousMicros = currentMicros;
      //push data to all connected telnet clients
      for (i = 0; i < MAX_SRV_CLIENTS; i++) {
        if (serverClients[i] && serverClients[i].connected()) {
          serverClients[i].println(str);
          delay(1);
        }
      }
    }
  }

}

//===================================================================================================================
//					Set of useful function to access acceleration, gyroscope, and temperature data
//===================================================================================================================

void getGres() {
  switch (Gscale)
  {
    // Possible gyro scales (and their register bit settings) are:
    // 250 DPS (00), 500 DPS (01), 1000 DPS (10), and 2000 DPS  (11).
    // Calculation of resolution - DPS/(ADC tick) based on that 2-bit value:
    case GFS_250DPS:
      gRes = 250.0 / 32768.0;
      break;
    case GFS_500DPS:
      gRes = 500.0 / 32768.0;
      break;
    case GFS_1000DPS:
      gRes = 1000.0 / 32768.0;
      break;
    case GFS_2000DPS:
      gRes = 2000.0 / 32768.0;
      break;
  }
}

void getAres() {
  switch (Ascale)
  {
    // Possible accelerometer scales (and their register bit settings) are:
    // 2 Gs (00), 4 Gs (01), 8 Gs (10), and 16 Gs  (11).
    case AFS_2G:
      aRes = 2.0 / 32768.0;
      break;
    case AFS_4G:
      aRes = 4.0 / 32768.0;
      break;
    case AFS_8G:
      aRes = 8.0 / 32768.0;
      break;
    case AFS_16G:
      aRes = 16.0 / 32768.0;
      break;
  }
}


void readAccelData(int16_t * destination)
{
  uint8_t rawData[6];  											// x/y/z accel register data stored here
  readBytes(MPU6050_ADDRESS, ACCEL_XOUT_H, 6, &rawData[0]);  	// Read the six raw data registers into data array
  destination[0] = (int16_t)((rawData[0] << 8) | rawData[1]) ;  // Turn the MSB and LSB into a signed 16-bit value
  destination[1] = (int16_t)((rawData[2] << 8) | rawData[3]) ;
  destination[2] = (int16_t)((rawData[4] << 8) | rawData[5]) ;
}

void readGyroData(int16_t * destination)
{
  uint8_t rawData[6];  											// x/y/z gyro register data stored here
  readBytes(MPU6050_ADDRESS, GYRO_XOUT_H, 6, &rawData[0]);  	// Read the six raw data registers sequentially into data array
  destination[0] = (int16_t)((rawData[0] << 8) | rawData[1]) ;  // Turn the MSB and LSB into a signed 16-bit value
  destination[1] = (int16_t)((rawData[2] << 8) | rawData[3]) ;
  destination[2] = (int16_t)((rawData[4] << 8) | rawData[5]) ;
}

int16_t readTempData()
{
  uint8_t rawData[2];  											// x/y/z gyro register data stored here
  readBytes(MPU6050_ADDRESS, TEMP_OUT_H, 2, &rawData[0]);  		// Read the two raw data registers sequentially into data array
  return ((int16_t)rawData[0]) << 8 | rawData[1] ;  			// Turn the MSB and LSB into a 16-bit value
}

//===================================================================================================================
// 											Initialize MPU6050 device
//===================================================================================================================
void initMPU6050()
{
  // wake up device-don't need this here if using calibration function below
  writeByte(MPU6050_ADDRESS, PWR_MGMT_1, 0x00); // Clear sleep mode bit (6), enable all sensors
  delay(100); // Delay 100 ms for PLL to get established on x-axis gyro; should check for PLL ready interrupt

  // get stable time source
  writeByte(MPU6050_ADDRESS, PWR_MGMT_1, 0x01);  // Set clock source to be PLL with x-axis gyroscope reference, bits 2:0 = 001
  delay(100);

  // Configure Gyro and Accelerometer
  // Disable FSYNC and set accelerometer and gyro bandwidth to 44 and 42 Hz, respectively;
  // DLPF_CFG = bits 2:0 = 010; this sets the sample rate at 1 kHz for both
  writeByte(MPU6050_ADDRESS, CONFIG, 0x03);

  // Set sample rate = gyroscope output rate/(1 + SMPLRT_DIV)
  writeByte(MPU6050_ADDRESS, SMPLRT_DIV, 0x04);  // Use a 200 Hz sample rate

  // Set gyroscope full scale range
  // Range selects FS_SEL and AFS_SEL are 0 - 3, so 2-bit values are left-shifted into positions 4:3
  uint8_t c =  readByte(MPU6050_ADDRESS, GYRO_CONFIG);
  writeByte(MPU6050_ADDRESS, GYRO_CONFIG, c & ~0xE0); // Clear self-test bits [7:5]
  writeByte(MPU6050_ADDRESS, GYRO_CONFIG, c & ~0x18); // Clear AFS bits [4:3]
  writeByte(MPU6050_ADDRESS, GYRO_CONFIG, c | Gscale << 3); // Set full scale range for the gyro

  // Set accelerometer configuration
  c =  readByte(MPU6050_ADDRESS, ACCEL_CONFIG);
  writeByte(MPU6050_ADDRESS, ACCEL_CONFIG, c & ~0xE0); // Clear self-test bits [7:5]
  writeByte(MPU6050_ADDRESS, ACCEL_CONFIG, c & ~0x18); // Clear AFS bits [4:3]
  writeByte(MPU6050_ADDRESS, ACCEL_CONFIG, c | Ascale << 3); // Set full scale range for the accelerometer

  // Configure Interrupts and Bypass Enable
  // Set interrupt pin active high, push-pull, and clear on read of INT_STATUS, enable I2C_BYPASS_EN so additional chips
  // can join the I2C bus and all can be controlled by the Arduino as master
  //  writeByte(MPU6050_ADDRESS, INT_PIN_CFG, 0x02);
  writeByte(MPU6050_ADDRESS, INT_ENABLE, 0x01);  // Enable data ready (bit 0) interrupt
}

// Accelerometer and gyroscope self test; check calibration wrt factory settings
void MPU6050SelfTest(float * destination) // Should return percent deviation from factory trim values, +/- 14 or less deviation is a pass
{
  uint8_t rawData[4];
  uint8_t selfTest[6];
  float factoryTrim[6];

  // Configure the accelerometer for self-test
  writeByte(MPU6050_ADDRESS, ACCEL_CONFIG, 0xF0); // Enable self test on all three axes and set accelerometer range to +/- 8 g
  writeByte(MPU6050_ADDRESS, GYRO_CONFIG,  0xE0); // Enable self test on all three axes and set gyro range to +/- 250 degrees/s
  delay(250);  // Delay a while to let the device execute the self-test
  rawData[0] = readByte(MPU6050_ADDRESS, SELF_TEST_X); // X-axis self-test results
  rawData[1] = readByte(MPU6050_ADDRESS, SELF_TEST_Y); // Y-axis self-test results
  rawData[2] = readByte(MPU6050_ADDRESS, SELF_TEST_Z); // Z-axis self-test results
  rawData[3] = readByte(MPU6050_ADDRESS, SELF_TEST_A); // Mixed-axis self-test results
  // Extract the acceleration test results first
  selfTest[0] = (rawData[0] >> 3) | (rawData[3] & 0x30) >> 4 ; // XA_TEST result is a five-bit unsigned integer
  selfTest[1] = (rawData[1] >> 3) | (rawData[3] & 0x0C) >> 2 ; // YA_TEST result is a five-bit unsigned integer
  selfTest[2] = (rawData[2] >> 3) | (rawData[3] & 0x03) >> 0 ; // ZA_TEST result is a five-bit unsigned integer
  // Extract the gyration test results first
  selfTest[3] = rawData[0]  & 0x1F ; // XG_TEST result is a five-bit unsigned integer
  selfTest[4] = rawData[1]  & 0x1F ; // YG_TEST result is a five-bit unsigned integer
  selfTest[5] = rawData[2]  & 0x1F ; // ZG_TEST result is a five-bit unsigned integer
  // Process results to allow final comparison with factory set values
  factoryTrim[0] = (4096.0 * 0.34) * (pow( (0.92 / 0.34) , (((float)selfTest[0] - 1.0) / 30.0))); // FT[Xa] factory trim calculation
  factoryTrim[1] = (4096.0 * 0.34) * (pow( (0.92 / 0.34) , (((float)selfTest[1] - 1.0) / 30.0))); // FT[Ya] factory trim calculation
  factoryTrim[2] = (4096.0 * 0.34) * (pow( (0.92 / 0.34) , (((float)selfTest[2] - 1.0) / 30.0))); // FT[Za] factory trim calculation
  factoryTrim[3] =  ( 25.0 * 131.0) * (pow( 1.046 , ((float)selfTest[3] - 1.0) ));         // FT[Xg] factory trim calculation
  factoryTrim[4] =  (-25.0 * 131.0) * (pow( 1.046 , ((float)selfTest[4] - 1.0) ));         // FT[Yg] factory trim calculation
  factoryTrim[5] =  ( 25.0 * 131.0) * (pow( 1.046 , ((float)selfTest[5] - 1.0) ));         // FT[Zg] factory trim calculation

  //  Output self-test results and factory trim calculation if desired
  //  Serial.println(selfTest[0]); Serial.println(selfTest[1]); Serial.println(selfTest[2]);
  //  Serial.println(selfTest[3]); Serial.println(selfTest[4]); Serial.println(selfTest[5]);
  //  Serial.println(factoryTrim[0]); Serial.println(factoryTrim[1]); Serial.println(factoryTrim[2]);
  //  Serial.println(factoryTrim[3]); Serial.println(factoryTrim[4]); Serial.println(factoryTrim[5]);

  // Report results as a ratio of (STR - FT)/FT; the change from Factory Trim of the Self-Test Response
  for (int i = 0; i < 6; i++) {
    destination[i] = 100.0 + 100.0 * ((float)selfTest[i] - factoryTrim[i]) / factoryTrim[i]; // Report percent differences
  }

}

void writeByte(uint8_t address, uint8_t subAddress, uint8_t data)
{
  Wire.beginTransmission(address);  // Initialize the Tx buffer
  Wire.write(subAddress);           // Put slave register address in Tx buffer
  Wire.write(data);                 // Put data in Tx buffer
  Wire.endTransmission();           // Send the Tx buffer
}

uint8_t readByte(uint8_t address, uint8_t subAddress)
{
  uint8_t data; 						   // `data` will store the register data
  Wire.beginTransmission(address);         // Initialize the Tx buffer
  Wire.write(subAddress);                  // Put slave register address in Tx buffer
  Wire.endTransmission(false);             // Send the Tx buffer, but send a restart to keep connection alive
  Wire.requestFrom(address, (uint8_t) 1);  // Read one byte from slave register address
  data = Wire.read();                      // Fill Rx buffer with result
  return data;                             // Return data read from slave register
}

void readBytes(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t * dest)
{
  Wire.beginTransmission(address);   // Initialize the Tx buffer
  Wire.write(subAddress);            // Put slave register address in Tx buffer
  Wire.endTransmission(false);       // Send the Tx buffer, but send a restart to keep connection alive
  uint8_t i = 0;
  Wire.requestFrom(address, count);  // Read bytes from slave register address
  while (Wire.available()) {
    dest[i++] = Wire.read();
  }         // Put read results in the Rx buffer
}

//===================================================================================================
//                    Function for turning the motor off
//===================================================================================================
void turnOff() {
  ledState = HIGH;
  digitalWrite(ledPin, ledState);
  webserver.send(200, "text/plain", "LED off");
}

//===================================================================================================
//                    Function for turning the motor on
//===================================================================================================
void turnOn() {
  ledState = LOW;
  digitalWrite(ledPin, ledState);
  webserver.send(200, "text/plain", "LED on");
}

//===================================================================================================
//                    Function for toggling the state of the motor
//===================================================================================================
void toggle() {
  ledState = !ledState;
  digitalWrite(ledPin, ledState);
  webserver.send(200, "text/plain", "LED toggled");
}

//===================================================================================================
//                    Function for starting the measurement 
//===================================================================================================
void StartMeasurement() {
  if (!startflag) {
    startflag = true;
    count = 0;
    ledState = !ledState;
    digitalWrite(ledPin, ledState);
    //Serial.println("Measurement Started!");
    webserver.send(200, "text/plain", "Measurement Started!");
  }
  else {
    webserver.send(200, "text/plain", "Measurement already running!");
    Serial.println("Measurement already running!");
  }
}

//===================================================================================================
// 					Function for sending the IP & MAC address of ESP8266
//===================================================================================================
void SendIP_Mac() {

  page = "IP : " + WiFi.localIP().toString() + "\t";
  page += "MAC : " + WiFi.macAddress();
  // webserver.sendHeader("Access-Control-Allow-Origin", "*", true);
  webserver.send(200, "application/json", page);

}

//===================================================================================================
//                    Function for rebooting ESP8266
//===================================================================================================
void Reboot() {
  Serial.println("Rebooting ESP...");
  webserver.send(200, "text/plain", "Rebooting ESP...");
  ESP.restart();
}

//===================================================================================================
// Function for estimating device orienation based on
// S. O. H. Madgwick, A. J. L. Harrison and R. Vaidyanathan, "Estimation of IMU and MARG orientation 
// using a gradient descent algorithm," 2011 IEEE International Conference on Rehabilitation Robotics,
// 2011, pp. 1-7, doi: 10.1109/ICORR.2011.5975346.
//===================================================================================================
//void MadgwickQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz)
//{
//  float q1 = q[0], q2 = q[1], q3 = q[2], q4 = q[3];         // short name local variable for readability
//  float norm;                                               // vector norm
//  float f1, f2, f3;                                         // objetive function elements
//  float J_11or24, J_12or23, J_13or22, J_14or21, J_32, J_33; // objective function Jacobian elements
//  float qDot1, qDot2, qDot3, qDot4;
//  float hatDot1, hatDot2, hatDot3, hatDot4;
//  float gerrx, gerry, gerrz, gbiasx, gbiasy, gbiasz;        // gyro bias error
//
//  // Auxiliary variables to avoid repeated arithmetic
//  float _halfq1 = 0.5f * q1;
//  float _halfq2 = 0.5f * q2;
//  float _halfq3 = 0.5f * q3;
//  float _halfq4 = 0.5f * q4;
//  float _2q1 = 2.0f * q1;
//  float _2q2 = 2.0f * q2;
//  float _2q3 = 2.0f * q3;
//  float _2q4 = 2.0f * q4;
//  float _2q1q3 = 2.0f * q1 * q3;
//  float _2q3q4 = 2.0f * q3 * q4;
//
//  // Normalise accelerometer measurement
//  norm = sqrt(ax * ax + ay * ay + az * az);
//  if (norm == 0.0f) return; // handle NaN
//  norm = 1.0f / norm;
//  ax *= norm;
//  ay *= norm;
//  az *= norm;
//
//  // Compute the objective function and Jacobian
//  f1 = _2q2 * q4 - _2q1 * q3 - ax;
//  f2 = _2q1 * q2 + _2q3 * q4 - ay;
//  f3 = 1.0f - _2q2 * q2 - _2q3 * q3 - az;
//  J_11or24 = _2q3;
//  J_12or23 = _2q4;
//  J_13or22 = _2q1;
//  J_14or21 = _2q2;
//  J_32 = 2.0f * J_14or21;
//  J_33 = 2.0f * J_11or24;
//
//  // Compute the gradient (matrix multiplication)
//  hatDot1 = J_14or21 * f2 - J_11or24 * f1;
//  hatDot2 = J_12or23 * f1 + J_13or22 * f2 - J_32 * f3;
//  hatDot3 = J_12or23 * f2 - J_33 * f3 - J_13or22 * f1;
//  hatDot4 = J_14or21 * f1 + J_11or24 * f2;
//
//  // Normalize the gradient
//  norm = sqrt(hatDot1 * hatDot1 + hatDot2 * hatDot2 + hatDot3 * hatDot3 + hatDot4 * hatDot4);
//  hatDot1 /= norm;
//  hatDot2 /= norm;
//  hatDot3 /= norm;
//  hatDot4 /= norm;
//
//  // Compute estimated gyroscope biases
//  gerrx = _2q1 * hatDot2 - _2q2 * hatDot1 - _2q3 * hatDot4 + _2q4 * hatDot3;
//  gerry = _2q1 * hatDot3 + _2q2 * hatDot4 - _2q3 * hatDot1 - _2q4 * hatDot2;
//  gerrz = _2q1 * hatDot4 - _2q2 * hatDot3 + _2q3 * hatDot2 - _2q4 * hatDot1;
//
//  // Compute and remove gyroscope biases
//  gbiasx += gerrx * deltat * zeta;
//  gbiasy += gerry * deltat * zeta;
//  gbiasz += gerrz * deltat * zeta;
//  gx -= gbiasx;
//  gy -= gbiasy;
//  gz -= gbiasz;
//
//  // Compute the quaternion derivative
//  qDot1 = -_halfq2 * gx - _halfq3 * gy - _halfq4 * gz;
//  qDot2 =  _halfq1 * gx + _halfq3 * gz - _halfq4 * gy;
//  qDot3 =  _halfq1 * gy - _halfq2 * gz + _halfq4 * gx;
//  qDot4 =  _halfq1 * gz + _halfq2 * gy - _halfq3 * gx;
//
//  // Compute then integrate estimated quaternion derivative
//  q1 += (qDot1 - (beta * hatDot1)) * deltat;
//  q2 += (qDot2 - (beta * hatDot2)) * deltat;
//  q3 += (qDot3 - (beta * hatDot3)) * deltat;
//  q4 += (qDot4 - (beta * hatDot4)) * deltat;
//
//  // Normalize the quaternion
//  norm = sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);    // normalise quaternion
//  norm = 1.0f / norm;
//  q[0] = q1 * norm;
//  q[1] = q2 * norm;
//  q[2] = q3 * norm;
//  q[3] = q4 * norm;
//}
