/*
* Gear Indicator
* Could be used alone or with Arduino DataLogger
* Some information about interrupts:
* - http://arduino.cc/en/Reference/attachInterrupt
* About I2C:
* - WSWireLib: https://github.com/steamfire/WSWireLib
* - http://stackoverflow.com/questions/21073085/how-to-send-4-pot-values-via-i2c-from-arduino-to-arduino-how-to-differentiate-t
* - http://www.gammon.com.au/i2c
* About SD Cards:
* - https://learn.adafruit.com/adafruit-micro-sd-breakout-board-card-tutorial/format
* About MPU6050:
* - http://playground.arduino.cc/Main/MPU-6050
* - https://github.com/jrowberg/i2cdevlib/tree/master/Arduino/MPU6050
* - http://www.geekmomprojects.com/mpu-6050-redux-dmp-data-fusion-vs-complementary-filter/
* - http://42bots.com/tutorials/arduino-uno-and-the-invensense-mpu-6050-6dof-imu/
* Some useful information about Aprilia RSV/SL 1000 bikes work: (See: http://www.aprilia-v60.com/index.php?topic=262315)
* - Speed sensor: purple/green wire=Vcc ~ 9V ? / grey/white: signal ~6.5V, falling to 0V
* - Engine pulses/revolution: 3 if read from the dashboard wire
* - Engine pulses/revolution: 6 if crank shaft sensor is used
* - Wheel pulses/revolution: 5
*/

#include <SPI.h>
#include <SD.h>
//#include <WSWire.h>
#include <Wire.h>
#include <PinChangeInt.h>

#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

#define OUTPUT_READABLE_YAWPITCHROLL

#define ENGINE_PULSES_PER_REVOLUTION 3
#define WHEEL_PULSES_PER_REVOLUTION 5

#define GEAR_RATIO_OFFSET 0.1

#define TIME_LAPSE 100

#define I2C_ADDRESS 4

#define PIN_INT A3

#define FIRST_FILE_INDEX 1

volatile byte engine_pulses;
volatile byte wheel_pulses;

unsigned int engine_rpm;
unsigned int wheel_rpm;
unsigned int ratio;
unsigned int gear_ratio_table[6]; // ratio values for each gear

unsigned long lastmillis;

const int chipSelect = 4;

uint8_t i2c_values[12];

MPU6050 mpu;
// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3] = {0,0,0};           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmp_data_ready() {
  mpuInterrupt = true;
}

int file_index = FIRST_FILE_INDEX;

void setup() {

  // join I2C bus (I2Cdev library doesn't do this automatically)
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    Wire.begin();
    TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
    Fastwire::setup(400, true);
  #endif
  Serial.begin(115200);
  while (!Serial); // wait for Leonardo enumeration, others continue immediately
  // Verify MPU connection
  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

  // load and configure the DMP
  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();

  // supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);
  mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    // turn on the DMP, now that it's ready
    Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);

    // enable Arduino interrupt detection
    Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
//  attachInterrupt(0, dmpDataReady, RISING);
//  "Pin change" interrupt instead of the two hardware interrupts, already used for engine rev & wheel rev
    pinMode(PIN_INT, INPUT_PULLUP);
    attachPinChangeInterrupt(PIN_INT, dmp_data_ready, RISING);

    mpuIntStatus = mpu.getIntStatus();

    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    Serial.println(F("DMP ready! Waiting for first interrupt..."));
    dmpReady = true;

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }

  attachInterrupt(0, rpm_counter, FALLING);
  attachInterrupt(1, speed_counter, FALLING);

  engine_pulses = 0;
  wheel_pulses = 0;
  engine_rpm = 0;
  wheel_rpm = 0;
  ratio = 0;

  lastmillis = 0;

  Serial.print("Initializing SD card...");
  // On the Ethernet Shield, CS is pin 4. It's set as an output by default.
  // Note that even if it's not used as the CS pin, the hardware SS pin 
  // (10 on most Arduino boards, 53 on the Mega) must be left as an output 
  // or the SD library functions will not work. 
  pinMode(10, OUTPUT);

  if (!SD.begin(chipSelect)) {
    Serial.println("Initialization failed! ");
    return;
  }
  Serial.println("initialization done.");

  File settings_file = SD.open("settings.ini");
  if (settings_file) {
    char* index_ch = readline(settings_file);
    file_index = atoi(index_ch);
    Serial.print("Index found in file:");
    Serial.println(file_index);
    file_index++;
    // close the file:
    settings_file.close();
  }
  SD.remove("settings.ini");
  settings_file = SD.open("settings.ini", FILE_WRITE);
  if (settings_file) {
    settings_file.println(file_index);
    settings_file.close();
  }

// I2C Slave configuration
//  Wire.begin(I2C_ADDRESS);                // join i2c bus with address #4
//  Wire.onRequest(requestEvent); // register event

}

char* readline(File file) {
  int i=0; 
  char linebuffer[100];
  while (file.available() && i < 99) {
      linebuffer[i++] = file.read();
    }
  return linebuffer;   
}

/**
 * Every n-th ms, measure the engine revolutions per minutes as well as the rear wheel revolutions per minutes
 * Then, measure the ratio between the two in order to determine which gear is currently used.
 * Write theses values in a file stored on the SD card
 * Update the screen (LCD will be used during the tests, then a 7-segments display)
 * Send all the data using I2C protocol
 */
void loop() {

  // wait for MPU interrupt or extra packet(s) available
  while (!mpuInterrupt && fifoCount < packetSize) {
    long current_time_lapse = millis() - lastmillis;
    if (current_time_lapse >= TIME_LAPSE) {
      detachInterrupt(0);
      detachInterrupt(1);
    
      engine_rpm = (60/ENGINE_PULSES_PER_REVOLUTION)*1000/(current_time_lapse) * engine_pulses;
      wheel_rpm = (60/WHEEL_PULSES_PER_REVOLUTION)*1000/(current_time_lapse) * wheel_pulses;
      ratio = 100*engine_rpm/wheel_rpm;
      engine_pulses = 0;
      wheel_pulses = 0;
    
      lastmillis = millis();
    
      attachInterrupt(0, rpm_counter, FALLING);
      attachInterrupt(1, speed_counter, FALLING);
    
      prepare_i2c_buffer(engine_rpm, wheel_rpm, gear(ratio), ypr);
      save_values_to_disk(engine_rpm, wheel_rpm, gear(ratio), ypr);
    }
  }

  read_mpu();

  Serial.print("ypr\t");
  Serial.print(ypr[0] * 180/M_PI);
  Serial.print("\t");
  Serial.print(ypr[1] * 180/M_PI);
  Serial.print("\t");
  Serial.println(ypr[2] * 180/M_PI);

}

void read_mpu() {
  if (!mpuInterrupt && fifoCount < packetSize) {return;}
  // reset interrupt flag and get INT_STATUS byte
  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();

  // get current FIFO count
  fifoCount = mpu.getFIFOCount();

  // check for overflow (this should never happen unless our code is too inefficient)
  if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
    // reset so we can continue cleanly
    mpu.resetFIFO();
    Serial.println(F("FIFO overflow!"));

  // otherwise, check for DMP data ready interrupt (this should happen frequently)
  } else if (mpuIntStatus & 0x02) {
    // wait for correct available data length, should be a VERY short wait
    while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

    // read a packet from FIFO
    mpu.getFIFOBytes(fifoBuffer, packetSize);
    
    // track FIFO count here in case there is > 1 packet available
    // (this lets us immediately read more without waiting for an interrupt)
    fifoCount -= packetSize;

    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
  }

}

/**
 * Return the gear corresponding to the engine/wheel ratio
 */
int gear(int ratio) {
  for (byte b = 0; b < sizeof(gear_ratio_table); b++) {
    if ( (gear_ratio_table[b] * (1-GEAR_RATIO_OFFSET)) < ratio && (gear_ratio_table[b] * (1+GEAR_RATIO_OFFSET)) > ratio) {
      return b+1;
    }
  }
  return 0;
}

void prepare_i2c_buffer(int rpm, int wheel, int gear, float ypr[]) {
  i2c_values[0] = rpm &0xFF;
  i2c_values[1] = (rpm >> 8) &0xFF;

  i2c_values[2] = wheel &0xFF;
  i2c_values[3] = (wheel >> 8) &0xFF;

  i2c_values[4] = gear &0xFF;
  i2c_values[5] = (gear >> 8) &0xFF;

  int ypr0 = ypr[0] * 180/M_PI;
  i2c_values[6] = ypr0 &0xFF;
  i2c_values[7] = (ypr0 >> 8) &0xFF;

  int ypr1 = ypr[1] * 180/M_PI;
  i2c_values[8] = ypr1 &0xFF;
  i2c_values[9] = (ypr1 >> 8) &0xFF;

  int ypr2 = ypr[2] * 180/M_PI;
  i2c_values[10] = ypr2 &0xFF;
  i2c_values[11] = (ypr2 >> 8) &0xFF;
}

void save_values_to_disk(int rpm, int wheel, int gear, float ypr[]) {
  char values[150];
  char buffer[5];
  itoa(rpm, buffer, 10);
  strcpy(values, buffer);
  strcat(values, ";");
  itoa(wheel, buffer, 10);
  strcat(values, buffer);
  strcat(values, ";");
  itoa(gear, buffer, 10);
  strcat(values, buffer);
  strcat(values, ";");
  itoa((ypr[0] * 180/M_PI), buffer, 10);
  strcat(values, buffer);
  strcat(values, ";");
  itoa((ypr[1] * 180/M_PI), buffer, 10);
  strcat(values, buffer);
  strcat(values, ";");
  itoa((ypr[2] * 180/M_PI), buffer, 10);
  strcat(values, buffer);
  strcat(values, ";");

  write_line_to_disk(values);
}

void write_line_to_disk(char* data) {
  char file_name[20];
  strcpy(file_name, "data-");
  char buffer[5];
  itoa(file_index, buffer, 10);
  strcat(file_name, buffer);
  strcat(file_name, ".log");

  File file = SD.open(file_name, FILE_WRITE);
  if (file) {
    file.println(data);
    file.close();
  } else {
    Serial.print("Error to open/create file ");
    Serial.println(file_name);
  }
}

void request_event() {
  Wire.write(i2c_values, sizeof(i2c_values)); // respond with message of 6 bytes as expected by master
}

void rpm_counter() {
  engine_pulses++;
}

void speed_counter() {
  wheel_pulses++;
}
