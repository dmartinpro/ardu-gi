/*
* Gear Indicator
* Could be used alone or with Arduino DataLogger
* Some information about interrupts:
* - http://arduino.cc/en/Reference/attachInterrupt
* About I2C:
* - WSWireLib: https://github.com/steamfire/WSWireLib
* - http://stackoverflow.com/questions/21073085/how-to-send-4-pot-values-via-i2c-from-arduino-to-arduino-how-to-differentiate-t
* About SD Cards:
* - https://learn.adafruit.com/adafruit-micro-sd-breakout-board-card-tutorial/format
* About MPU6050:
* - http://playground.arduino.cc/Main/MPU-6050
* - https://github.com/jrowberg/i2cdevlib/tree/master/Arduino/MPU6050
* - http://www.geekmomprojects.com/mpu-6050-redux-dmp-data-fusion-vs-complementary-filter/
* Some useful information about Aprilia RSV/SL 1000 bikes work: (See: http://www.aprilia-v60.com/index.php?topic=262315)
* - Speed sensor: purple/green wire=Vcc ~ 9V ? / grey/white: signal ~6.5V, falling to 0V
* - Engine pulses/revolution: 3 if read from the dashboard wire
* - Engine pulses/revolution: 6 if crank shaft sensor is used
* - Wheel pulses/revolution: 5
*/

#include <SPI.h>
#include <SD.h>
#include <WSWire.h>

#define ENGINE_PULSES_PER_REVOLUTION 3
#define WHEEL_PULSES_PER_REVOLUTION 5

#define GEAR_RATIO_OFFSET 0.1

#define TIME_LAPSE 100

#define I2C_ADDRESS 4

volatile byte engine_pulses;
volatile byte wheel_pulses;

unsigned int engine_rpm;
unsigned int wheel_rpm;
unsigned int ratio;
unsigned int gear_ratio_table[6]; // ratio values for each gear

unsigned long lastmillis;

const int chipSelect = 4;

uint8_t i2c_values[6];


void setup() {
  Serial.begin(9600);

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

  // I2C Slave configuration
  Wire.begin(I2C_ADDRESS);                // join i2c bus with address #4
  Wire.onRequest(requestEvent); // register event
}

/**
 * Every n-th ms, measure the engine revolutions per minutes as well as the rear wheel revolutions per minutes
 * Then, measure the ratio between the two in order to determine which gear is currently used.
 * Write theses values in a file stored on the SD card
 * Update the screen (LCD will be used during the tests, then a 7-segments display)
 * Send all the data using I2C protocol
 */
void loop() {

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

    saveValues(engine_rpm, wheel_rpm, gear(ratio));
    writeToFile("test");
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

void saveValues(int rpm, int wheel, int gear) {
  i2c_values[0] = rpm &0xFF;
  i2c_values[1] = (rpm >> 8) &0xFF;

  i2c_values[2] = wheel &0xFF;
  i2c_values[3] = (wheel >> 8) &0xFF;

  i2c_values[4] = gear &0xFF;
  i2c_values[5] = (gear >> 8) &0xFF;
}

void writeToFile(char* data) {
  File file = SD.open("data.log", FILE_WRITE);
  if (file) {
    file.println(data);
    file.close();
  } else {
    Serial.println("Error to open/create file");
  }
}

void requestEvent() {
  Wire.write(i2c_values, sizeof(i2c_values)); // respond with message of 6 bytes as expected by master
}

void rpm_counter() {
  engine_pulses++;
}

void speed_counter() {
  wheel_pulses++;
}
