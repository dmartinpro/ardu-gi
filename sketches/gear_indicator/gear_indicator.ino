/*
* Gear Indicator
* Could be used alone or with Arduino DataLogger
* Some useful information about Aprilia RSV/SL bikes work: (See: http://www.aprilia-v60.com/index.php?topic=262315)
* - Speed sensor: purple/green wire=Vcc ~ 9V ? / grey/white: signal ~6.5V, falling to 0V
* - Engine pulses/revolution: 3 if read from the dashboard wire
* - Engine pulses/revolution: 6 if crank shaft sensor is used
* - Wheel pulses/revolution: 5
*/

#define ENGINE_PULSES_PER_REVOLUTION 3
#define WHEEL_PULSES_PER_REVOLUTION 5

#define TIME_LAPSE 100

volatile byte engine_pulses;
volatile byte wheel_pulses;

unsigned int engine_rpm;
unsigned int wheel_rpm;
unsigned float ratio;

unsigned long lastmillis;

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
    ratio = engine_rpm/wheel_rpm;
    engine_pulses = 0;
    wheel_pulses = 0;

    lastmillis = millis();

    attachInterrupt(0, rpm_counter, FALLING);
    attachInterrupt(1, speed_counter, FALLING);
  }
  
}
void rpm_counter() {
  engine_pulses++;
}

void speed_counter() {
  wheel_pulses++;
}