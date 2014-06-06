
#include <limits.h>
#include <math.h>

#define M_PI 3.14159265
#define RADIUS_EARTH 6367444.7

#include <Adafruit_GPS.h>
#if ARDUINO >= 100
#include <SoftwareSerial.h>
#else
// Older Arduino IDE requires NewSoftSerial, download from:
// http://arduiniana.org/libraries/newsoftserial/
// #include <NewSoftSerial.h>
// DO NOT install NewSoftSerial if using Arduino 1.0 or later!
#endif

#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_PWMServoDriver.h"

// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 
// Or, create it with a different I2C address (say for stacking)
// Adafruit_MotorShield AFMS = Adafruit_MotorShield(0x61); 

// Select which 'port' M1, M2, M3 or M4. In this case, M1
Adafruit_DCMotor *left_propeller = AFMS.getMotor(1);
// You can also make another motor on port M2
Adafruit_DCMotor *right_propeller = AFMS.getMotor(2);

// Connect the GPS Power pin to 5V
// Connect the GPS Ground pin to ground
// If using software serial (sketch example default):
//   Connect the GPS TX (transmit) pin to Digital 3
//   Connect the GPS RX (receive) pin to Digital 2
// If using hardware serial (e.g. Arduino Mega):
//   Connect the GPS TX (transmit) pin to Arduino RX1, RX2 or RX3
//   Connect the GPS RX (receive) pin to matching TX1, TX2 or TX3

// If using software serial, keep these lines enabled
// (you can change the pin numbers to match your wiring):
#if ARDUINO >= 100
SoftwareSerial mySerial(3, 2);
#else
NewSoftSerial mySerial(3, 2);
#endif
Adafruit_GPS GPS(&mySerial);
// If using hardware serial (e.g. Arduino Mega), comment
// out the above six lines and enable this line instead:
//Adafruit_GPS GPS(&Serial1);

// Set GPSECHO to 'false' to turn off echoing the GPS data to the Serial console
// Set to 'true' if you want to debug and listen to the raw GPS sentences. 
#define GPSECHO  true

int started = 0;
float last_lon, last_lat;

int next_waypoint = 0;
#define N_WAYPOINTS 5
float waypoints_lat[N_WAYPOINTS] = { 46.513279, 46.513058, 46.513103, 46.513377, 46.513414 };
float waypoints_lon[N_WAYPOINTS] = { 6.572326, 6.572336, 6.573065, 6.573107, 6.572770 };
#define WAYPOINT_RADIUS 3.3

// Sampling interval for the GPS
#define GPS_SAMPLING 60000
unsigned long last_time;

#define TIME_TURN 1000
#define MOTOR_SPEED_LEFT 10
#define MOTOR_SPEED_RIGHT 10

void setup()
{
  Serial.begin(57600);

  // 9600 NMEA is the default baud rate for MTK - some use 4800
  GPS.begin(9600);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);

  // the nice thing about this code is you can have a timer0 interrupt go off
  // every 1 millisecond, and read data from the GPS for you. that makes the
  // loop code a heck of a lot easier!
  useInterrupt(true);

  // Start the motor
  AFMS.begin();  // create with the default frequency 1.6KHz

  // start counter
  last_time = millis();
}

void loop()
{

  // if a sentence is received, we can check the checksum, parse it...
  if (GPS.newNMEAreceived()) {
    // a tricky thing here is if we print the NMEA sentence, or data
    // we end up not listening and catching other sentences! 
    // so be very wary if using OUTPUT_ALLDATA and trytng to print out data
    //Serial.println(GPS.lastNMEA());   // this also sets the newNMEAreceived() flag to false

    if (!GPS.parse(GPS.lastNMEA()))   // this also sets the newNMEAreceived() flag to false
      return;  // we can fail to parse a sentence in which case we should just wait for another
  }

  // when counter expires, do the interesting stuff
  if (ellapsed_millis(last_time) > GPS_SAMPLING)
  {
    // STOP moving straight
    motor_stop();

    // Process geographical coordinate
    if (!started)
    {
      // if we just started, just recorde the starting point
      last_lon = GPS.longitude/180.*M_PI;
      last_lat = GPS.latitude/180.*M_PI;
    }
    else
    {
      // transform angles to radian
      float lat_now = GPS.latitude/180.*M_PI;
      float lon_now = GPS.longitude/180.*M_PI;
      float lat_way = waypoints_lat[next_waypoint]/180.*M_PI;
      float lon_way = waypoints_lon[next_waypoint]/180.*M_PI;

      // compute distance to next waypoint using haversine formula
      float s_lat = sin((lat_now-lat_way)/2);
      float s_lon = sin((lon_now-lon_way)/2);
      float a = s_lat*s_lat + cos(GPS.latitude)*cos(waypoints_lat[next_waypoint])*s_lon*s_lon;
      float c = 2*atan2(sqrt(a), sqrt(1-a));
      float distance = RADIUS_EARTH*c;

      if (distance < WAYPOINT_RADIUS)
      {
        // MAYBE DO SOMETHING HERE

        // go to next waypoint
        next_waypoint++;
      }
      else if (next_waypoint < N_WAYPOINTS)
      {
        // Now see where you should be going next
        float nx = -(lon_way-last_lon);
        float ny = lat_way-last_lat;
        float dx = lat_now - last_lat;
        float dy = lon_now - last_lon;
        int go_left = nx*dx + ny*dy > 0;

        motor_turn(go_left);
        delay(TIME_TURN);
        motor_stop();
      }

      // Now start moving straight
      motor_go_straight(MOTOR_SPEED_LEFT, MOTOR_SPEED_RIGHT);

      // reset timer
      last_time = millis();

      // save current location
      last_lat = lat_now;
      last_lon = lon_now;

    }
  }
}

// Interrupt is called once a millisecond, looks for any new GPS data, and stores it
SIGNAL(TIMER0_COMPA_vect) {
  char c = GPS.read();
  // if you want to debug, this is a good time to do it!
  if (GPSECHO)
    if (c) UDR0 = c;  
  // writing direct to UDR0 is much much faster than Serial.print 
  // but only one character can be written at a time. 
}

void useInterrupt(boolean v) {
  if (v) {
    // Timer0 is already used for millis() - we'll just interrupt somewhere
    // in the middle and call the "Compare A" function above
    OCR0A = 0xAF;
    TIMSK0 |= _BV(OCIE0A);
    //usingInterrupt = true;
  } else {
    // do not call the interrupt function COMPA anymore
    TIMSK0 &= ~_BV(OCIE0A);
    //usingInterrupt = false;
  }
}

// quick helper that compute securely ellapsed time
unsigned long ellapsed_millis(unsigned long start)
{
  unsigned long now = millis();

  if (now >= start)
    return now - start;
  else
    return (ULONG_MAX + now - start);
}

void motor_go_straight(int speed_motor_left, int speed_motor_right)
{
  // Set the speed to start, from 0 (off) to 255 (max speed)
  left_propeller->setSpeed(speed_motor_left);
  left_propeller->run(FORWARD);

  right_propeller->setSpeed(speed_motor_right);
  right_propeller->run(FORWARD);
}

void motor_stop()
{
  left_propeller->run(RELEASE);
  right_propeller->run(RELEASE);
}

void motor_turn(int go_left)
{
  if (go_left)
  {
    right_propeller->run(FORWARD);
  }
  else
  {
    left_propeller->run(FORWARD);
  }
}

