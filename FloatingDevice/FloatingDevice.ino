
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

// Select which 'port' M1, M2, M3 or M4. In this case, M1
Adafruit_DCMotor *left_propeller = AFMS.getMotor(2);
// You can also make another motor on port M2
Adafruit_DCMotor *right_propeller = AFMS.getMotor(1);

// GPS module connection
// Connect the GPS Power pin to 5V
// Connect the GPS Ground pin to ground
// If using software serial (sketch example default):
//   Connect the GPS TX (transmit) pin to Digital 2
//   Connect the GPS RX (receive) pin to Digital 3

// If using software serial, keep these lines enabled
// (you can change the pin numbers to match your wiring):
#if ARDUINO >= 100
SoftwareSerial mySerial(2, 3);
#else
NewSoftSerial mySerial(2, 3);
#endif
Adafruit_GPS GPS(&mySerial);

// implement a state machine
#define WAITING_GPS 0
#define STRAIGHT 1
#define TURN 2
#define STOPPED 3
int state = WAITING_GPS;

// the last position we recorded
float last_lon, last_lat;
float cur_lat, cur_lon;

// The waypoint array
// implement this with SD card later
int next_waypoint = 0;
#define N_WAYPOINTS 5
// 5 points around plage du pelican
float waypoints_lat[N_WAYPOINTS] = { 46.513279, 46.513058, 46.513103, 46.513377, 46.513414 };
float waypoints_lon[N_WAYPOINTS] = { 6.572326, 6.572336, 6.573065, 6.573107, 6.572770 };
// how close we need to be to a waypoint before we consider arrived
#define WAYPOINT_RADIUS 3.3

// Sampling interval for the GPS
unsigned long last_time;
#define TIME_WAITING 5000
#define TIME_STRAIGHT 10000
#define TIME_TURN 2000
#define MOTOR_SPEED_LEFT 100
#define MOTOR_SPEED_RIGHT 100

void setup()
{
  Serial.begin(57600);

  // 9600 NMEA is the default baud rate for MTK - some use 4800
  GPS.begin(9600);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);

  // Start the motor
  AFMS.begin();  // create with the default frequency 1.6KHz

  // stop the motors
  motor_stop();

  // start counter
  last_time = millis();

  Serial.println("Hello Float.");
}

void loop()
{
  // Receive from GPS
  char c = GPS.read();

  // if a sentence is received, we can check the checksum, parse it...
  if (GPS.newNMEAreceived()) {
    // a tricky thing here is if we print the NMEA sentence, or data
    // we end up not listening and catching other sentences! 
    // so be very wary if using OUTPUT_ALLDATA and trytng to print out data

    if (!GPS.parse(GPS.lastNMEA()))   // this also sets the newNMEAreceived() flag to false
      return;  // we can fail to parse a sentence in which case we should just wait for another

    // Convert silly GPS lat/lon format to degrees
    int lat_i = (int)(GPS.latitude/100);
    float lat_f = (GPS.latitude-(lat_i*100.))/60.;
    cur_lat = (lat_i + lat_f);

    int lon_i = (int)(GPS.longitude/100);
    float lon_f = (GPS.longitude-(lon_i*100.))/60.;
    cur_lon = (lon_i + lon_f);
  }

  switch (state)
  {
    // GPS finally started
    case WAITING_GPS:
      if (GPS.fix)
      {
        Serial.println("Acquired GPS fix.");
        // if we just started, just recorde the starting point
        last_lat = cur_lat;
        last_lon = cur_lon;

        // go into straight driving mode
        state = STRAIGHT;
        motor_go_straight();
        last_time = millis();
      }
      else if (ellapsed_millis(last_time) > TIME_WAITING)
      {
        Serial.println("No GPS yet.");
        last_time = millis();
      }
      break;

    case STRAIGHT:
      if (ellapsed_millis(last_time) > TIME_STRAIGHT)
      {
        // STOP moving straight
        motor_stop();

        if (GPS.fix)
        {
          Serial.print("Current lat,lon: ");
          Serial.print(cur_lat, 6);
          Serial.print(",");
          Serial.println(cur_lon, 6);
          Serial.print("Next lat,lon: ");
          Serial.print(waypoints_lat[next_waypoint], 6);
          Serial.print(",");
          Serial.println(waypoints_lon[next_waypoint], 6);

          // transform angles to radian
          float lat_now = cur_lat/180.*M_PI;
          float lon_now = cur_lon/180.*M_PI;
          float lat_way = waypoints_lat[next_waypoint]/180.*M_PI;
          float lon_way = waypoints_lon[next_waypoint]/180.*M_PI;

          // compute distance to next waypoint using haversine formula
          float s_lat = sin((lat_now-lat_way)/2);
          float s_lon = sin((lon_now-lon_way)/2);
          float a = s_lat*s_lat + cos(lat_now)*cos(lat_way)*s_lon*s_lon;
          float c = 2*atan2(sqrt(a), sqrt(1-a));
          float distance = RADIUS_EARTH*c;

          Serial.print("Distance to next point :");
          Serial.println(distance);

          if (distance < WAYPOINT_RADIUS)
          {
            // MAYBE DO SOMETHING HERE

            // go to next waypoint
            next_waypoint++;

            // update waypoint
            if (next_waypoint < N_WAYPOINTS)
            {
              lat_way = waypoints_lat[next_waypoint]/180.*M_PI;
              lon_way = waypoints_lon[next_waypoint]/180.*M_PI;
            }
            else
            {
              // we got to the last waypoint, do nothing
              Serial.println("Arrived to destination.");
              state = STOPPED;
            }
          }

          if (next_waypoint < N_WAYPOINTS)
          {
            // Now see where you should be going next
            float nx = -(lon_way-last_lon);
            float ny = lat_way-last_lat;
            float dx = lat_now - last_lat;
            float dy = lon_now - last_lon;
            int go_left = nx*dx + ny*dy > 0;

            if (go_left)
              Serial.println("Going left");
            else
              Serial.println("Going right");

            // go into turning mode
            state = TURN;
            motor_turn(go_left);
            last_time = millis();
          }

          // save current location
          last_lat = lat_now;
          last_lon = lon_now;
        }
        else
        {
          Serial.println("Lost GPS fix.");
          state = WAITING_GPS;
          last_time = millis();
        }
      }
      break;

    case TURN:
      if (ellapsed_millis(last_time) > TIME_TURN)
      {
        // stop the motor
        motor_stop();
        
        // go into driving straight mode
        state = STRAIGHT;
        motor_go_straight();
        last_time = millis();

      }
      break;
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

void motor_go_straight()
{
  // Set the speed to start, from 0 (off) to 255 (max speed)
  left_propeller->setSpeed(MOTOR_SPEED_LEFT);
  left_propeller->run(FORWARD);

  right_propeller->setSpeed(MOTOR_SPEED_RIGHT);
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
    left_propeller->run(BACKWARD);
  }
  else
  {
    left_propeller->run(FORWARD);
    right_propeller->run(BACKWARD);
  }
}

