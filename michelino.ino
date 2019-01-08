// Michelino
// Robot Vehicle firmware for the Arduino platform
//
// Copyright (c) 2013 by Miguel Grinberg
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is furnished
// to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
// FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
// COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN
// AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION
// WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

/**
 * @file michelino.ino
 * @brief Arduino robot vehicle firmware.
 * @author Miguel Grinberg
 */

#define LOGGING

// enable the proper drivers from the following
//#define ENABLE_ADAFRUIT_MOTOR_DRIVER
#define ENABLE_L298N_MOTOR_DRIVER

#define ENABLE_NEWPING_DISTANCE_SENSOR_DRIVER

#define US_SERVO

// constants
#define RUN_TIME 30                     /**< seconds the robot will run */
#define TOO_CLOSE 15                    /**< distance to obstacle in centimeters */
#define MAX_DISTANCE (TOO_CLOSE * 20)   /**< maximum distance to track with sensor */
#define RANDOM_ANALOG_PIN 5             /**< unused analog pin to use as random seed */

#ifdef ENABLE_ADAFRUIT_MOTOR_DRIVER
#include <AFMotor.h>
#include "adafruit_motor_driver.h"
#define LEFT_MOTOR_INIT 1
#define RIGHT_MOTOR_INIT 3
#endif

#ifdef ENABLE_L298N_MOTOR_DRIVER
#include <L298N.h>
#include "l298n_motor_driver.h"
#define LEFT_MOTOR_INIT 0, 6, 9
#define RIGHT_MOTOR_INIT 1, 10, 11
#endif

#ifdef ENABLE_NEWPING_DISTANCE_SENSOR_DRIVER
#include <NewPing.h>
#include "newping_distance_sensor.h"
#define DISTANCE_SENSOR_INIT 3, 2, MAX_DISTANCE
#endif

#ifdef US_SERVO
#include <Servo.h>
#include "180degrees_servo_driver.h"
#define US_SERVO_INIT 4
#endif

#include "logging.h"
#include "moving_average.h"

int rightMotorCorr = 2;
int leftMotorCorr = 0;

int usScan=2;
int usScanArray[5];
int usAngles[5] = {0, 45, 90, 135, 180};

namespace Michelino
{
    class Robot
    {
    public:
        /*
         * @brief Class constructor.
         */
        Robot()
            : leftMotor(LEFT_MOTOR_INIT), rightMotor(RIGHT_MOTOR_INIT),
              distanceSensor(DISTANCE_SENSOR_INIT),
              distanceAverage(TOO_CLOSE * 10),
              USMotor(US_SERVO_INIT)
        {
            initialize();
        }
        
        /*
         * @brief Initialize the robot state.
         */
        void initialize()
        {
            USMotor.attach(US_SERVO_INIT);
            randomSeed(analogRead(RANDOM_ANALOG_PIN));
            endTime = millis() + RUN_TIME * 1000;
            move();
        }
        
        /*
         * @brief Update the state of the robot based on input from sensor and remote control.
         *  Must be called repeatedly while the robot is in operation.
         */
        void run()
        {
            USMotor.write(usAngles[usScan]);
            usScan += 1;
            if (usScan == 6) {usScan=0;}

            unsigned long currentTime = millis();
            log("state: %d, currentTime: %lu, actual angle: %u\n", state, currentTime, usAngles[usScan]);
            
            if (stopped())
                return;

            currentTime = millis();
            int distance = distanceAverage.add(distanceSensor.getDistance());
            log("state: %d, currentTime: %lu, distance: %u\n", state, currentTime, distance);
            
            if (doneRunning(currentTime))
                stop();
            else if (moving()) {
                if (obstacleAhead(distance))
                    turn(currentTime);
            }
            else if (turning()) {
                if (doneTurning(currentTime, distance))
                    move();
            }
        }

    protected:
        void move()
        {
            leftMotor.setSpeed(255-rightMotorCorr);
            rightMotor.setSpeed(255-leftMotorCorr);
            state = stateMoving;
        }
        
        void stop()
        {
            leftMotor.setSpeed(0);
            rightMotor.setSpeed(0);
            state = stateStopped;
        }
        
        bool doneRunning(unsigned long currentTime)
        {
            return (currentTime >= endTime);
        }
        
        bool obstacleAhead(unsigned int distance)
        {
            return (distance <= TOO_CLOSE);
        }
        
        bool turn(unsigned long currentTime)
        {
            if (random(2) == 0) {
                leftMotor.setSpeed(-255+leftMotorCorr);
                rightMotor.setSpeed(255-rightMotorCorr);
            }
            else {
                leftMotor.setSpeed(255-leftMotorCorr);
                rightMotor.setSpeed(-255+rightMotorCorr);
            }
            state = stateTurning;
            endStateTime = currentTime + random(200, 1000);
        }
        
        bool doneTurning(unsigned long currentTime, unsigned int distance)
        {
            if (currentTime >= endStateTime)
                return (distance > TOO_CLOSE);
            return false;
        }
        
        bool moving() { return (state == stateMoving); }
        bool turning() { return (state == stateTurning); }
        bool stopped() { return (state == stateStopped); }

    private:
        Motor leftMotor;
        Motor rightMotor;
        ServoMotor USMotor;
        DistanceSensor distanceSensor;
        MovingAverage<unsigned int, 3> distanceAverage;
        enum state_t { stateStopped, stateMoving, stateTurning };
        state_t state;
        unsigned long endStateTime;
        unsigned long endTime;
    };
};

Michelino::Robot robot;

void setup()
{
    //Serial.begin(9600);
    Serial.begin(19200);
    robot.initialize();
}

void loop()
{
    robot.run();
}
