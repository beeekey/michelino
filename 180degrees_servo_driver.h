/**
 * @file adafruit_motor_driver.h
 * @brief Motor device driver for the Adafruit motor shield.
 * @author Miguel Grinberg
 */

#include "servo_driver.h"

namespace Michelino
{
    class ServoMotor : public ServoDriver
    {
    public:
        /*
         * @brief Class constructor.
         * @param number the DC motor number to control, from 1 to 4.
         */
        ServoMotor(int Pin)
            : ServoDriver(), servo(), currentPos(0)
        {
        }

        void write(int angle)
        {
            servo.write(angle);
        }

        void attach(int pin)
        {
            servo.attach(pin);  
        }
        
        int read()
        {
            return servo.read();
        }

        void scan()
        {
          return;
        }
        
    private:
        Servo servo;
        int currentPos;
    };
};
