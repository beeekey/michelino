/**
 * @file adafruit_motor_driver.h
 * @brief Motor device driver for the Adafruit motor shield.
 * @author Miguel Grinberg
 */

#include "motor_driver.h"

namespace Michelino
{
    class Motor : public MotorDriver
    {
    public:
        /*
         * @brief Class constructor.
         * @param number the DC motor number to control, from 1 to 4.
         */
        Motor(int EN, int IN1, int IN2)
            : MotorDriver(), motor(EN, IN1, IN2), currentSpeed(0)
        {
        }

        void setSpeed(int speed)
        {
            currentSpeed = speed;
            if (speed >= 0) {
                motor.setSpeed(speed);
                motor.run(L298N::FORWARD);
            }
            else if (speed == 0)
            {
              motor.stop();
            }
            else {
                motor.setSpeed(-speed);
                motor.run(L298N::BACKWARD);
            }
        }
        
        int getSpeed() const
        {
            return currentSpeed;
        }

        void stop() 
        {
          motor.stop();
        }
        
    private:
        L298N motor;
        int currentSpeed;
    };
};
