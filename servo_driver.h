/**
 * @file servo_driver.h
 * @brief Motor device driver definition for the Michelino robot.
 * @author Ben Koch
 */

namespace Michelino
{
    class ServoDriver
    {
    public:
        /**
         * @brief Change the position of the motor.
         * @param angle The new angle of the motor.
         *  Valid values are between 0 and 180. 
         */
        virtual void write(int angle) = 0;
        
        /**
         * @brief Return the current angle of the motor.
         * @return The current angle of the motor with range 0 to 180.
         */
        virtual int read();            
    };
};
