#include <Servo.h>
#include "motion.hpp"

Motion::Motion(){}

void Motion::setup()
{
    servo_left_wheel_.attach(
        servo_pin_left_wheel_,
        Servo::CHANNEL_NOT_ATTACHED,
        45,
        120
    );
    /**
    servo_right_wheel_.attach(
        servo_pin_right_wheel_,
        Servo::CHANNEL_NOT_ATTACHED,
        45,
        120
    );

    servo_arm_.attach(
        servo_pin_arm_,
        Servo::CHANNEL_NOT_ATTACHED,
        45,
        120
    );*/

    Serial.println("Setup done");
}

void Motion::sweep(int a)
{

    for (int posDegrees = 0; posDegrees <= 180; posDegrees++) {
        servo_left_wheel_.write(posDegrees);
        Serial.println(posDegrees);
        delay(20);
    }

    for (int posDegrees = 180; posDegrees >= 0; posDegrees--) {
        servo_left_wheel_.write(posDegrees);
        Serial.println(posDegrees);
        delay(20);
    }
}
