#include <Servo.h>
#include "motion.hpp"

Motion::Motion() {}

void Motion::setup()
{
    servo_left_wheel_.attach(
        servo_pin_left_wheel_,
        Servo::CHANNEL_NOT_ATTACHED,
        MOTION__SERVO_MIN_SPEED,
        MOTION__SERVO_MAX_SPEED
    );

    servo_right_wheel_.attach(
        servo_pin_right_wheel_,
        Servo::CHANNEL_NOT_ATTACHED,
        MOTION__SERVO_MIN_SPEED,
        MOTION__SERVO_MAX_SPEED
    );

    servo_arm_.attach(
        servo_pin_arm_,
        Servo::CHANNEL_NOT_ATTACHED,
        MOTION__SERVO_MIN_ANGLE,
        MOTION__SERVO_MAX_ANGLE
    );

    Serial.println("Setup done");
}

void Motion::sweep(int a)
{

    for (int posDegrees = 0; posDegrees <= 180; posDegrees++) {
        servo_left_wheel_.write(posDegrees);
        servo_right_wheel_.write(posDegrees);
        Serial.println(posDegrees);
        delay(20);
    }

    for (int posDegrees = 180; posDegrees >= 0; posDegrees--) {
        servo_left_wheel_.write(posDegrees);
        servo_right_wheel_.write(posDegrees);
        Serial.println(posDegrees);
        delay(20);
    }
}

void Motion::auxilary_action()
{
    int current_pos = servo_arm_.read();
    if (current_pos == MOTION__SERVO_MIN_ANGLE) {
        servo_arm_.write(MOTION__SERVO_MAX_ANGLE);
    }
    else {
        servo_arm_.write(MOTION__SERVO_MIN_ANGLE);
    }
}

void Motion::movement(float percent_speed, float angle, float distance)
{
    float speed = percent_speed * MOTION__SERVO_MAX_SPEED;
    float speed_left_wheel = 0;
    float speed_right_wheel = 0;

    if (angle == 0) {
        speed_left_wheel = speed;
        speed_right_wheel = speed;
    }
    else if (distance == 0) {
        speed_left_wheel = speed;
        speed_right_wheel = -speed;
    }
    else if (angle > 0) {
        float radius = distance / (angle * MOTION__DEGREE_TO_RAD_RATIO);
        speed_right_wheel = (1 + MOTION__PAMI_WIDTH / (2 * radius)) * speed;
        speed_left_wheel = (1 - MOTION__PAMI_WIDTH / (2 * radius)) * speed;
    }
    else {
        float radius = distance / (angle * MOTION__DEGREE_TO_RAD_RATIO);
        speed_right_wheel = (1 - MOTION__PAMI_WIDTH / (2 * radius)) * speed;
        speed_left_wheel = (1 + MOTION__PAMI_WIDTH / (2 * radius)) * speed;
    }

    if (speed_right_wheel > MOTION__SERVO_MAX_SPEED || speed_left_wheel > MOTION__SERVO_MAX_SPEED) {
        speed_right_wheel *= MOTION__SERVO_MAX_SPEED / max(speed_right_wheel, speed_left_wheel);
        speed_left_wheel *= MOTION__SERVO_MAX_SPEED / max(speed_right_wheel, speed_left_wheel);
    }

    servo_right_wheel_.write(speed_right_wheel);
    servo_left_wheel_.write(speed_left_wheel);
}
