#ifndef MOTION_HPP
#define MOTION_HPP


/**
 * @brief Variables representing the max and min speed and angle
 */
#define MOTION__SERVO_MAX_SPEED 180
#define MOTION__SERVO_MIN_SPEED 0
#define MOTION__SERVO_MAX_ANGLE 120
#define MOTION__SERVO_MIN_ANGLE 45
#define MOTION__DEGREE_TO_RAD_RATIO PI / 180

//Temporary value in meters
#define MOTION__PAMI_WIDTH 0.08

/**
 * @class Motion
 * @brief Control all servos and mouvements
 */
class Motion
{
public:
    /**
     * @brief A constructor
     */
    Motion();

    /**
     * @brief Setup all servos
     */
    void setup();

    /**
     * @brief Make all servos sweep once
     * @param a a test value not used
     */
    void sweep(int a);

    /**
     * @brief Move the auxilary servo back and forth
     */
    void auxilary_action();

    /**
     * @brief Move the pami
     * @param speed the speed at which the pami should move in %
     * @param angle the angle the pami should rotate in degrees
     * @param distance the distance the pami should traverse in m
     */
    void movement(float percent_speed, float angle, float distance);

protected:
    /**
     * @brief Variables representing the GPIO pin on the ESP-S3-Zero
     */
    static const int servo_pin_right_wheel_ = 3;
    static const int servo_pin_left_wheel_ = 4;
    static const int servo_pin_arm_ = 5;

    /**
     * @brief Variables representing the servos
     */
    Servo servo_left_wheel_;
    Servo servo_right_wheel_;
    Servo servo_arm_;

};
#endif // MOTION_HPP
