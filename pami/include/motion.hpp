#ifndef MOTION_HPP
#define MOTION_HPP

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

protected:
    /**
     * @brief Variables representing the GPIO pin on the ESP-S3-Zero
     */
    static const int servo_pin_left_wheel_ = 4;
    static const int servo_pin_right_wheel_ = 5;
    static const int servo_pin_arm_ = 6;

    /**
     * @brief Variables representing the servos
     */
    Servo servo_left_wheel_;
    Servo servo_right_wheel_;
    Servo servo_arm_;
};
#endif // MY_CLASS_HPP
