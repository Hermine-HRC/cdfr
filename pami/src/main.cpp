#include <Servo.h>
#include "motion.hpp"
/*
 * Description:
 * Example for setting the minimal and maximal angle.
 */

Motion testMotion;

void setup()
{
    Serial.begin(115200);
    delay(5000);
    Serial.println("Main setup start");

    testMotion.setup();

    Serial.println("Main setup done");
}

void loop()
{
    Serial.println("0");
    testMotion.auxilary_action();
    testMotion.mouvement(0.1, 90, 1);
}
