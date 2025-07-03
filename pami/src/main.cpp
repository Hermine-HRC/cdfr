#include <Servo.h>
#include "motion.hpp"
#include "BLEDevice.h"
#include "robot_communication.hpp"

/*
 * Description:
 * Example for setting the minimal and maximal angle.
 */

Motion testMotion;
RobotCommunication testComm;


void setup()
{
    delay(5000);
    Serial.begin(115200);

    //delay(5000);
    Serial.println("Main setup start");

    //testMotion.setup();

    //Serial.println("Main setup done");
    testComm.setup();

}

void loop()
{
    Serial.println("0");
    //testMotion.auxilary_action();
    //testMotion.movement(0.1, 90, 1);
    std::string instruction = testComm.readServer();
    Serial.println(instruction.c_str());
    delay(1000); // Delay a second between loops.
}
