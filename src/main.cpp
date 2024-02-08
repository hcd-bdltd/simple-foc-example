/**
 *  Hall sensor example code
 *
 * This is a code intended to test the hall sensors connections
 * and to demonstrate the hall sensor setup.
 *
 */

#include <SimpleFOC.h>

#define POLE_PAIRS 7

// Hall sensor instance
// HallSensor(int hallA, int hallB , int hallC, int pp)
//  - hallA, hallB, hallC    - HallSensor A, B and C pins
//  - pp                     - pole pairs
HallSensor sensor = HallSensor(A_HALL1, A_HALL2, A_HALL3, POLE_PAIRS);

// Interrupt routine initialisation
// channel A, B and C callbacks
void doA(){sensor.handleA();}
void doB(){sensor.handleB();}
void doC(){sensor.handleC();}

void setup() {
        // monitoring port
        Serial.begin(115200);

        // check if you need internal pullup
        sensor.pullup = Pullup::USE_EXTERN;

        // initialise encoder hardware
        sensor.init();
        // hardware interrupt enable
        sensor.enableInterrupts(doA, doB, doC);

        Serial.println("Sensor ready");
        _delay(1000);
}

void loop() {
        // iterative function updating the sensor internal variables
        // it is usually called in motor.loopFOC()
        sensor.update();
        // display the angle and the angular velocity to the terminal
        Serial.print(sensor.getAngle());
        Serial.print("\t");
        Serial.println(sensor.getVelocity());
        delay(100);
}
