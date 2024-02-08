/**
 *
 * Torque control example using voltage control loop.
 *
 * Most of the low-end BLDC driver boards doesn't have current measurement therefore SimpleFOC offers
 * you a way to control motor torque by setting the voltage to the motor instead of the current.
 *
 * This makes the BLDC motor effectively a DC motor, and you can use it in a same way.
 */
#include <SimpleFOC.h>

#define POLE_PAIRS 7

// BLDC motor & driver instance
BLDCMotor motor = BLDCMotor(POLE_PAIRS);
BLDCDriver6PWM driver = BLDCDriver6PWM(A_PHASE_UH, A_PHASE_UL, A_PHASE_VH, A_PHASE_VL, A_PHASE_WH, A_PHASE_WL);

// hall sensor instance
HallSensor sensor = HallSensor(A_HALL1, A_HALL2, A_HALL3, POLE_PAIRS);

// Interrupt routine initialisation
// channel A and B callbacks
void doA(){sensor.handleA();}
void doB(){sensor.handleB();}
void doC(){sensor.handleC();}

// voltage set point variable
float target_voltage = 2;
// instantiate the commander
Commander command = Commander(Serial);
void doTarget(char* cmd) { command.scalar(&target_voltage, cmd); }

void setup() {
        Serial.begin(115200);
        delay(2000);

        // initialize hall sensor hardware
        sensor.init();
        sensor.enableInterrupts(doA, doB, doC);
        // link the motor to the sensor
        motor.linkSensor(&sensor);

        // driver config
        // power supply voltage [V]
        driver.voltage_power_supply = 12;
        driver.init();
        // link driver
        motor.linkDriver(&driver);


        // aligning voltage
        motor.voltage_sensor_align = 3;

        // choose FOC modulation (optional)
        motor.foc_modulation = FOCModulationType::SpaceVectorPWM;

        // set motion control loop to be used
        motor.controller = MotionControlType::torque;

        // use monitoring with serial
        motor.useMonitoring(Serial);

        // initialize motor
        motor.init();
        // align sensor and start FOC
        motor.initFOC();

        // add target command T
        command.add('T', doTarget, "target voltage");

        Serial.println(F("Motor ready."));
        Serial.println(F("Set the target voltage using serial terminal:"));
        delay(1000);
}

void loop() {
        // main FOC algorithm function
        // the faster you run this function the better
        // Arduino UNO loop  ~1kHz
        // Bluepill loop ~10kHz
        motor.loopFOC();

        // Motion control function
        // velocity, position or voltage (defined in motor.controller)
        // this function can be run at much lower frequency than loopFOC() function
        // You can also use motor.move() and set the motor.target in the code
        motor.move(target_voltage);

        // user communication
        command.run();
}
