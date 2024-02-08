// Open loop motor control example
#include <SimpleFOC.h>

#define POLE_PAIRS 7

// BLDC motor & driver instance
BLDCMotor motor = BLDCMotor(POLE_PAIRS);
BLDCDriver6PWM driver = BLDCDriver6PWM(A_PHASE_UH, A_PHASE_UL, A_PHASE_VH, A_PHASE_VL, A_PHASE_WH, A_PHASE_WL);

//target variable
float target_velocity = 0;

// instantiate the commander
Commander command = Commander(Serial);
void doTarget(char* cmd) { command.scalar(&target_velocity, cmd); }
void doLimit(char* cmd) { command.scalar(&motor.voltage_limit, cmd); }

void setup() {
        Serial.begin(115200);
        delay(2000);

        // driver config
        // power supply voltage [V]
        driver.voltage_power_supply = 12;
        // limit the maximal dc voltage the driver can set
        // as a protection measure for the low-resistance motors
        // this value is fixed on startup
        driver.voltage_limit = 6;
        driver.init();
        // link the motor and the driver
        motor.linkDriver(&driver);

        // limiting motor movements
        // limit the voltage to be set to the motor
        // start very low for high resistance motors
        // current = voltage / resistance, so try to be well under 1Amp
        motor.voltage_limit = 3;   // [V]

        // open loop control config
        motor.controller = MotionControlType::velocity_openloop;

        // init motor hardware
        motor.init();
        // use monitoring with serial
        motor.useMonitoring(Serial);

        // add target command T
        command.add('T', doTarget, "target velocity");
        command.add('L', doLimit, "voltage limit");

        Serial.println("Motor ready!");
        Serial.println("Set target velocity [rad/s]");
}

void loop() {
        // open loop velocity movement
        // using motor.voltage_limit and motor.velocity_limit
        // to turn the motor "backwards", just set a negative target_velocity
        motor.move(target_velocity);

        // user communication
        command.run();
}
