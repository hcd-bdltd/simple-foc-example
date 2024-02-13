/**
 * B-G431B-ESC1 position motion control example with hall sensor
 *
 */

#include <SimpleFOC.h>

#define _MON_ALL        (0b1111111)
#define POLE_PAIRS      (7)
#define MONITOR_WAIT_MS (50)

uint32_t last_monitor_ms = 0;

// BLDC motor & driver instance
BLDCMotor motor = BLDCMotor(POLE_PAIRS);
BLDCDriver6PWM driver = BLDCDriver6PWM(A_PHASE_UH, A_PHASE_UL, A_PHASE_VH, A_PHASE_VL, A_PHASE_WH, A_PHASE_WL);
LowsideCurrentSense currentSense = LowsideCurrentSense(0.003f, -64.0f/7.0f, A_OP1_OUT, A_OP2_OUT, A_OP3_OUT);

// hall sensor instance
HallSensor sensor = HallSensor(A_HALL1, A_HALL2, A_HALL3, POLE_PAIRS);

// Interrupt routine initialisation
void doA(){sensor.handleA();}
void doB(){sensor.handleB();}
void doC(){sensor.handleC();}

// angle set point variable
float target_angle = 0;
// instantiate the commander
Commander command = Commander(Serial);
void doTarget(char* cmd) {
        float angle_degrees;
        command.scalar(&angle_degrees, cmd);
        target_angle = angle_degrees * (PI / 180.0);
}

void setup() {
        Serial.begin(115200);

        // initialize hall sensor hardware
        sensor.init();
        sensor.enableInterrupts(doA, doB, doC);
        // link the motor to the sensor
        motor.linkSensor(&sensor);

        // driver config
        // power supply voltage [V]
        driver.voltage_power_supply = 24;
        driver.init();
        // link the motor and the driver
        motor.linkDriver(&driver);
        // link current sense and the driver
        currentSense.linkDriver(&driver);

        // current sensing
        currentSense.init();
        // no need for aligning
        currentSense.skip_align = true;
        motor.linkCurrentSense(&currentSense);

        // aligning voltage [V]
        motor.voltage_sensor_align = 3;
        // index search velocity [rad/s]
        motor.velocity_index_search = 3;

        // set motion control loop to be used
        motor.controller = MotionControlType::angle;

        // velocity PI controller parameters
        motor.PID_velocity.P = 0.2f;
        motor.PID_velocity.I = 2;
        motor.PID_velocity.D = 0;
        // default voltage_power_supply
        motor.voltage_limit = 12;
        // jerk control using voltage voltage ramp
        // default value is 300 volts per sec  ~ 0.3V per millisecond
        motor.PID_velocity.output_ramp = 1000;

        // velocity low pass filtering time constant
        motor.LPF_velocity.Tf = 0.01f;

        // angle P controller
        motor.P_angle.P = 20;
        // maximal velocity of the position control
        motor.velocity_limit = 4;

        // use monitoring with serial
        motor.monitor_downsample = 1;
        motor.monitor_separator = '\t';
        motor.monitor_decimals = 2;
        motor.monitor_variables = _MON_ALL;
        motor.useMonitoring(Serial);

        // initialize motor
        motor.init();
        // align sensor and start FOC
        motor.initFOC();

        // add target command T
        command.add('T', doTarget, "target angle [deg]");

        Serial.println(F("Motor ready."));
        Serial.println(F("Set the target angle using serial terminal:"));

        last_monitor_ms = millis();
}

void loop() {
        // main FOC algorithm function
        // the faster you run this function the better
        // Arduino UNO loop ~1kHz (16 Mhz)
        // Bluepill loop ~10kHz (72Mhz)
        motor.loopFOC();

        // Motion control function
        // velocity, position or voltage (defined in motor.controller)
        // this function can be run at much lower frequency than loopFOC() function
        // You can also use motor.move() and set the motor.target in the code
        motor.move(target_angle);

        if (millis() - last_monitor_ms > MONITOR_WAIT_MS) {
                last_monitor_ms = millis();
                // function intended to be used with serial plotter to monitor motor variables
                // significantly slowing the execution down!!!!
                motor.monitor();
        }

        // user communication
        command.run();
}
