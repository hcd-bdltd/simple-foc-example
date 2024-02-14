/**
 * B-G431B-ESC1 position motion control example with hall sensor
 *
 */

#include <SimpleFOC.h>

// FOC classes (motor, driver, current sensor and hall sensor)
BLDCMotor motor = BLDCMotor(7);
BLDCDriver6PWM driver = BLDCDriver6PWM(A_PHASE_UH, A_PHASE_UL, A_PHASE_VH, A_PHASE_VL, A_PHASE_WH, A_PHASE_WL);
LowsideCurrentSense current_sensor = LowsideCurrentSense(0.003f, -64.0f/7.0f, A_OP1_OUT, A_OP2_OUT, A_OP3_OUT);
HallSensor hall_sensor = HallSensor(A_HALL1, A_HALL2, A_HALL3, 7);

// Interrupt routine initialisation
void doA(){ hall_sensor.handleA(); }
void doB(){ hall_sensor.handleB(); }
void doC(){ hall_sensor.handleC(); }

// angle set point variable
float target = 0;
// instantiate the commander
Commander commander = Commander(Serial);
void doMotor(char* cmd) { commander.motor(&motor, cmd); }
void doRadians(char* cmd) { commander.scalar(&target, cmd); }
void doDegrees(char* cmd) {
        float angle_degrees;
        commander.scalar(&angle_degrees, cmd);
        target = angle_degrees * (PI / 180.0);
}

void checkRC(int rc) {
        if (rc == 0) {
                for(;;) {
                        SIMPLEFOC_DEBUG("Setup failed.");
                        SIMPLEFOC_DEBUG("Check motor and sensor connections.");
                        _delay(1000);
                }
        }
}

void setup() {
        int rc = 0;
        Serial.begin(115200);
        SimpleFOCDebug::enable(&Serial);

        // initialize the hall sensor
        hall_sensor.init();
        hall_sensor.enableInterrupts(doA, doB, doC);
        // link the hall sensor to the motor
        motor.linkSensor(&hall_sensor);

        // initialize the driver
        driver.voltage_power_supply = 24.0f;
        rc = driver.init();
        checkRC(rc); SIMPLEFOC_DEBUG("Driver ready.");
        // link the current sensor and the driver
        current_sensor.linkDriver(&driver);
        // link the motor and the driver
        motor.linkDriver(&driver);

        // initialize the current sensor
        current_sensor.skip_align = true;
        rc = current_sensor.init();
        checkRC(rc); SIMPLEFOC_DEBUG("Current sensor ready.");
        // link the current sensor to the motor
        motor.linkCurrentSense(&current_sensor);

        // set motion control loop to be used
        motor.controller = MotionControlType::angle;
        motor.foc_modulation = FOCModulationType::SpaceVectorPWM;

        // index search velocity [rad/s]
        motor.velocity_index_search = 3.0f;
        // aligning voltage [V]
        motor.voltage_sensor_align = 3.0f;

        // velocity PI controller
        motor.PID_velocity.P = 0.2f;
        motor.PID_velocity.I = 2.0f;
        motor.PID_velocity.D = 0.0f;
        motor.PID_velocity.output_ramp = 1000.0f;
        motor.LPF_velocity.Tf = 0.05f;

        // angle P controller
        motor.P_angle.P = 20.0f;

        // motor limits
        motor.voltage_limit = 12.0f;
        motor.velocity_limit = 4.0f;
        motor.current_limit = 2.0f;

        // motion and monitoring settings
        motor.motion_downsample = 0;
        motor.monitor_downsample = 10000;

        // use monitoring with serial
        motor.monitor_separator = '\t';
        motor.monitor_decimals = 2;
        motor.monitor_variables = 0b1111111;
        motor.useMonitoring(Serial);

        // initialize motor
        motor.init();
        // align sensor and start FOC
        rc = motor.initFOC();
        checkRC(rc); SIMPLEFOC_DEBUG("Motor ready.");

        // initialize serial communication
        commander.verbose = VerboseMode::user_friendly;
        commander.add('m',doMotor,"motor");
        commander.add('r', doRadians, "angle [rad]");
        commander.add('d', doDegrees, "angle [deg]");

        SIMPLEFOC_DEBUG("Setup ready.");
        SIMPLEFOC_DEBUG("Set the target angle using serial terminal:");
        _delay(1000);
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
        motor.move(target);

        // function intended to be used with serial plotter to monitor motor variables
        // significantly slowing the execution down!!!!
        motor.monitor();

        // user communication
        commander.run();
}
