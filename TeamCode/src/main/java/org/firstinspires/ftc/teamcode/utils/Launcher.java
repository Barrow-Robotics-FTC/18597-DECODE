package org.firstinspires.ftc.teamcode.utils;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Launcher {
    // Launcher constants
    int TARGET_RPM = 1500; // Target RPM for both launcher motors
    final int RPM_TOLERANCE = 100; // Tolerance of RPM required for launch
    final int RPM_IN_RANGE_TIME = 250; // How long the launcher must be within the target RPM tolerance to launch (milliseconds)
    final int MIN_TIME_BETWEEN_LAUNCHES = 500; // Minimum time between launches (milliseconds)
    final int TAPPER_POSITIONING_TIME = 250; // Time to wait for the tapper to reach the pushed position (milliseconds)
    final double TAPPER_PUSHED_POSITION = 0.5; // How much the tapper servo rotates to push a ball into the shooter
    final double TAPPER_HOME_POSITION = 0.0; // Position of the tapper when retracted

    // Motors and servos
    private final DcMotorEx leftMotor; // Left flywheel motor (looking from the robots perspective)
    private final DcMotorEx rightMotor; // Right flywheel motor (looking from the robots perspective)
    private final Servo tapperServo; // Tapper servo that pushes the ball into the shooter wheels

    // Timers
    private final ElapsedTime inToleranceTimer = new ElapsedTime();
    private final ElapsedTime timeSinceLastLaunch = new ElapsedTime();
    private final ElapsedTime tapperRaisedTimer = new ElapsedTime();

    // Other variables
    private final double motorTicksPerRev; // How many encoder ticks per revolution the motors have
    private State state = State.IDLE; // Current state of the launcher
    private int launches; // How many artifacts have been launched in the current launch cycle
    private boolean tapperCommanded = false; // Whether the tapper has been commanded to the push position
    private boolean tapperPositioned = false; // Whether the tapper is in the pushed position

    // Launcher states
    public enum State {
        IDLE,
        SPEED_UP,
        LAUNCH
    }

    // Constructor
    public Launcher(HardwareMap hardwareMap) {
        // initialize hardware (drivetrain is initialized by Pedro Pathing)
        leftMotor = hardwareMap.get(DcMotorEx.class, "launcher_left");
        rightMotor = hardwareMap.get(DcMotorEx.class, "launcher_right");
        tapperServo = hardwareMap.get(Servo.class, "tapper");

        // Set ticks per revolution variable
        motorTicksPerRev = leftMotor.getMotorType().getTicksPerRev();

        // Set launcher motor characteristics with a list and a for loop to reduce redundant code
        java.util.List<DcMotorEx> motors = java.util.Arrays.asList(leftMotor, rightMotor);
        for (DcMotorEx motor : motors) {
            motor.setZeroPowerBehavior(BRAKE);
            motor.setMode(com.qualcomm.robotcore.hardware.DcMotorEx.RunMode.RUN_USING_ENCODER);
            motor.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(300,0,0,10));
        }
        leftMotor.setDirection(DcMotorEx.Direction.REVERSE);

        // Make sure tapper is in the starting position
        tapperServo.setPosition(0.0);
    }

    // Helper function to stop motors and reset servos
    private void resetMotorsAndServos() {
        leftMotor.setPower(0);
        rightMotor.setPower(0);
        tapperServo.setPosition(0.0);
    }

    // Stop the launcher and return to idle state
    public State stop() {
        // When the state is set to idle, whatever ran this state machine will know that artifacts have been launched
        // With the state being idle, the next time update is called, the launch cycle will start over again.
        state = State.IDLE;

        // Stop the launcher motors and reset tapper
        resetMotorsAndServos();

        // Reset launch count
        launches = 0;

        return state;
    }

    // Unit converters
    private double rpmToTps(double rpm) {
        return rpm * motorTicksPerRev / 60.0;
    }

    private double tpsToRpm(double tps) {
        return tps * 60.0 / motorTicksPerRev;
    }

    // RPM fetchers
    public double getLeftRPM() {
        return tpsToRpm(leftMotor.getVelocity());
    }

    public double getRightRPM() {
        return tpsToRpm(rightMotor.getVelocity());
    }

    // Tapper position fetcher
    public double getCommandedTapperRotation() {
        return tapperServo.getPosition();
    }

    // Getters and setters
    public int getLaunches() {
        return launches;
    }

    public int getTargetRPM() {
        return TARGET_RPM;
    }

    public void setTargetRPM(int rpm) {
        TARGET_RPM = rpm;
    }

    // Update function, runs the launcher state machine
    // If launchIfReady is true, the launcher will launch as soon as it is ready
    // If launchIfReady is false, the launcher will spin up to speed and wait in the SPEED_UP state until launchIfReady is true
    public State update(boolean launchIfReady) {
        switch(state) {
            case IDLE:
                // If this rums, we are starting a new launch cycle, so we'll move to the speed up state
                state = State.SPEED_UP;
                launches = 0; // Reset launch amount
                inToleranceTimer.reset(); // Reset in tolerance timer
                leftMotor.setPower(0); // Start with 0 power, will be adjusted in SPEED_UP state
                rightMotor.setPower(0); // Start with 0 power, will be adjusted in SPEED_UP state
                tapperServo.setPosition(0.0); // Make sure tapper is in the starting position

                break;
            case SPEED_UP:
                // Set motor powers to reach target RPM
                leftMotor.setVelocity(rpmToTps(TARGET_RPM));
                rightMotor.setVelocity(rpmToTps(TARGET_RPM));

                // Create variables to check if each motor is within the RPM tolerance
                boolean leftInTol = Math.abs(TARGET_RPM - getLeftRPM()) <= RPM_TOLERANCE;
                boolean rightInTol = Math.abs(TARGET_RPM - getRightRPM()) <= RPM_TOLERANCE;

                // Check if we are within the tolerance
                if (leftInTol && rightInTol) {
                    // Check if we have been within tolerance for the required amount of time (eliminates inconsistency due to oscillation)
                    if (inToleranceTimer.milliseconds() >= RPM_IN_RANGE_TIME) {
                        // We have reached all prerequisites for launch
                        if (!launchIfReady) {
                            // If we are not supposed to launch yet, stay in this state
                            break;
                        }
                        state = State.LAUNCH; // Move to launch state
                        tapperRaisedTimer.reset(); // Reset tapper raised timer
                    }
                } else {
                    inToleranceTimer.reset();
                }
                break;
            case LAUNCH:
                if (launches != 0) { // If we aren't on the first launch
                    if (timeSinceLastLaunch.milliseconds() < MIN_TIME_BETWEEN_LAUNCHES) {
                        break; // Wait until the minimum time between launches has passed
                    }
                }

                if (!tapperPositioned) { // If the tapper isn't already positioned
                    if (!tapperCommanded) { // If the tapper hasn't already been commanded to the pushed position
                        tapperServo.setPosition(TAPPER_PUSHED_POSITION); // Command the tapper to the pushed position
                        tapperCommanded = true; // Mark that the tapper has been commanded
                        tapperRaisedTimer.reset(); // Reset the timer to measure how long since the tapper was commanded
                    }

                    // Wait for the tapper to be positioned
                    if (tapperRaisedTimer.milliseconds() < TAPPER_POSITIONING_TIME) {
                        break; // Wait until the tapper has had time to reach the pushed position
                    }

                    tapperPositioned = true; // Mark that the tapper is now in the pushed position
                }

                // At this point, the tapper should be in the pushed position
                // This means an artifact should have been launched
                launches += 1; // Increment launch count
                if (launches >= 3) { // If we have launched 3 artifacts
                    stop(); // Stop the launcher
                } else {
                    state = State.SPEED_UP; // Recover motor speed for the next launch
                    inToleranceTimer.reset(); // Reset in tolerance timer
                }

                // Reset variables for next launch (if any)
                tapperServo.setPosition(0.0); // Retract the tapper
                tapperCommanded = false; // Mark that the tapper is no longer commanded
                tapperPositioned = false; // Mark that the tapper is no longer positioned
                timeSinceLastLaunch.reset(); // Reset the time since last launch timer

                break;
        }
        return state;
    }
}