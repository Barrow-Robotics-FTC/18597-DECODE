package org.firstinspires.ftc.teamcode.utils;

// FTC SDK
import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

// Launcher constants
import static org.firstinspires.ftc.teamcode.utils.Constants.LauncherConstants.*;

public class Launcher {
    // Motors and servos
    private final DcMotorEx leftMotor; // Left flywheel motor (looking from the robots perspective)
    private final DcMotorEx rightMotor; // Right flywheel motor (looking from the robots perspective)
    private final Servo tapperServo; // Tapper servo that pushes the ball into the shooter wheels

    // Timers
    private final ElapsedTime inToleranceTimer = new ElapsedTime();
    private final ElapsedTime timeSinceLastLaunch = new ElapsedTime();
    private final ElapsedTime tapperRaisedTimer = new ElapsedTime();

    // Other variables
    private LauncherState state = LauncherState.IDLE; // Current state of the launcher
    private int launchesToPerform; // How many launches to perform in this launch cycle
    private int launches; // How many artifacts have been launched in the current launch cycle
    private boolean launchWhenReady = false; // Has the launcher been commanded to launch
    private boolean tapperCommanded = false; // Has the tapper been commanded to the push position
    private boolean tapperPositioned = false; // Has the tapper reached the pushed position
    private boolean returnToIdleAfterLaunch = true; // Whether to return to idle state after launching

    // Constructor
    public Launcher(HardwareMap hardwareMap) {
        // initialize hardware
        leftMotor = hardwareMap.get(DcMotorEx.class, "launcher_left");
        rightMotor = hardwareMap.get(DcMotorEx.class, "launcher_right");
        tapperServo = hardwareMap.get(Servo.class, "tapper");

        // Set motor directions and characteristics
        rightMotor.setDirection(REVERSE); // Reverse right motor
        rightMotor.setZeroPowerBehavior(BRAKE);
        rightMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        rightMotor.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, LAUNCHER_PIDF_COEFFICIENTS);
        leftMotor.setZeroPowerBehavior(BRAKE);
        leftMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        leftMotor.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, LAUNCHER_PIDF_COEFFICIENTS);

        // Reset motors and servos to default state
        resetMotorsAndServos();
    }

    // Helper function to stop motors and reset servos
    private void resetMotorsAndServos() {
        leftMotor.setVelocity(0);
        rightMotor.setVelocity(0);
        tapperServo.setPosition(TAPPER_HOME_POSITION);
    }

    // Called after each cycle to reset variables
    private void resetCycleVariables() {
        launchesToPerform = 0; // Reset launches to perform
        launches = 0; // Reset launch amount
        launchWhenReady = false; // Reset launch when ready
    }

    // Get the left launcher motor RPM
    public double getLeftRPM() {
        return leftMotor.getVelocity();
    }

    // Get the right launcher motor RPM
    public double getRightRPM() {
        return rightMotor.getVelocity();
    }

    // Get current commanded tapper position
    public double getCommandedTapperPosition() {
        return tapperServo.getPosition();
    }

    // Get the amount of launches performed in this cycle
    public int getLaunches() {
        return launches;
    }

    // Get the target RPM of the launcher motors
    public int getTargetRPM() {
        return TARGET_RPM;
    }

    // Command the launcher to speed up to target RPM
    public void speedUp() {
        state = LauncherState.SPEED_UP; // Move to speed up state
        inToleranceTimer.reset(); // Reset in tolerance timer
    }

    // Command the launcher to launch a certain amount of artifacts
    public void launch(int artifacts) {
        if (state == LauncherState.IDLE) {
            speedUp(); // If the launcher hasn't sped up, start the speed up now
            returnToIdleAfterLaunch = true; // After launching, return to idle
        } else if (state == LauncherState.SPEED_UP) {
            returnToIdleAfterLaunch = false; // If we are already speeding up, don't return to idle after launch
        }

        launchesToPerform = artifacts; // Set the amount of launches to perform
        launchWhenReady = true; // Command the launcher to launch when ready
    }

    // Stop the launcher and return to idle state
    public LauncherState stop() {
        state = LauncherState.IDLE; // Set the launcher to an idle state
        resetMotorsAndServos(); // Stop the launcher motors and reset tapper
        resetCycleVariables(); // Reset cycle variables

        return state;
    }

    // Update function, runs the launcher state machine
    // Returns true if a launch cycle was completed in the frame
    public LauncherReturnProps update() {
        // Return flag for update
        boolean launchCycleCompleted = false; // Reset return flag
        switch(state) {
            case IDLE:
                // Do nothing while idle
                break;
            case SPEED_UP:
                // Set motor velocity to reach target RPM
                leftMotor.setVelocity(getTargetRPM());
                rightMotor.setVelocity(getTargetRPM());

                // Create variables to check if each motor is within the RPM tolerance
                boolean leftInTol = Math.abs(getTargetRPM() - getLeftRPM()) <= RPM_TOLERANCE;
                boolean rightInTol = Math.abs(getTargetRPM() - getRightRPM()) <= RPM_TOLERANCE;

                // Check if we are within the tolerance
                if (leftInTol && rightInTol) {
                    // Check if we have been within tolerance for the required amount of time (eliminates inconsistency due to oscillation)
                    if (inToleranceTimer.milliseconds() >= RPM_IN_RANGE_TIME) {
                        // We have reached all prerequisites for launch
                        if (!launchWhenReady) {
                            // If we are not supposed to launch yet, stay in this state
                            break;
                        }
                        state = LauncherState.LAUNCH; // Move to launch state
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
                if (launches >= launchesToPerform) { // If we have launched the target amount of artifacts
                    resetCycleVariables(); // Reset cycle variables
                    launchCycleCompleted = true; // Mark that a launch cycle was completed
                    if (!returnToIdleAfterLaunch) { // If the launcher shouldn't return to idle after the launch cycle
                        speedUp(); // Return to speed up state
                    } else { // If the launcher should return to idle after the launch cycle
                        stop(); // Stop the launcher
                    }
                } else {
                    state = LauncherState.SPEED_UP; // Recover motor speed for the next launch
                    inToleranceTimer.reset(); // Reset in tolerance timer
                }

                // Reset variables for next launch (if any)
                tapperServo.setPosition(TAPPER_HOME_POSITION); // Retract the tapper
                tapperCommanded = false; // Reset tapper commanded
                tapperPositioned = false; // Reset tapper positioned
                timeSinceLastLaunch.reset(); // Reset the time since last launch timer

                break;
        }
        return new LauncherReturnProps(state, launchCycleCompleted);
    }
}