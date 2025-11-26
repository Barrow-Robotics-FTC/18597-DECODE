package org.firstinspires.ftc.teamcode.subsystem;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import static org.firstinspires.ftc.teamcode.Constants.LauncherConstants.*;
import org.firstinspires.ftc.teamcode.Robot;

public class LauncherBackup {
    public static PIDFCoefficients leftLauncherCoefficients = new PIDFCoefficients(85, 0, 80, 13.2);
    public static PIDFCoefficients rightLauncherCoefficients = new PIDFCoefficients(85, 0, 80, 13.2);

    // Timers
    private final ElapsedTime inToleranceTimer = new ElapsedTime();
    private final ElapsedTime timeSinceLastLaunch = new ElapsedTime();

    // Other variables
    private LauncherState state = LauncherState.IDLE; // Current state of the launcher
    private int launchesToPerform; // How many launches to perform in this launch cycle
    private int launches; // How many artifacts have been launched in the current launch cycle
    private boolean launchWhenReady = false; // Has the launcher been commanded to launch
    private boolean launchCycleCompleted = false; // Was a launch cycle completed in the last update
    private boolean prevIntaking = false; // Was the intake running in the previous update
    private boolean returnToIdleAfterLaunch; // Should the launcher return to idle after launching

    // Constructor
    public LauncherBackup(Robot robot) {
        // Configure motors
        robot.leftLauncherMotor.setZeroPowerBehavior(BRAKE);
        robot.leftLauncherMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        robot.leftLauncherMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, leftLauncherCoefficients);
        robot.rightLauncherMotor.setDirection(REVERSE); // Reverse right motor
        robot.rightLauncherMotor.setZeroPowerBehavior(BRAKE);
        robot.rightLauncherMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        robot.rightLauncherMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, rightLauncherCoefficients);
    }

    // Called after each cycle to reset variables
    private void resetCycleVariables() {
        launchesToPerform = 0; // Reset launches to perform
        launches = 0; // Reset launch amount
        launchWhenReady = false; // Reset launch when ready
    }

    private void setPowers(double leftPower, double rightPower, Robot robot) {
        robot.leftLauncherMotor.setPower(leftPower);
        robot.rightLauncherMotor.setPower(rightPower);
    }

    private void updateControllers(Robot robot) {
        robot.leftLauncherMotor.setVelocity(TARGET_RPM);
        robot.rightLauncherMotor.setVelocity(TARGET_RPM);
    }

    /**
     * Get the current state of the launcher
     *
     * @return The current LauncherState
     */
    public LauncherState getState() {
        return state;
    }

    /**
     * Check if a launch cycle was completed in the last update
     *
     * @return True if a launch cycle was completed, false otherwise
     */
    public boolean didCompleteCycle() {
        return launchCycleCompleted;
    }

    /**
     * Check if the launcher is active (speeding up or launching)
     *
     * @return True if the launcher is active, false otherwise
     */
    public boolean isActive() {
        return state == LauncherState.SPEED_UP || state == LauncherState.LAUNCH;
    }

    /**
     * Check if the launcher is currently launching artifacts
     *
     * @return True if the launcher is launching, false otherwise
     */
    public boolean isLaunching() {
        return state == LauncherState.LAUNCH;
    }

    /**
     * Get the left launcher motor RPM
     *
     * @param robot The Robot object
     * @return The left launcher motor RPM
     */
    public double getLeftRPM(Robot robot) {
        return robot.leftLauncherMotor.getVelocity();
    }

    /**
     * Get the right launcher motor RPM
     *
     * @param robot The Robot object
     * @return The right launcher motor RPM
     */
    public double getRightRPM(Robot robot) {
        return robot.rightLauncherMotor.getVelocity();
    }

    /**
     * Get the target RPM of the launcher motors
     *
     * @return The target RPM
     */
    public int getTargetRPM() {
        return TARGET_RPM;
    }

    /**
     * Update the PIDF controller coefficients for the left motor
     *
     * @param robot The Robot object
     * @param coefficients The new PIDF coefficients
     */
    public void updateLeftControllerCoefficients(Robot robot, PIDFCoefficients coefficients) {
        robot.leftLauncherMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, coefficients);
    }

    /**
     * Update the PIDF controller coefficients for the right motor
     *
     * @param robot The Robot object
     * @param coefficients The new PIDF coefficients
     */
    public void updateRightControllerCoefficients(Robot robot, PIDFCoefficients coefficients) {
        robot.rightLauncherMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, coefficients);
    }

    /**
     * Get the PDFT coefficients for the left motor
     *
     * @param robot The Robot object
     * @return PIDF coefficients
     */
    public PIDFCoefficients getLeftControllerCoefficients(Robot robot) {
        return robot.leftLauncherMotor.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    /**
     * Get the PDFT coefficients for the right motor
     *
     * @param robot The Robot object
     * @return FilteredPIDF coefficients
     */
    public PIDFCoefficients getRightControllerCoefficients(Robot robot) {
        return robot.rightLauncherMotor.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    /**
     * Command the launcher to speed up to target RPM
     */
    public void speedUp() {
        state = LauncherState.SPEED_UP;
        inToleranceTimer.reset(); // Reset in tolerance timer when entering speed up state
    }

    /**
     * Command the launcher to launch a set number of artifacts
     * The launcher will speed up if it is not already at target RPM
     *
     * @param artifacts The number of artifacts to launch
     */
    public void launch(int artifacts) {
        returnToIdleAfterLaunch = state == LauncherState.IDLE; // Determine if we should return to idle after launching
        if (state == LauncherState.IDLE) {
            speedUp(); // If the launcher hasn't sped up, start the speed up now
        }

        launchesToPerform = artifacts; // Set the amount of launches to perform
        launchWhenReady = true; // Command the launcher to launch when ready
    }

    public void stop() {
        state = LauncherState.IDLE; // Set the launcher to an idle state
    }

    public void update(Robot robot) {
        launchCycleCompleted = false; // Reset launch cycle completed flag

        // Check if intake is running
        if (robot.intake.isActive()) {
            // Run the launcher wheels in reverse to avoid jamming
            // Make sure we don't suddenly switch directions and cause damage or back EMF
            if (getLeftRPM(robot) > DIRECTION_SWITCH_SAFETY_RPM || getRightRPM(robot) > DIRECTION_SWITCH_SAFETY_RPM) {
                // Help the launcher wheels coast down below the safety threshold
                setPowers(COAST_DOWN_POWER, COAST_DOWN_POWER, robot);
            } else {
                // Run the launcher wheels at full intake power
                setPowers(POWER_WHILE_INTAKING, POWER_WHILE_INTAKING, robot);
            }
            prevIntaking = true; // Mark that the intake was running this update
            return;
        } else if (prevIntaking) { // Intaking last update but not this update
            // No longer intaking, move back to speed up state if the launcher wasn't previously idle
            setPowers(0, 0, robot);
            if (state != LauncherState.IDLE) { // If the launcher wasn't idle
                speedUp(); // Return to speed up state
            }
            prevIntaking = false; // Mark that the intake is no longer running
            return;
        }

        switch(state) {
            case IDLE:
                setPowers(0, 0, robot); // Stop the launcher motors

                break;
            case SPEED_UP:
                // Update the motor speeds using the controllers
                updateControllers(robot);

                // Create variables to check if each motor is within the RPM tolerance
                boolean leftInTol = Math.abs(getTargetRPM() - getLeftRPM(robot)) <= RPM_TOLERANCE;
                boolean rightInTol = Math.abs(getTargetRPM() - getRightRPM(robot)) <= RPM_TOLERANCE;

                // Check if the motors are within tolerance
                if (leftInTol && rightInTol) {
                    // Check if they have been within tolerance for long enough
                    if (inToleranceTimer.milliseconds() >= RPM_IN_RANGE_TIME) {
                        // At this point, the launcher is ready to launch
                        if (!launchWhenReady) { // But if a launch hasn't been commanded
                            break; // Stay in speed up state
                        }
                        state = LauncherState.LAUNCH; // Move to launch state
                    }
                } else {
                    inToleranceTimer.reset(); // If the motors aren't in tolerance, reset the timer
                }

                break;
            case LAUNCH:
                // Continue updating the motor speeds to maintain RPM
                updateControllers(robot);

                // Command and wait for the tapper to push an artifact into the launcher
                if (!robot.tapper.isPushed()) {
                    robot.tapper.push(); // Command the tapper to push
                    break; // Wait for the tapper to reach the pushed position
                }

                // An artifact has been launched at this point
                launches += 1; // Increment launch count
                if (launches >= launchesToPerform) { // If we have launched the target amount of artifacts
                    resetCycleVariables(); // Reset cycle variables
                    launchCycleCompleted = true; // Mark that a launch cycle was completed

                    if (returnToIdleAfterLaunch) {
                        stop(); // Stop the launcher if we were previously idle
                    } else {
                        speedUp(); // Otherwise, remain at speed for the next launch cycle
                    }
                } else {
                    speedUp(); // Recover motor speed for the next launch
                }

                // Reset variables for next launch
                robot.tapper.retract(); // Retract the tapper
                timeSinceLastLaunch.reset(); // Reset the time since last launch timer

                break;
        }
    }
}