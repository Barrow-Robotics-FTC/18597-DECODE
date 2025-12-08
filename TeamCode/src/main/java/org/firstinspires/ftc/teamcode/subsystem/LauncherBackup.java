package org.firstinspires.ftc.teamcode.subsystem;

// FTC SDK
import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

// Local helpers
import static org.firstinspires.ftc.teamcode.Constants.LauncherConstants.*;
import static org.firstinspires.ftc.teamcode.Constants.LauncherConstants.leftLauncherCoefficients;
import static org.firstinspires.ftc.teamcode.Constants.LauncherConstants.rightLauncherCoefficients;
import org.firstinspires.ftc.teamcode.Robot;

public class LauncherBackup {
    // Timers
    private final ElapsedTime inToleranceTimer = new ElapsedTime();
    private final ElapsedTime timeSinceLastLaunch = new ElapsedTime();

    // Other variables
    private LauncherState state = LauncherState.IDLE; // Current state of the launcher
    private int launchesToPerform; // How many launches to perform in this launch cycle
    private int launches; // How many artifacts have been launched in the current launch cycle
    private boolean holdSpeed = false; // Should the launcher return to the speed up state after any action
    private boolean launchWhenReady = false; // Has the launcher been commanded to launch
    private boolean launchCycleCompleted = false; // Was a launch cycle completed in the last update

    // Constructor
    public LauncherBackup(Robot robot) {
        // Launcher motor configuration
        robot.leftLauncherMotor.setZeroPowerBehavior(BRAKE);
        robot.leftLauncherMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        robot.leftLauncherMotor.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(leftLauncherCoefficients.P, leftLauncherCoefficients.I, leftLauncherCoefficients.D, leftLauncherCoefficients.F));
        robot.rightLauncherMotor.setDirection(REVERSE); // Reverse right motor
        robot.rightLauncherMotor.setZeroPowerBehavior(BRAKE);
        robot.rightLauncherMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        robot.rightLauncherMotor.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(rightLauncherCoefficients.P, rightLauncherCoefficients.I, rightLauncherCoefficients.D, rightLauncherCoefficients.F));
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
        robot.leftLauncherMotor.setVelocity(getTargetRPM());
        robot.rightLauncherMotor.setVelocity(getTargetRPM());
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
        return launchWhenReady;
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
     * @param coefficients The new PIDF coefficients
     */
    public void updateLeftControllerCoefficients(Robot robot, PIDFCoefficients coefficients) {
        robot.leftLauncherMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, coefficients);
    }

    /**
     * Update the PIDF controller coefficients for the right motor
     *
     * @param coefficients The new PIDF coefficients
     */
    public void updateRightControllerCoefficients(Robot robot, PIDFCoefficients coefficients) {
        robot.rightLauncherMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, coefficients);
    }

    /**
     * Get the PDFT coefficients for the left motor
     *
     * @return FilteredPIDF coefficients
     */
    public PIDFCoefficients getLeftControllerCoefficients(Robot robot) {
        return robot.leftLauncherMotor.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    /**
     * Get the PDFT coefficients for the right motor
     *
     * @return FilteredPIDF coefficients
     */
    public PIDFCoefficients getRightControllerCoefficients(Robot robot) {
        return robot.leftLauncherMotor.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    /**
     * Command the launcher to speed up to target RPM
     */
    public void speedUp(boolean holdSpeed) {
        state = LauncherState.SPEED_UP; // Move to speed up state
        this.holdSpeed = holdSpeed; // Set hold speed flag
        inToleranceTimer.reset(); // Ensure that the in tolerance timer is reset, probably redundant
    }

    /**
     * Command the launcher to launch a set number of artifacts
     * The launcher will speed up if it is not already at target RPM
     *
     * @param artifacts The number of artifacts to launch
     */
    public void launch(int artifacts) {
        if (state == LauncherState.IDLE) {
            speedUp(false); // If the launcher hasn't sped up, start the speed up now, don't hold speed after
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
            setPowers(POWER_WHILE_INTAKING, POWER_WHILE_INTAKING, robot);
            return;
        }

        switch(state) {
            case IDLE:
                setPowers(0, 0, robot); // Stop the launcher motors
                inToleranceTimer.reset(); // Reset in tolerance timer

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
                // This condition only returns true when the tapper is actually pushed
                if (!robot.tapper.isInPushedPosition()) {
                    robot.tapper.push(); // Command the tapper to push
                    break; // Wait for the tapper to reach the pushed position
                }

                // An artifact has been launched at this point
                launches += 1; // Increment launch count
                if (launches >= launchesToPerform) { // If we have launched the target amount of artifacts
                    resetCycleVariables(); // Reset cycle variables
                    launchCycleCompleted = true; // Mark that a launch cycle was completed
                    if (!holdSpeed) {
                        stop(); // Stop the launcher
                    } else {
                        speedUp(true); // Keep the launcher at speed
                    }
                } else {
                    speedUp(holdSpeed); // Recover motor speed for the next launch
                    inToleranceTimer.reset(); // Reset in tolerance timer
                }

                // Reset variables for next launch
                robot.tapper.retract(); // Retract the tapper
                timeSinceLastLaunch.reset(); // Reset the time since last launch timer

                break;
        }
    }
}