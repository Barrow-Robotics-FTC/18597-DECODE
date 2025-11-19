package org.firstinspires.ftc.teamcode.subsystem;

// FTC SDK
import com.pedropathing.control.FilteredPIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;
 
// Pedro Pathing
import com.pedropathing.control.FilteredPIDFController;

// Local helpers
import static org.firstinspires.ftc.teamcode.Constants.LauncherConstants.*;
import org.firstinspires.ftc.teamcode.Robot;

public class Launcher {
    // PIDFT controllers
    private final FilteredPIDFController leftController;
    private final FilteredPIDFController rightController;

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

    // Constructor
    public Launcher(Robot robot) {
        // Initialize PIDF controllers
        leftController = new FilteredPIDFController(leftLauncherCoefficients);
        rightController = new FilteredPIDFController(rightLauncherCoefficients);
        leftController.setTargetPosition(TARGET_RPM);
        rightController.setTargetPosition(TARGET_RPM);
        leftController.updateFeedForwardInput(TARGET_RPM);
        rightController.updateFeedForwardInput(TARGET_RPM);
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
        // Update controller inputs
        leftController.updatePosition(getLeftRPM(robot));
        rightController.updatePosition(getRightRPM(robot));

        // Set motor powers with updated controller outputs
        setPowers(leftController.run(), rightController.run(), robot);
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
     * Update the PIDFT controller coefficients for the left motor
     *
     * @param coefficients The new PIDFT coefficients
     */
    public void updateLeftControllerCoefficients(FilteredPIDFCoefficients coefficients) {
        leftController.setCoefficients(coefficients);
    }

    /**
     * Update the PIDFT controller coefficients for the right motor
     *
     * @param coefficients The new PIDFT coefficients
     */
    public void updateRightControllerCoefficients(FilteredPIDFCoefficients coefficients) {
        rightController.setCoefficients(coefficients);
    }

    /**
     * Command the launcher to speed up to target RPM
     */
    public void speedUp() {
        state = LauncherState.SPEED_UP; // Move to speed up state
        inToleranceTimer.reset(); // Reset in tolerance timer
    }

    /**
     * Command the launcher to launch a set number of artifacts
     * The launcher will speed up if it is not already at target RPM
     *
     * @param artifacts The number of artifacts to launch
     */
    public void launch(int artifacts) {
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
            if (getLeftRPM(robot) > POSITIVE_TO_NEGATIVE_SAFETY_RPM || getRightRPM(robot) > POSITIVE_TO_NEGATIVE_SAFETY_RPM) {
                // Help the launcher wheels coast down below the safety threshold
                setPowers(COAST_DOWN_POWER, COAST_DOWN_POWER, robot);
            } else {
                // Run the launcher wheels at full intake power
                setPowers(-LAUNCHER_POWER_WHILE_INTAKING, -LAUNCHER_POWER_WHILE_INTAKING, robot);
            }
            prevIntaking = true; // Mark that the intake was running
            return;
        } else if (prevIntaking) { // Intaking last update but not this update
            // No longer intaking, move back to speed up state if the launcher wasn't previously idle
            if (state != LauncherState.IDLE) {
                state = LauncherState.SPEED_UP; // Move to speed up state
                inToleranceTimer.reset(); // Reset in tolerance timer
            }
            prevIntaking = false; // Mark that the intake is no longer running
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

                // Check if we need to wait before launching again
                if (timeSinceLastLaunch.milliseconds() < MIN_TIME_BETWEEN_LAUNCHES) {
                    break; // Wait until the minimum time between launches has passed
                }

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
                    stop(); // Stop the launcher
                } else {
                    state = LauncherState.SPEED_UP; // Recover motor speed for the next launch
                    inToleranceTimer.reset(); // Reset in tolerance timer
                }

                // Reset variables for next launch
                robot.tapper.retract(); // Retract the tapper
                timeSinceLastLaunch.reset(); // Reset the time since last launch timer

                break;
        }
    }
}