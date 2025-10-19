package org.firstinspires.ftc.teamcode.utils;

// FTC SDK
import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
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
    private final double motorTicksPerRev; // How many encoder ticks per revolution the motors have
    private LauncherState state = LauncherState.IDLE; // Current state of the launcher
    private int launches; // How many artifacts have been launched in the current launch cycle
    private boolean tapperCommanded = false; // Whether the tapper has been commanded to the push position
    private boolean tapperPositioned = false; // Whether the tapper is in the pushed position

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

        // Reverse left launcher motor
        leftMotor.setDirection(DcMotorEx.Direction.REVERSE);

        // Reset motors and servos to default state
        resetMotorsAndServos();
    }

    // Helper function to stop motors and reset servos
    private void resetMotorsAndServos() {
        leftMotor.setPower(0);
        rightMotor.setPower(0);
        tapperServo.setPosition(TAPPER_HOME_POSITION);
    }

    // Stop the launcher and return to idle state
    public LauncherState stop() {
        // When the state is set to idle, whatever ran this state machine will know that artifacts have been launched
        // With the state being idle, the next time update is called, the launch cycle will start over again.
        state = LauncherState.IDLE;

        // Stop the launcher motors and reset tapper
        resetMotorsAndServos();

        // Reset launch count
        launches = 0;

        return state;
    }

    // Convert RPM to TPS
    private double rpmToTps(double rpm) {
        return rpm * motorTicksPerRev / 60.0;
    }

    // Convert TPS to RPM
    private double tpsToRpm(double tps) {
        return tps * 60.0 / motorTicksPerRev;
    }

    // Get the left launcher motor RPM
    public double getLeftRPM() {
        return tpsToRpm(leftMotor.getVelocity());
    }

    // Get the right launcher motor RPM
    public double getRightRPM() {
        return tpsToRpm(rightMotor.getVelocity());
    }

    // Get current commanded tapper position
    public double getCommandedTapperPosition() {
        return tapperServo.getPosition();
    }

    // Get the amount of launches in this cycle
    public int getLaunches() {
        return launches;
    }

    // Get the target RPM of the launcher motors
    public int getTargetRPM() {
        return TARGET_RPM;
    }

    // Set the target RPM of the launcher motors
    public void setTargetRPM(int rpm) {
        TARGET_RPM = rpm;
    }

    // Set the amount of launches to perform in a cycle, default is 3
    public void setAmountOfLaunches(int launches) {
        AMOUNT_OF_LAUNCHES = launches;
    }

    // Update function, runs the launcher state machine
    // If launchIfReady is true, the launcher will launch as soon as it is ready
    // If launchIfReady is false, the launcher will spin up to speed and wait in the SPEED_UP state until launchIfReady is true
    public LauncherState update(boolean launchIfReady) {
        switch(state) {
            case IDLE:
                // If this rums, we are starting a new launch cycle, so we'll move to the speed up state
                state = LauncherState.SPEED_UP;
                launches = 0; // Reset launch amount
                inToleranceTimer.reset(); // Reset in tolerance timer
                resetMotorsAndServos(); // Ensure motors and servos are reset
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
                if (launches >= AMOUNT_OF_LAUNCHES) { // If we have launched the target amount of artifacts
                    stop(); // Stop the launcher
                } else {
                    state = LauncherState.SPEED_UP; // Recover motor speed for the next launch
                    inToleranceTimer.reset(); // Reset in tolerance timer
                }

                // Reset variables for next launch (if any)
                tapperServo.setPosition(TAPPER_HOME_POSITION); // Retract the tapper
                tapperCommanded = false; // Mark that the tapper is no longer commanded
                tapperPositioned = false; // Mark that the tapper is no longer positioned
                timeSinceLastLaunch.reset(); // Reset the time since last launch timer

                break;
        }
        return state;
    }
}