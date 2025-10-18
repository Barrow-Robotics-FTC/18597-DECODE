package org.firstinspires.ftc.teamcode.utils;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class ManualLauncher {
    // Launcher constants
    int TARGET_RPM = 1000; // Target RPM for both launcher motors
    final int RPM_TOLERANCE = 100; // Tolerance of RPM required for launch
    final int RPM_IN_RANGE_TIME = 200; // How long the launcher must be within the target RPM tolerance to launch (milliseconds)
    final int TAPPER_POSITIONING_TIME = 500; // Time to wait for the tapper to reach the pushed position (milliseconds)
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

    // Gamepad
    Gamepad gamepad;

    // Other variables
    private final double motorTicksPerRev; // How many encoder ticks per revolution the motors have
    private State state = State.IDLE; // Current state of the launcher
    private int launches; // How many artifacts have been launched in the current launch cycle
    private boolean tapperCommanded = false; // Whether the tapper has been commanded to the push position
    private boolean tapperPositioned = false; // Whether the tapper is in the pushed position
    private boolean readyForLaunch = false; // Launcher is ready to launch

    // Launcher states
    public enum State {
        IDLE,
        SPEED_UP,
        LAUNCH
    }

    // Constructor
    public ManualLauncher(HardwareMap hardwareMap, Gamepad gamepad) {
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
            motor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            motor.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(300,0,0,10));
        }
        leftMotor.setDirection(DcMotorEx.Direction.REVERSE);

        // Reset motors and servos to default state
        resetMotorsAndServos();

        // Assign gamepad
        this.gamepad = gamepad;
    }

    // Helper function to stop motors and reset servos
    private void resetMotorsAndServos() {
        leftMotor.setPower(0);
        rightMotor.setPower(0);
        tapperServo.setPosition(TAPPER_HOME_POSITION);
    }

    // Stop the launcher and return to idle state
    public State stop() {
        resetMotorsAndServos(); // Stop the launcher motors and reset tapper
        launches = 0; // Reset launch amount
        readyForLaunch = false; // Reset ready for launch state
        tapperCommanded = false; // Reset tapper commanded state
        tapperPositioned = false; // Reset tapper positioned state
        inToleranceTimer.reset(); // Reset in tolerance timer
        timeSinceLastLaunch.reset(); // Reset time since last launch timer
        tapperRaisedTimer.reset(); // Reset tapper raised timer
        state = State.IDLE; // Set state to IDLE

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

    // Check if the launcher is ready for launch
    public boolean isReadyForLaunch() {
        return readyForLaunch;
    }

    // Command launch
    public void commandLaunch() {
        if (state == State.SPEED_UP && readyForLaunch) {
            state = State.LAUNCH; // Command the launcher to launch
            readyForLaunch = false; // Reset ready for launch
        }
    }

    // Update function, runs the launcher state machine with controller inputs
    public State update() {
        if (gamepad.leftBumperWasPressed()) {
            if (state == State.IDLE) {
                state = State.SPEED_UP; // Command the launcher to speed up
            } else {
                stop(); // Stop the launcher
            }
        }

        // If the right bumper is pressed while in SPEED_UP state and the launcher is ready, command a launch
        if (gamepad.rightBumperWasPressed() && state == State.SPEED_UP && readyForLaunch) {
            state = State.LAUNCH; // Command the launcher to launch
            readyForLaunch = false; // Reset ready for launch
        }

        switch(state) {
            case IDLE:
                // Ensure variables are reset before starting a launch cycle
                launches = 0; // Reset launch amount
                readyForLaunch = false; // Reset ready for launch state
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
                        readyForLaunch = true; // Launcher is ready to launch
                        // Now waiting for the user to command a launch via the gamepad
                    }
                } else {
                    readyForLaunch = false; // Launcher is not ready to launch
                    inToleranceTimer.reset(); // Reset in tolerance timer
                }
                break;
            case LAUNCH:
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
                state = State.SPEED_UP; // Recover motor speed for the next launch
                inToleranceTimer.reset(); // Reset in tolerance timer

                // Reset variables for next launch (if any)
                tapperServo.setPosition(TAPPER_HOME_POSITION); // Retract the tapper
                tapperCommanded = false; // Mark that the tapper is no longer commanded
                tapperPositioned = false; // Mark that the tapper is no longer positioned
                timeSinceLastLaunch.reset(); // Reset the time since last launch timer

                // At this point, the speed up cycle will restart and the user will need to command the next launch

                break;
        }
        return state;
    }
}