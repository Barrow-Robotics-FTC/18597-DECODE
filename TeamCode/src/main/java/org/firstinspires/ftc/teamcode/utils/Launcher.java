package org.firstinspires.ftc.teamcode.utils;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class Launcher {
    // Launcher constants
    int TARGET_RPM = 1500; // Target RPM for both launcher motors
    final int RPM_TOLERANCE = 100; // Tolerance of RPM required for launch
    final int RPM_IN_RANGE_TIME = 250; // How long the launcher must be within the target RPM tolerance to launch (milliseconds)
    final int ARTIFACT_LAUNCHED_RPM_TOLERANCE = TARGET_RPM - 100; // Launcher motor RPM must be below this for an artifact to be considered launched
    final double TAPPER_ROTATION_AMOUNT = 0.5; // How much the tapper servo rotates to push a ball into the shooter

    // Motors and servos
    private final DcMotorEx leftMotor; // Left flywheel motor (looking from the robots perspective)
    private final DcMotorEx rightMotor; // Right flywheel motor (looking from the robots perspective)
    private final Servo tapperServo; // Tapper servo that pushes the ball into the shooter wheels

    // Other variables
    private final ElapsedTime inToleranceTimer = new ElapsedTime();
    private double motorTicksPerRev; // How many encoder ticks per revolution the motors have
    private State state = State.IDLE;
    private int launches;

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

    // Stop the launcher and return to idle state
    public State stop() {
        // When the state is set to idle, whatever ran this state machine will know that artifacts have been launched
        // With the state being idle, the next time update is called, the launch cycle will start over again.
        state = State.IDLE;

        // Stop the shooter motors
        leftMotor.setPower(0);
        rightMotor.setPower(0);

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

    private int getLaunchedRpmThreshold() {
        // Dynamic threshold tied to the current target
        return TARGET_RPM - 100;
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
                    }
                } else {
                    inToleranceTimer.reset();
                }
                break;
            case LAUNCH:
                // Push the ball into the shooter wheels
                tapperServo.setPosition(TAPPER_ROTATION_AMOUNT);

                // Detect shooter flywheel RPM drop to know when ball shoots
                if (leftMotor.getVelocity() <= ARTIFACT_LAUNCHED_RPM_TOLERANCE) {
                    // Put tapper down
                    tapperServo.setPosition(0.0);

                    // Check if we've launched 3 artifacts
                    launches += 1;
                    if (launches >= 3) {
                        // Stop the launcher after all 3 artifacts have been launched
                        stop();
                    } else {
                        // Recover from launch
                        state = State.SPEED_UP;
                        inToleranceTimer.reset(); // Reset in tolerance timer
                    }
                }
                break;
        }
        return state;
    }
}