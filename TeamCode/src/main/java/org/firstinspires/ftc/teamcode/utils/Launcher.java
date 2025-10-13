package org.firstinspires.ftc.teamcode.utils;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Launcher {
    // Launcher constants
    final int TARGET_RPM = 1500; // Target RPM for both launcher motors
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
    private State state = State.IDLE;
    private int launches;

    public enum State {
        IDLE,
        SPEED_UP,
        LAUNCH
    }

    public Launcher(HardwareMap hardwareMap) {
        // initialize hardware (drivetrain is initialized by Pedro Pathing)
        leftMotor = hardwareMap.get(DcMotorEx.class, "launcher_left");
        rightMotor = hardwareMap.get(DcMotorEx.class, "launcher_right");
        tapperServo = hardwareMap.get(Servo.class, "tapper");

        // Set launcher motors to brake
        leftMotor.setZeroPowerBehavior(BRAKE);
        rightMotor.setZeroPowerBehavior(BRAKE);
    }

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

    public double getLeftRPM() {
        return leftMotor.getVelocity();
    }

    public double getRightRPM() {
        return rightMotor.getVelocity();
    }

    public double getCommandedTapperRotation() {
        return tapperServo.getPosition();
    }

    public State update() {
        switch(state) {
            case IDLE:
                // If this rums, we are starting a new launch cycle, so we'll move to the speed up state
                state = State.SPEED_UP;
                launches = 0; // Reset launch amount
                inToleranceTimer.reset(); // Reset in tolerance timer

                break;
            case SPEED_UP:
                double currentLeftRPM = leftMotor.getVelocity();
                double currentRightRPM = rightMotor.getVelocity();

                // Check if we are within the tolerance
                if (Math.abs(TARGET_RPM - currentLeftRPM) <= RPM_TOLERANCE && Math.abs(TARGET_RPM - currentRightRPM) <= RPM_TOLERANCE) {
                    // Check if we have been within tolerance for the required amount of time (eliminates inconsistency due to oscillation)
                    if (inToleranceTimer.milliseconds() >= RPM_IN_RANGE_TIME) {
                        // We have reached all prerequisites for launch
                        state = State.LAUNCH;
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