package org.firstinspires.ftc.teamcode.tests;

// FTC SDK
import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Servo;

// Panels
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.bylazar.telemetry.PanelsTelemetry;

@TeleOp(name = "Launcher Test", group = "Test")
@Configurable // Panels
@SuppressWarnings("FieldCanBeLocal") // Stop Android Studio from bugging about variables being predefined
public class LauncherTest extends LinearOpMode {
    // Editable variables
    final int TARGET_LAUNCHER_RPM = 1500; // Target RPM for both launcher motors
    final int LAUNCHER_RPM_TOLERANCE = 100; // Tolerance of RPM required for launch
    final int LAUNCHER_RPM_IN_RANGE_TIME = 250; // How long the launcher must be within the target RPM tolerance to launch (milliseconds)
    final double TAPPER_ROTATION_AMOUNT = 0.5; // How much the tapper servo rotates to push a ball into the shooter

    // Initialize elapsed timer
    private final ElapsedTime runtime = new ElapsedTime();

    // Motors and servos
    private DcMotorEx leftLauncherMotor; // Left flywheel motor (looking from the robots perspective)
    private DcMotorEx rightLauncherMotor; // Right flywheel motor (looking from the robots perspective)
    private Servo tapperServo; // Tapper servo that pushes the ball into the shooter wheels

    // Other variables
    private LauncherStateMachine launcherStateMachine; // Custom launcher state machine
    private TelemetryManager panelsTelemetry; // Panels telemetry
    private LauncherStateMachine.State launcherState;
    private boolean launcherStateMachineActive = false;

    @Override
    public void runOpMode() {
        // Initialize Panels telemetry
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        // initialize hardware (drivetrain is initialized by Pedro Pathing)
        leftLauncherMotor = hardwareMap.get(DcMotorEx.class, "launcher_left");
        rightLauncherMotor = hardwareMap.get(DcMotorEx.class, "launcher_right");
        tapperServo = hardwareMap.get(Servo.class, "tapper");

        // Set launcher motors to brake
        leftLauncherMotor.setZeroPowerBehavior(BRAKE);
        rightLauncherMotor.setZeroPowerBehavior(BRAKE);

        // Create launcher state machine and initialize
        launcherStateMachine = new LauncherStateMachine();
        launcherStateMachine.init(leftLauncherMotor, rightLauncherMotor, tapperServo,
                TARGET_LAUNCHER_RPM, LAUNCHER_RPM_TOLERANCE, LAUNCHER_RPM_IN_RANGE_TIME, TAPPER_ROTATION_AMOUNT);

        // Log completed initialization to Panels and driver station
        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry); // Update Panels and driver station after logging

        // Wait for the game to start (driver presses START)
        waitForStart();

        // Reset runtime timer
        runtime.reset();

        while (opModeIsActive()) {
            // If the launcher state machine is active, run the update loop
            if (launcherStateMachineActive) {
                launcherState = launcherStateMachine.update();

                // If the launcher state machine stops running, set running to false
                if (launcherState == LauncherStateMachine.State.IDLE) {
                    launcherStateMachineActive = false;
                }

                // If the user presses X, stop the state machine
                if (gamepad1.xWasPressed()) {
                    launcherStateMachineActive = false;
                    launcherStateMachine.state = LauncherStateMachine.State.IDLE; // Set to idle to stop the state machine
                    leftLauncherMotor.setPower(0); // Stop left launch motor
                    rightLauncherMotor.setPower(0); // Stop right launch motor
                    tapperServo.setPosition(0); // Reset tapper
                }
            } else {
                // If the user presses X, start the state machine
                if (gamepad1.xWasPressed()) {
                    launcherStateMachineActive = true;
                }
            }

            // Update Panels and driver station after logging
            panelsTelemetry.debug("Elapsed", runtime.toString());
            panelsTelemetry.debug("Status", launcherState);
            panelsTelemetry.update(telemetry);
        }
    }

    // State machine for a horizontal double flywheel launcher with 6k RPM GoBilda Yellow Jackets
    static class LauncherStateMachine {
        private final ElapsedTime inToleranceTimer = new ElapsedTime();
        private DcMotorEx leftMotor;
        private DcMotorEx rightMotor;
        private Servo tapperServo;
        private State state;
        private int targetRPM;
        private int RPMTolerance;
        private int requiredInToleranceTime;
        private double tapperRotationAmount;

        private double currentLeftRPM;
        private double currentRightRPM;
        private int launches;

        public enum State {
            IDLE,
            SPEED_UP,
            LAUNCH
        }

        public void init(DcMotorEx left_motor, DcMotorEx right_motor, Servo tapper_servo,
                         int target_rpm, int rpm_tolerance, int required_in_tolerance_time, double tapper_rotation_amount) {
            leftMotor = left_motor;
            rightMotor = right_motor;
            tapperServo = tapper_servo;
            targetRPM = target_rpm;
            RPMTolerance = rpm_tolerance;
            requiredInToleranceTime = required_in_tolerance_time;
            tapperRotationAmount = tapper_rotation_amount;
            state = State.IDLE;
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
                    currentLeftRPM = leftMotor.getVelocity();
                    currentRightRPM = rightMotor.getVelocity();

                    // Check if we are within the tolerance
                    if (Math.abs(currentLeftRPM - targetRPM) <= RPMTolerance && Math.abs(currentRightRPM - targetRPM) <= RPMTolerance) {
                        // Check if we have been within tolerance for the required amount of time (eliminates inconsistency due to oscillation)
                        if (inToleranceTimer.milliseconds() >= requiredInToleranceTime) {
                            // We have reached all prerequisites for launch
                            state = State.LAUNCH;
                        }
                    } else {
                        inToleranceTimer.reset();
                    }
                    break;
                case LAUNCH:
                    // Push the ball into the shooter wheels
                    tapperServo.setPosition(tapperRotationAmount);

                    // Detect shooter flywheel RPM drop to know when ball shoots
                    if (leftMotor.getVelocity() <= targetRPM - 100) {
                        // Put tapper down
                        tapperServo.setPosition(0.0);

                        // Check if we've launched 3 artifacts
                        launches += 1;
                        if (launches >= 3) {
                            // When the state is set to idle, the main state machine will catch this and continue on.
                            // With the state being idle, the next time update is called, this cycle will start over again.
                            state = State.IDLE;

                            // Stop the shooter
                            leftMotor.setPower(0);
                            rightMotor.setPower(0);
                        } else {
                            // Recover from launch
                            state = State.SPEED_UP;
                        }
                    }
                    break;
            }
            return state;
        }
    }
}