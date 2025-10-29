package org.firstinspires.ftc.teamcode.autonomous;

// FTC SDK
import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

// Pedro Pathing
import com.pedropathing.follower.Follower;

// Local helper files
import org.firstinspires.ftc.teamcode.utils.Launcher;
import org.firstinspires.ftc.teamcode.utils.AllianceSelector;
import org.firstinspires.ftc.teamcode.utils.StartPositionSelector;
import org.firstinspires.ftc.teamcode.utils.AprilTag;
import org.firstinspires.ftc.teamcode.utils.Constants;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import static org.firstinspires.ftc.teamcode.utils.Constants.MovementVectors;

// Java
import java.util.Arrays;
import java.util.List;

@Autonomous(name = "LM1 Autonomous April Tag", group = "Autonomous", preselectTeleOp="LM1TeleOp")
@SuppressWarnings("FieldCanBeLocal") // Suppress pointless Android Studio warnings
public class LM1AutoAprilTag extends LinearOpMode {
    // Editable variables
    final List<State> stateList = Arrays.asList( // Add autonomous states for the state machine here
            State.DRIVE_FORWARD,
            State.LINE_UP_WITH_GOAL,
            State.LAUNCH,
            State.STRAFE_OFF_LAUNCH_LINE
    );

    // Hardware
    private DcMotor frontLeftDrive = null;
    private DcMotor frontRightDrive = null;
    private DcMotor backLeftDrive = null;
    private DcMotor backRightDrive = null;

    // Utilities
    private final ElapsedTime runtime = new ElapsedTime(); // Runtime elapsed timer
    private Follower follower; // Pedro Pathing follower
    private StateMachine stateMachine; // Custom autonomous state machine
    private Constants.Paths paths; // Custom paths class
    private Launcher launcher; // Custom launcher class
    private AprilTag aprilTag; // Custom April Tag class

    // Other variables
    private MovementVectors movementVectors; // Movement vectors for robot drive
    private Constants.Alliance alliance; // Alliance of the robot
    private Constants.StartPositionConstants.StartSelection startPosition; // Start position of the robot
    private Constants.Pattern targetPattern; // Target pattern determined by obelisk April Tag
    private State pathState; // Current state machine value

    @Override
    public void runOpMode() {
        // Initialize Pedro Pathing follower (ONLY USED FOR PATH GENERATION)
        follower = Constants.Pedro.createFollower(hardwareMap);

        // Initialize hardware
        frontLeftDrive = hardwareMap.get(DcMotor.class, Constants.Pedro.driveConstants.leftFrontMotorName);
        frontRightDrive = hardwareMap.get(DcMotor.class, Constants.Pedro.driveConstants.rightFrontMotorName);
        backLeftDrive = hardwareMap.get(DcMotor.class, Constants.Pedro.driveConstants.leftRearMotorName);
        backRightDrive = hardwareMap.get(DcMotor.class, Constants.Pedro.driveConstants.rightRearMotorName);

        // Set motor directions
        frontLeftDrive.setDirection(Constants.Pedro.driveConstants.leftFrontMotorDirection);
        frontRightDrive.setDirection(Constants.Pedro.driveConstants.rightFrontMotorDirection);
        backLeftDrive.setDirection(Constants.Pedro.driveConstants.leftRearMotorDirection);
        backRightDrive.setDirection(Constants.Pedro.driveConstants.rightRearMotorDirection);

        // Set motors to brake mode
        frontLeftDrive.setZeroPowerBehavior(BRAKE);
        frontRightDrive.setZeroPowerBehavior(BRAKE);
        backLeftDrive.setZeroPowerBehavior(BRAKE);
        backRightDrive.setZeroPowerBehavior(BRAKE);

        // Initialize all utilities used in auto
        paths = new Constants.Paths();
        launcher = new Launcher(hardwareMap);
        aprilTag = new AprilTag(hardwareMap);
        stateMachine = new StateMachine();

        // Prompt the driver to select an alliance
        alliance = AllianceSelector.run(gamepad1, telemetry);

        // Prompt user to select start position and set starting pose
        startPosition = StartPositionSelector.run(gamepad1, telemetry, (alliance == Constants.Alliance.BLUE));

        // Log completed initialization
        telemetry.addData("Status", "Initialized");
        telemetry.addData("Alliance", alliance);
        telemetry.addData("Start Position", startPosition.startPosition);
        telemetry.addData("Start Pose", startPosition.pose);
        telemetry.update();

        // Wait for the game to start (driver presses START)
        waitForStart();

        // Reset runtime timer
        runtime.reset();

        // Speed up launcher
        stateMachine.speedUpLauncher();

        /*
        The April tag obelisk is randomized after the OpMode is initialized, so right after we run the OpMode
        we'll need to immediately scan the April Tag and then initialize our poses and paths.
        */
        targetPattern = aprilTag.detectPattern();

        // Build paths based on detected pattern
        paths.build(follower, targetPattern, alliance, startPosition.pose);

        while (opModeIsActive()) {
            // If the state machine is complete or time is almost up, hold position to avoid penalties
            if (pathState == State.COMPLETED || runtime.milliseconds() > 28000) {
                movementVectors = new MovementVectors(0, 0, 0); // No movement
            } else {
                // Run the state machine update loop (updates movementVectors)
                pathState = stateMachine.update();
            }

            // Set motor powers
            Constants.WheelPowers powers = movementVectors.getWheelPowers();
            frontLeftDrive.setPower(powers.frontLeft);
            frontRightDrive.setPower(powers.frontRight);
            backLeftDrive.setPower(powers.backLeft);
            backRightDrive.setPower(powers.backRight);

            // Log status
            telemetry.addData("Elapsed", runtime.toString());
            telemetry.addData("Path State", pathState);
            telemetry.addData("Launcher State", launcher.getState());
            telemetry.addData("Path Index", stateMachine.statesIndex);
            telemetry.addData("Movement Vectors", movementVectors);
            telemetry.update();
        }

        // OpMode is ending, stop all mechanisms
        stateMachine.stop(); // Stop the state machine
        follower.breakFollowing(); // Stop the position holding

        // Save values for TeleOp
        blackboard.put("alliance", alliance);
        blackboard.put("autoEndPose", paths.poses.home); // Not actually used in the TeleOp, so for LM1 we can just set it random
        blackboard.put("paths", paths);
    }

    public enum State {
        LAUNCH,
        DRIVE_FORWARD,
        LINE_UP_WITH_GOAL,
        STRAFE_OFF_LAUNCH_LINE,
        COMPLETED
    }

    class StateMachine {
        private int statesIndex; // Current index in states
        private State currentState; // Current state (only used in update for cleaner code)
        private boolean launchCommanded; // Has the launch been commanded by the LAUNCH state
        private boolean launcherActive; // Is the launcher currently active
        private boolean movingForward; // Is the robot currently driving forward from the start position
        private boolean strafingOffLaunchLine; // Is the robot currently strafing off the launch line
        private final ElapsedTime moveForwardTimer = new ElapsedTime(); // Timer for driving off the start line
        private final ElapsedTime strafeOffLaunchLineTimer = new ElapsedTime(); // Timer for the strafe off launch line state

        private void nextState() {
            statesIndex += 1;
        }

        public StateMachine() {
            this.statesIndex = 0;
        }

        public void speedUpLauncher() {
            launcher.speedUp();
            this.launcherActive = true; // Launcher is now active
        }

        public void stop() {
            if (currentState != State.COMPLETED) { // If not already completed
                statesIndex = stateList.size(); // Set index to end (COMPLETED state)
                update(); // Run update to stop mechanisms
            }
        }

        public State update() {
            // Handle out of bounds index
            if (statesIndex >= stateList.size()) {
                currentState = State.COMPLETED;
            } else {
                currentState = stateList.get(statesIndex);
            }

            // State machine switch
            switch (currentState) {
                case LAUNCH:
                    if (!this.launchCommanded) { // Command the launcher to launch 3 artifacts
                        this.launchCommanded = true; // Launch has been commanded
                        launcher.launch(3); // Command launcher
                    }
                    if (launcher.update().cycleCompleted) {
                        this.launchCommanded = false; // Reset launch commanded flag
                        nextState();
                    }

                    movementVectors = new MovementVectors(0, 0, 0); // No movement while launching
                    break;
                case DRIVE_FORWARD:
                    if (!movingForward) { // If we haven't started driving forward yet
                        moveForwardTimer.reset(); // Reset the strafe timer
                        movingForward = true; // Set moving forward flag
                    }

                    // Move forward for 1 second then stop and move on
                    if (moveForwardTimer.milliseconds() > 1000) { // Move forward for 1 second
                        movementVectors = new MovementVectors(0, 0, 0); // No movement
                        strafingOffLaunchLine = false; // Reset strafing flag
                        nextState();
                    } else {
                        movementVectors = new MovementVectors(0.4, 0, 0.05); // Move forward (slight heading correction)
                    }
                    break;
                case LINE_UP_WITH_GOAL:
                    // Attempt to detect the goal April tag
                    AprilTagDetection goalTag = aprilTag.getTag(alliance == Constants.Alliance.BLUE ? Constants.AprilTagConstants.BLUE_GOAL_TAG_ID : Constants.AprilTagConstants.RED_GOAL_TAG_ID);
                    if (goalTag != null) { // If the goal tag was detected
                        // Set movement vectors to drive to the April Tag
                        Constants.MovementVectors alignmentVectors = aprilTag.driveToAprilTag(goalTag, Constants.DISTANCE_FROM_APRIL_TAG);
                        if (alignmentVectors.moveCompleted) { // If alignment is complete
                            movementVectors = new MovementVectors(0, 0, 0); // No movement
                            nextState();
                        } else {
                            movementVectors = alignmentVectors; // Apply alignment movement vectors
                        }
                    } else {
                        // Rotate until the tag is found
                        movementVectors = new MovementVectors(0, 0, alliance == Constants.Alliance.BLUE ? -0.3 : 0.3);
                    }
                    break;
                case STRAFE_OFF_LAUNCH_LINE:
                    if (!strafingOffLaunchLine) { // If we haven't started strafing yet
                        strafeOffLaunchLineTimer.reset(); // Reset the strafe timer
                        strafingOffLaunchLine = true; // Set strafing flag
                    }

                    // Strafe right for 1 second then stop and move on
                    if (strafeOffLaunchLineTimer.milliseconds() > 1000) { // Strafe for 1 second
                        movementVectors = new MovementVectors(0, 0, 0); // No movement
                        strafingOffLaunchLine = false; // Reset strafing flag
                        nextState();
                    } else {
                        movementVectors = new MovementVectors(0, 0.4, 0); // Strafe right
                    }
                    break;
                case COMPLETED:
                    // If the launcher is currently active, stop it
                    if (this.launcherActive) {
                        launcher.stop();
                        this.launcherActive = false; // Launcher is no longer active
                    }

                    movementVectors = new MovementVectors(0, 0, 0); // No movement
                    break;
            }
            return currentState;
        }
    }
}