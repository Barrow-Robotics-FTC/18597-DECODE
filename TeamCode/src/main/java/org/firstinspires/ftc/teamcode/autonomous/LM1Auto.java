package org.firstinspires.ftc.teamcode.autonomous;

// FTC SDK
import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

// Pedro Pathing
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;

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

@Autonomous(name = "LM1 Autonomous", group = "Autonomous", preselectTeleOp="LM1TeleOpNoPedro")
@SuppressWarnings("FieldCanBeLocal") // Suppress pointless Android Studio warnings
public class LM1Auto extends LinearOpMode {
    // Editable variables
    final List<State> stateList = Arrays.asList( // Add autonomous states for the state machine here
            State.DRIVE_TO_GOAL_AREA,
            State.LINE_UP_WITH_GOAL,
            State.LAUNCH,
            State.STRAFE_OFF_LAUNCH_LINE
    );

    // Hardware
    private DcMotor frontLeftDrive = null;
    private DcMotor frontRightDrive = null;
    private DcMotor backLeftDrive = null;
    private DcMotor backRightDrive = null;
    private GoBildaPinpointDriver pinpoint;

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
    private Pose currentPose; // Current pose of the robot
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
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, Constants.Pedro.localizerConstants.hardwareMapName);

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

        // Configure the GoBilda Pinpoint localizer
        //noinspection SuspiciousNameCombination (Apparently Pedro reverses offsets?)
        pinpoint.setOffsets(Constants.Pedro.localizerConstants.forwardPodY, Constants.Pedro.localizerConstants.strafePodX,
                Constants.Pedro.localizerConstants.distanceUnit);
        pinpoint.setEncoderResolution(Constants.Pedro.localizerConstants.encoderResolution);
        pinpoint.setEncoderDirections(Constants.Pedro.localizerConstants.forwardEncoderDirection,
                Constants.Pedro.localizerConstants.strafeEncoderDirection);
        pinpoint.resetPosAndIMU();

        // Initialize all utilities used in auto
        paths = new Constants.Paths();
        launcher = new Launcher(hardwareMap);
        aprilTag = new AprilTag(hardwareMap);
        stateMachine = new StateMachine();

        // Prompt the driver to select an alliance
        alliance = AllianceSelector.run(gamepad1, telemetry);

        // Prompt user to select start position and set starting pose
        startPosition = StartPositionSelector.run(gamepad1, telemetry, (alliance == Constants.Alliance.BLUE));
        pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, startPosition.pose.getX(), startPosition.pose.getY(),
                AngleUnit.DEGREES, startPosition.pose.getHeading()
        ));

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
            // Update Pinpoint
            pinpoint.update();
            currentPose = new Pose(
                    pinpoint.getPosition().getX(DistanceUnit.INCH),
                    pinpoint.getPosition().getY(DistanceUnit.INCH),
                    pinpoint.getPosition().getHeading(AngleUnit.RADIANS)
            ); // Update the current pose

            // If the state machine is complete or time is almost up, hold position to avoid penalties
            if (pathState == State.COMPLETED || runtime.milliseconds() > 28000) {
                movementVectors = new MovementVectors(0, 0, 0); // No movement
            } else {
                // Run the state machine update loop (updates movementVectors)
                pathState = stateMachine.update();
            }

            // Apply movement vectors to motors
            double frontLeftPower = movementVectors.forward + movementVectors.strafe + movementVectors.turn;
            double frontRightPower = movementVectors.forward - movementVectors.strafe - movementVectors.turn;
            double backLeftPower = movementVectors.forward - movementVectors.strafe + movementVectors.turn;
            double backRightPower = movementVectors.forward + movementVectors.strafe - movementVectors.turn;

            // Normalize the values so no wheel power exceeds 100%
            // This ensures that the robot maintains the desired motion.
            double max = Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower));
            max = Math.max(max, Math.abs(backLeftPower));
            max = Math.max(max, Math.abs(backRightPower));

            if (max > 1.0) {
                frontLeftPower /= max;
                frontRightPower /= max;
                backLeftPower /= max;
                backRightPower /= max;
            }

            // Set motor powers
            frontLeftDrive.setPower(frontLeftPower);
            frontRightDrive.setPower(frontRightPower);
            backLeftDrive.setPower(backLeftPower);
            backRightDrive.setPower(backRightPower);

            // Log status
            telemetry.addData("Elapsed", runtime.toString());
            telemetry.addData("Path State", pathState);
            telemetry.addData("Launcher State", launcher.getState());
            telemetry.addData("Path Index", stateMachine.statesIndex);
            telemetry.addData("X", currentPose.getX());
            telemetry.addData("Y", currentPose.getY());
            telemetry.addData("Heading", currentPose.getHeading());
            telemetry.addData("Movement Vectors", movementVectors);
            telemetry.update();
        }

        // OpMode is ending, stop all mechanisms
        stateMachine.stop(); // Stop the state machine
        follower.breakFollowing(); // Stop the position holding

        // Save values for TeleOp
        blackboard.put("alliance", alliance);
        blackboard.put("autoEndPose", currentPose);
        blackboard.put("paths", paths);
    }

    public enum State {
        LAUNCH,
        DRIVE_TO_GOAL_AREA,
        LINE_UP_WITH_GOAL,
        STRAFE_OFF_LAUNCH_LINE,
        COMPLETED
    }

    class StateMachine {
        private int statesIndex; // Current index in states
        private State currentState; // Current state (only used in update for cleaner code)
        private boolean launchCommanded; // Has the launch been commanded by the LAUNCH state
        private boolean launcherActive; // Is the launcher currently active
        private boolean strafingOffLaunchLine; // Is the robot currently strafing off the launch line
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
                case DRIVE_TO_GOAL_AREA:
                    boolean inTolX = Math.abs(72 - currentPose.getX()) < 3; // Check if we are within 3 inches of target X (72)
                    boolean inTolY = Math.abs(72 - currentPose.getY()) < 3; // Check if we are within 3 inches of target Y (72)
                    if (inTolX && inTolY) {
                        movementVectors = new MovementVectors(0, 0, 0); // No movement
                        nextState();
                        break;
                    }

                    if (!inTolX) { // X is out of tolerance
                        //double moveSpeed = 72 - currentPose.getX() > 0 ? -0.25 : 0.25; // Determine forward/backward speed
                        movementVectors = new MovementVectors(0.25, 0, 0); // Move forward
                    } else { // Y is out of tolerance
                        //double moveSpeed = 72 - currentPose.getY() > 0 ? -0.25 : 0.25; // Determine left/right speed
                        movementVectors = new MovementVectors(0, 0.25, 0); // Strafe right
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
                        movementVectors = new MovementVectors(0, 0, -0.25); // Rotate left until the tag is found
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
                        movementVectors = new MovementVectors(0, 0.5, 0); // Strafe right
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