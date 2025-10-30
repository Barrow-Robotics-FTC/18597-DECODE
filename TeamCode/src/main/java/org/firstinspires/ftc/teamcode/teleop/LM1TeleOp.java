package org.firstinspires.ftc.teamcode.teleop;

// FTC SDK
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

// Pedro Pathing
import com.pedropathing.follower.Follower;

// Local helper files
import static org.firstinspires.ftc.teamcode.utils.Constants.TeleOp.*;
import static org.firstinspires.ftc.teamcode.utils.Constants.Alliance;
import static org.firstinspires.ftc.teamcode.utils.Constants.AprilTagConstants.BLUE_GOAL_TAG_ID;
import static org.firstinspires.ftc.teamcode.utils.Constants.AprilTagConstants.RED_GOAL_TAG_ID;
import org.firstinspires.ftc.teamcode.utils.Constants.LauncherConstants.LauncherReturnProps;
import org.firstinspires.ftc.teamcode.utils.Launcher;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.teamcode.utils.AprilTag;
import org.firstinspires.ftc.teamcode.utils.Constants;

/*
Gamepad Map for LM1 TeleOp

Drive Coach: Parley
Human Player: Cedar
Gamepad 1 (Driver): Dylan
    Left Stick X: Robot translation movement
    Left Stick Y: Robot axial movement
    Right Stick X: Robot rotational movement
    Left Bumper: Toggle slow mode
    Light:
        Red: Lining up with April Tag
        Blue: Slow mode
Gamepad 2 (Operator): Jozy
    Right Bumper: Toggle launcher speed up (will hold speed once sped up until you press this again)
    Left Trigger: Launch 1 artifact
    Right Trigger: Launch 3 artifacts
    Light:
        Red: Lining up with April Tag
        Blue: Launcher is active
        Green: Ready for launch
 */

@TeleOp(name = "LM1 TeleOp", group = "TeleOp")
@SuppressWarnings("FieldCanBeLocal") // Suppress pointless Android Studio warnings
public class LM1TeleOp extends LinearOpMode {
    // Values retrieved from blackboard
    private Alliance alliance; // Alliance of the robot

    // Driver controller variables
    private boolean slowMode = false;

    // Utilities
    private final ElapsedTime runtime = new ElapsedTime();
    private Follower follower; // Pedro pathing follower
    private Launcher launcher; // Custom launcher class
    private LauncherReturnProps launcherStatus; // Current launcher state
    private Constants.MovementVectors movementVectors; // Movement vectors for robot drive
    private AprilTag aprilTag; // Custom April Tag class

    // Other variables
    private int artifactsToLaunch = 0; // Number of artifacts to launch
    private boolean liningUpWithGoal = false; // Is the robot currently lining up with the goal?
    private boolean launcherIsActive = false; // Is the launcher currently active (sped up)?
    private boolean launcherIsLaunching = false; // Is the launcher currently launching?
    
    // Gamepad 2 trigger edge detection
    private boolean prevLeftTriggerPressed = false;
    private boolean prevRightTriggerPressed = false;
    private boolean leftTriggerPressed() {
        boolean currState = gamepad2.left_trigger > 0.5 && !prevLeftTriggerPressed; // Current state
        prevLeftTriggerPressed = gamepad2.left_trigger > 0.5; // Update previous state
        return currState;
    }
    private boolean rightTriggerPressed() {
        boolean currState = gamepad2.right_trigger > 0.5 && !prevRightTriggerPressed; // Current state
        prevRightTriggerPressed = gamepad2.right_trigger > 0.5; // Update previous state
        return currState;
    }

    private void startLaunch(int numArtifacts) {
        artifactsToLaunch = numArtifacts; // Indicate that we want to launch the specified number of artifacts
        liningUpWithGoal = true; // Start by lining up with the goal
    }

    // Gamepad light helpers
    @SuppressWarnings("SameParameterValue")
    private void setGamepad1Light(int r, int g, int b) {
        gamepad1.setLedColor(r, g, b, 100);
    }
    private void setGamepad2Light(int r, int g, int b) {
        gamepad2.setLedColor(r, g, b, 100);
    }

    @Override
    public void runOpMode() {
        // Get variables from Blackboard
        alliance = (Alliance) blackboard.getOrDefault("alliance", Alliance.BLUE);

        // Initialize the Pedro Pathing follower and set the start pose to the autonomous ending pose
        follower = Constants.Pedro.createFollower(hardwareMap);
        follower.update();

        // Initialize all utilities used in TeleOp
        movementVectors = new Constants.MovementVectors(0, 0, 0);
        launcher = new Launcher(hardwareMap);
        aprilTag = new AprilTag(hardwareMap);

        // Initialize launcher status (.update() will do nothing since launcher is idle)
        launcherStatus = launcher.update();

        // Log completed initialization
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the TeleOp period to start (driver presses START)
        waitForStart();
        runtime.reset();

        // Start Pedro Pathing with TeleOp (manual) drive
        follower.startTeleopDrive(BRAKE_MODE);

        while (opModeIsActive()) {
            // Update Pedro Pathing every iteration
            follower.update();

            // Gamepad 1 Joysticks): Movement
            movementVectors.forward = -gamepad1.left_stick_y * (slowMode ? SLOW_MODE_MULTIPLIER : NORMAL_SPEED_MULTIPLIER);
            movementVectors.strafe = -gamepad1.left_stick_x * (slowMode ? SLOW_MODE_MULTIPLIER : NORMAL_SPEED_MULTIPLIER);
            movementVectors.turn = -gamepad1.right_stick_x * (slowMode ? SLOW_MODE_MULTIPLIER : NORMAL_SPEED_MULTIPLIER);

            // Gamepad 1 Left Bumper: Toggle slow mode
            if (gamepad1.leftBumperWasPressed()) {
                slowMode = !slowMode;
            }

            // Gamepad 2 Right Bumper: Toggle launcher speed up
            if (gamepad2.rightBumperWasPressed()) {
                if (launcherIsActive) {
                    launcher.stop(); // Stop the launcher
                    launcherIsActive = false; // Set active flag to false
                } else {
                    launcher.speedUp(); // Speed up the launcher
                    launcherIsActive = true; // Set active flag to true
                }
            }

            // Gamepad 2 Left Trigger: Launch 1 artifact
            if (leftTriggerPressed() && !launcherIsLaunching) {
                startLaunch(1); // Indicate that we want to launch 1 artifact
            }

            // Gamepad 2 Right Trigger: Launch 3 artifacts
            if (rightTriggerPressed() && !launcherIsLaunching) {
                startLaunch(3); // Indicate that we want to launch 3 artifacts
            }

            // If we are in the process of launching artifacts
            if (artifactsToLaunch > 0) {

                // Check if we need to line up with the goal first
                if (liningUpWithGoal) {
                    AprilTagDetection goalTag = aprilTag.getTag(alliance == Alliance.RED ? RED_GOAL_TAG_ID : BLUE_GOAL_TAG_ID);
                    if (goalTag != null) { // If the goal tag was detected
                        // Set movement vectors to drive to the April Tag
                        Constants.MovementVectors alignmentVectors = aprilTag.driveToAprilTag(goalTag, Constants.DISTANCE_FROM_APRIL_TAG);
                        if (alignmentVectors.moveCompleted) { // If alignment is complete
                            liningUpWithGoal = false; // No longer lining up with the goal
                        }

                        // Set movement vectors to alignment vectors
                        movementVectors.forward = alignmentVectors.forward;
                        movementVectors.strafe = alignmentVectors.strafe;
                        movementVectors.turn = alignmentVectors.turn;
                    }
                } else {
                    // We are lined up with the goal, proceed to launch
                    // Note: The launch command will call speedUp() if the launcher is idle, so no need to check here
                    launcher.launch(artifactsToLaunch); // Start the launch of artifacts
                    launcherIsLaunching = true; // Indicate that the launcher is launching
                    artifactsToLaunch = 0; // Reset the launch request
                }
            }

            // Update launcher (nothing will happen when launcher is idle)
            launcherStatus = launcher.update();
            if (launcherStatus.cycleCompleted) { // If a launch cycle has completed
                launcherIsLaunching = false; // Indicate that the launcher is no longer launching
            }

            // Set robot movement based on movement vectors
            follower.setTeleOpDrive(movementVectors.forward, movementVectors.strafe, movementVectors.turn, ROBOT_CENTRIC);

            // Set Gamepad 1 lights (priority goes first to last)
            if (liningUpWithGoal) { // Lining up with goal: red
                setGamepad1Light(255, 0, 0);
            } else if (slowMode) { // Slow mode: blue
                setGamepad1Light(0, 0, 255);
            } else { // None of the above conditions: off
                setGamepad1Light(0, 0, 0);
            }
            
            // Set Gamepad 1 lights (priority goes first to last)
            if (liningUpWithGoal) { // Lining up with goal: red
                setGamepad2Light(255, 0, 0);
            } else if (launcherIsLaunching) { // Launcher is launching: green
                setGamepad2Light(0, 255, 0);
            } else if (launcherIsActive) { // Launcher is active: blue
                setGamepad2Light(0, 0, 255);
            } else { // None of the above conditions: off
                setGamepad2Light(0, 0, 0);
            }

            // Log status
            telemetry.addData("Run Time: ", runtime.seconds());
            telemetry.addData("Launcher State: ", launcherStatus.state);
            telemetry.addData("Artifacts to launch", artifactsToLaunch);
            telemetry.addData("Lining up with goal: ", liningUpWithGoal);
            telemetry.update();
        }

        // Stop everything at the end of TeleOp
        follower.setTeleOpDrive(0, 0, 0); // Stop robot movement
        follower.update();
        launcher.stop(); // Stop the launcher
        aprilTag.stopCamera(); // Stop camera streaming
    }
}