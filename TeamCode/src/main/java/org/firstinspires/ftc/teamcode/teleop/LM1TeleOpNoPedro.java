package org.firstinspires.ftc.teamcode.teleop;

// FTC SDK
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

// Pedro Pathing
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;

// Local helper files
import static org.firstinspires.ftc.teamcode.utils.Constants.TeleOp.*;
import org.firstinspires.ftc.teamcode.utils.Constants;
import org.firstinspires.ftc.teamcode.utils.Constants.LauncherConstants.LauncherReturnProps;
import org.firstinspires.ftc.teamcode.utils.Constants.LauncherConstants.LauncherState;
import org.firstinspires.ftc.teamcode.utils.Launcher;

/*
Gamepad Map for LM1 TeleOp

Drive Coach: NAME
Human Player: NAME
Gamepad 1 (Driver): NAME
    Left Stick X: Robot translation movement
    Left Stick Y: Robot axial movement
    Right Stick X: Robot rotational movement
    DPad Right: Toggle slow mode
    Right Bumper: Toggle launcher speed up (will hold speed once sped up until you press this again)
    Left Trigger: Launch 1 artifact
    Right Trigger: Launch 3 artifacts
 */

@TeleOp(name = "LM1 TeleOp (No Pedro)", group = "TeleOp")
@SuppressWarnings("FieldCanBeLocal") // Suppress pointless Android Studio warnings
public class LM1TeleOpNoPedro extends LinearOpMode {
    // Values retrieved from blackboard
    private Pose autoEndPose; // End pose of the autonomous, start pose of TeleOp

    // Driver controller variables
    private boolean slowMode = false;

    // Launcher commanded state machine variables (runs when a launch is requested)
    private LaunchCommandedState launchCommandedState = LaunchCommandedState.DETECT_GOAL_APRIL_TAG;
    private boolean launchCommanded = false;

    // Other variables
    private final ElapsedTime runtime = new ElapsedTime();
    private final Constants.MovementVectors movementVectors = new Constants.MovementVectors(0, 0, 0); // Movement vectors for robot drive
    private Follower follower; // Pedro pathing follower
    private Pose currentPose; // Current pose of the robot
    private LauncherReturnProps launcherStatus; // Current launcher state
    private Launcher launcher; // Custom launcher class
    private int artifactsToLaunch = 0; // Number of artifacts to launch
    private boolean launcherIsActive = false; // Is the launcher currently active (sped up)?
    private boolean launcherIsLaunching = false; // Is the launcher currently launching?

    // Gamepad trigger states
    private boolean prevLeftTriggerPressed = false;
    private boolean prevRightTriggerPressed = false;

    private boolean leftTriggerPressed() {
        boolean currState = gamepad1.left_trigger > 0.5 && !prevLeftTriggerPressed; // Current state
        prevLeftTriggerPressed = gamepad1.left_trigger > 0.5; // Update previous state
        return currState;
    }

    private boolean rightTriggerPressed() {
        boolean currState = gamepad1.right_trigger > 0.5 && !prevRightTriggerPressed; // Current state
        prevRightTriggerPressed = gamepad1.right_trigger > 0.5; // Update previous state
        return currState;
    }

    @Override
    public void runOpMode() {
        // Get variables from Blackboard
        autoEndPose = (Pose) blackboard.getOrDefault("autoEndPose", new Pose(72, 8, Math.toRadians(90)));

        // Initialize the Pedro Pathing follower and set the start pose to the autonomous ending pose
        follower = Constants.Pedro.createFollower(hardwareMap);
        follower.setStartingPose(autoEndPose);
        follower.update();

        // Initialize all utilities used in TeleOp
        launcher = new Launcher(hardwareMap);

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
            currentPose = follower.getPose();

            // Set default movement vectors (controller joysticks)
            movementVectors.forward = -gamepad1.left_stick_y * (slowMode ? SLOW_MODE_MULTIPLIER : NORMAL_SPEED_MULTIPLIER);
            movementVectors.strafe = -gamepad1.left_stick_x * (slowMode ? SLOW_MODE_MULTIPLIER : NORMAL_SPEED_MULTIPLIER);
            movementVectors.turn = -gamepad1.right_stick_x * (slowMode ? SLOW_MODE_MULTIPLIER : NORMAL_SPEED_MULTIPLIER);

            // Gamepad 1 DPad Right: toggle slow mode
            if (gamepad1.dpadRightWasPressed()) {
                slowMode = !slowMode;
            }

            // Gamepad 1 Right Bumper: Toggle launcher speed up
            if (gamepad1.rightBumperWasPressed() && launcherStatus.state == LauncherState.IDLE) {
                if (launcherIsActive) {
                    launcher.stop(); // Stop the launcher
                    launcherIsActive = false; // Set active flag to false
                } else {
                    launcher.speedUp(); // Speed up the launcher
                    launcherIsActive = true; // Set active flag to true
                }
            }

            // Gamepad 1 Left Trigger: Launch 1 artifact
            if (leftTriggerPressed() && !launcherIsLaunching) {
                artifactsToLaunch = 1; // Indicate that we want to launch 1 artifact
            }

            // Gamepad 1 Right Trigger: Launch 3 artifacts
            if (rightTriggerPressed() && !launcherIsLaunching) {
                artifactsToLaunch = 3; // Indicate that we want to launch 3 artifacts
            }

            // Start launch if requested (we have already confirmed that the launcher isn't already launching)
            if (artifactsToLaunch > 0) {
                // Note: The launch command will call speedUp() if the launcher is idle, so no need to check here
                launcher.launch(artifactsToLaunch); // Start the launch of artifacts
                launcherIsLaunching = true; // Indicate that the launcher is launching
                artifactsToLaunch = 0; // Reset the launch request
            }

            // Update launcher (nothing will happen when launcher is idle)
            launcherStatus = launcher.update();
            if (launcherStatus.cycleCompleted) { // If a launch cycle has completed
                launcherIsLaunching = false; // Indicate that the launcher is no longer launching
            }

            // Set robot movement based on movement vectors
            follower.setTeleOpDrive(movementVectors.forward, movementVectors.strafe, movementVectors.turn, ROBOT_CENTRIC);

            // Log status
            telemetry.addData("Run Time: ", runtime.seconds());
            telemetry.addData("Launcher State: ", launcherStatus.state);
            telemetry.addData("", "");
            telemetry.addData("X: ", currentPose.getX());
            telemetry.addData("Y: ", currentPose.getY());
            telemetry.addData("Heading: ", currentPose.getHeading());
            telemetry.update();
        }
    }

    private enum LaunchCommandedState {
        DETECT_GOAL_APRIL_TAG,
        LINE_UP_WITH_GOAL,
        LAUNCH_ARTIFACTS,
    }
}