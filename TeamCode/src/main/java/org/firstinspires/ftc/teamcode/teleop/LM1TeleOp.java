package org.firstinspires.ftc.teamcode.teleop;

// FTC SDK
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

// Pedro Pathing
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;

// Local helper files
import org.firstinspires.ftc.teamcode.utils.Launcher;
import org.firstinspires.ftc.teamcode.utils.Constants;
import static org.firstinspires.ftc.teamcode.utils.Constants.TeleOp.*;
import org.firstinspires.ftc.teamcode.utils.Constants.LauncherConstants.LauncherState;
import org.firstinspires.ftc.teamcode.utils.Constants.LauncherConstants.LauncherReturnProps;

/*
Gamepad Map for LM1 TeleOp
The FTCPadMap file and image available in /TeamCode/src/main/gamepadMaps/lm1.ftcpadmap and /TeamCode/src/main/gamepadMaps/lm1.png
You can't view in Android Studio unless you switch from "Android" to "Project" view in the left side file viewer, then navigate to the file
Upload the file to https://barrow-robotics-ftc.github.io/FTCPadMap/ for an interactive view

Drive Coach: NAME
Human Player: NAME
Gamepad 1 (Driver): NAME
    Left Stick X: Robot translation movement
        - Can be used to disable automated driving or position holding
    Left Stick Y: Robot axial movement
        - Can be used to disable automated driving or position holding
    Right Stick X: Robot rotational movement
        - Can be used to disable automated driving or position holding
    DPad Right: Toggle slow mode
    A: Go to scoring position
    Y: Go to human player position
    B: Go to base zone (endgame parking) position
    Left Bumper: Hold current position (lock robot movement and correct error with Pedro Pathing)
    Right Bumper: Toggle launcher speed up (will hold speed once sped up until you press this again)
    Left Trigger: Launch 1 artifact (position will be held automatically until launch completes)
    Right Trigger: Launch 3 artifacts (position will be held automatically until launch completes)
 */

@TeleOp(name = "LM1 TeleOp", group = "TeleOp")
@SuppressWarnings("FieldCanBeLocal") // Suppress pointless Android Studio warnings
public class LM1TeleOp extends LinearOpMode {
    // Values retrieved from blackboard
    private Pose autoEndPose; // End pose of the autonomous, start pose of TeleOp
    private Constants.Paths paths; // Custom paths class

    // Driver controller variables
    private boolean slowMode = false;

    // Other variables
    private final ElapsedTime runtime = new ElapsedTime();
    private Follower follower; // Pedro pathing follower
    private Pose currentPose; // Current pose of the robot
    private boolean automatedDrive = false; // Is Pedro Pathing driving?
    private LauncherReturnProps launcherStatus; // Current launcher state
    private Launcher launcher; // Custom launcher class
    private boolean launcherIsLaunching = false; // Is the launcher currently launching?
    private boolean holdingPoint = false; // Is Pedro Pathing holding a point?

    private void stopAutoDrive() {
        automatedDrive = false; // No longer in automated drive
        holdingPoint = false; // No longer holding point
        follower.startTeleopDrive(BRAKE_MODE); // Restart manual control
    }

    @Override
    public void runOpMode() {
        // Get variables from Blackboard
        autoEndPose = (Pose) blackboard.getOrDefault("autoEndPose", new Pose(72, 8, Math.toRadians(90)));
        paths = (Constants.Paths) blackboard.getOrDefault("paths", null);

        // Initialize the Pedro Pathing follower and set the start pose to the autonomous ending pose
        follower = Constants.Pedro.createFollower(hardwareMap);
        follower.setStartingPose(autoEndPose);
        follower.update();

        // Initialize all utilities used in TeleOp
        launcher = new Launcher(hardwareMap);

        // Log completed initialization to Panels and driver station
        telemetry.addData("Status", "Initialized");
        telemetry.update(); // Update Panels and driver station after logging

        // Wait for the TeleOp period to start (driver presses START)
        waitForStart();
        runtime.reset();

        // Start with TeleOp (manual) drive
        follower.startTeleopDrive(BRAKE_MODE);

        while (opModeIsActive()) {
            // Update Pedro Pathing every iteration
            follower.update();
            currentPose = follower.getPose();

            // If the robot isn't being controlled by Pedro Pathing, send gamepad inputs to the robot
            if (!automatedDrive) {
                // Set movement vectors based on gamepad inputs
                follower.setTeleOpDrive(
                        -gamepad1.left_stick_y * (slowMode ? SLOW_MODE_MULTIPLIER : NORMAL_SPEED_MULTIPLIER),
                        -gamepad1.left_stick_x * (slowMode ? SLOW_MODE_MULTIPLIER : NORMAL_SPEED_MULTIPLIER),
                        -gamepad1.right_stick_x * (slowMode ? SLOW_MODE_MULTIPLIER : NORMAL_SPEED_MULTIPLIER),
                        ROBOT_CENTRIC
                );
            } else {
                if (holdingPoint) { // If holding point, check if we need to release
                    // If the driver moves the joystick, release the hold
                    if (Math.abs(gamepad1.left_stick_x) > 0.25 || Math.abs(gamepad1.left_stick_y) > 0.25 || Math.abs(gamepad1.right_stick_x) > 0.25) {
                        stopAutoDrive(); // Stop automated driving
                    }
                } else if (!follower.isBusy()) { // If not holding point, check if the path is complete
                    stopAutoDrive(); // Stop automated driving
                }
            }

            // Gamepad 1 DPad Right: toggle slow mode
            if (gamepad1.dpadRightWasPressed()) {
                slowMode = !slowMode;
            }

            // Gamepad 1 A: Go to scoring position
            if (gamepad1.aWasPressed()) {
                follower.followPath(paths.buildPath(follower, follower.getPose(), paths.poses.score));
                automatedDrive = true; // Start auto drive
            }

            // Gamepad 1 Y: Go to human player position
            if (gamepad1.yWasPressed()) {
                follower.followPath(paths.buildPath(follower, follower.getPose(), paths.poses.loadingZone));
                automatedDrive = true; // Start auto drive
            }

            // Gamepad 1 B: Go to base zone (endgame parking) position
            if (gamepad1.bWasPressed()) {
                follower.followPath(paths.buildPath(follower, follower.getPose(), paths.poses.baseZone));
                automatedDrive = true; // Start auto drive
            }

            // Gamepad 1 Left Bumper: Hold current pose
            if (gamepad1.leftBumperWasPressed()) {
                follower.holdPoint(currentPose); // Hold position at the current pose
                holdingPoint = true; // Indicate that we are holding point
                automatedDrive = true; // Start auto drive
            }

            // Gamepad 1 Right Bumper: Toggle launcher speed up
            if (gamepad1.rightBumperWasPressed() && launcherStatus.state == LauncherState.IDLE) {
                launcher.speedUp(); // Speed up the launcher
            }

            // Gamepad 1 Left Trigger: Launch 1 artifact
            if (gamepad1.right_trigger > 0.5 && !launcherIsLaunching) {
                // Note: If the launch command will call speedUp() if the launcher is idle, so no need to check here
                launcher.launch(1); // Start the launch of 1 artifact
                launcherIsLaunching = true; // Indicate that the launcher is launching
            }

            // Gamepad 1 Right Trigger: Launch 3 artifacts
            if (gamepad1.right_trigger > 0.5 && !launcherIsLaunching) {
                // Note: If the launch command will call speedUp() if the launcher is idle, so no need to check here
                launcher.launch(3); // Start the launch of 3 artifacts
                launcherIsLaunching = true; // Indicate that the launcher is launching
            }

            // Update launcher (nothing will happen when launcher is idle)
            launcherStatus = launcher.update();
            if (launcherStatus.cycleCompleted) { // If a launch cycle has completed
                stopAutoDrive(); // Stop holding the position once the launch is complete
                launcherIsLaunching = false; // Indicate that the launcher is no longer launching
            }

            // Log status
            telemetry.addData("Run Time: ", runtime.seconds());
            telemetry.addData("Automated Drive: ", automatedDrive);
            telemetry.addData("Holding Point: ", holdingPoint);
            telemetry.addData("Launcher State: ", launcherStatus.state);
            telemetry.addData("", "");
            telemetry.addData("X: ", currentPose.getX());
            telemetry.addData("Y: ", currentPose.getY());
            telemetry.addData("Heading: ", currentPose.getHeading());
            telemetry.update();

            // No intake for meet one, so this code may be referenced in LM2
            /*
            // When Left trigger is pressed: Initialize / Disable Intake
            if (gamepad2.left_trigger > 0.1) { // When right bumper is pressed
                intakeRunning = !intakeRunning; // Toggle intake state
                if (intakeRunning) {
                    intake.run(); // Start the intake
                }
                else {
                    intake.stop(); // Stop the intake
                }
            }
            */
        }
    }
}