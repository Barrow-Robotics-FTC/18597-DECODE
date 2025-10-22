package org.firstinspires.ftc.teamcode.teleop;

// FTC SDK
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

// Pedro Pathing
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;
import com.pedropathing.geometry.Pose;

// Local helper files
import org.firstinspires.ftc.teamcode.utils.Launcher;
import org.firstinspires.ftc.teamcode.utils.Intake;
import org.firstinspires.ftc.teamcode.utils.Constants;

import org.firstinspires.ftc.teamcode.utils.Launcher;
import org.firstinspires.ftc.teamcode.utils.Constants.LauncherConstants.LauncherState;
import org.firstinspires.ftc.teamcode.utils.Constants.LauncherConstants.LauncherReturnProps;

/*
Gamepad Map for LM1 TeleOp (FTCPadMap file available in this programs folder)
Upload the file to https://barrow-robotics-ftc.github.io/FTCPadMap/ for an interactive view

Drive Coach: NAME
Human Player: NAME
Gamepad 1 (Driver): NAME
    Right Trigger (When Pressed): Go to scoring position
    Left Trigger (When Pressed): Go to human player position
Gamepad 2 (Operator): NAME
 */

@TeleOp(name = "LM1 TeleOp", group = "TeleOp")
@SuppressWarnings("FieldCanBeLocal") // Stop Android Studio from bugging about variables being predefined
public class LM1TeleOp extends LinearOpMode {
    // Editable variables
    private final boolean brakeMode = true; // Whether the motors should break on stop (recommended)
    private final boolean robotCentric = true; // True for robot centric driving, false for field centric
    private final double slowModeMultiplier = 0.5; // Multiplier for slow mode speed
    private final double nonSlowModeMultiplier = 1; // Multiplier for normal driving speed

    // Values retrieved from blackboard
    private Constants.Alliance alliance; // Alliance of the robot
    private Pose autoEndPose; // End pose of the autonomous, start pose of TeleOp

    // Driver controller variables
    private boolean slowMode = false;
    private boolean launching = false;
    // private boolean intaking = false;

    // Other variables
    private final ElapsedTime runtime = new ElapsedTime();
    private Follower follower; // Pedro pathing follower
    private Constants.Paths paths; // Custom paths class
    private Pose currentPose; // Current pose of the robot
    private boolean automatedDrive = false; // Is Pedro Pathing driving?
    private boolean launcherActive = false; // Is the launcher update loop running?
    // private boolean intakeActive = false; // Is the intake running?
    private boolean readyForLaunch = false; // Are we in position to allow the launcher to launch?
    private Intake intake; // Custom intake class
    // private boolean intakeRunning = false; // True when intake is running
    // Go from the current position to any pose
    private Launcher launcher; // Custom launcher class
    private LauncherReturnProps launcherStatus; // Current launcher state
    private ElapsedTime launchCycleTimer = new ElapsedTime(); // Keeps track of the time since the last completed launch cycle


    private PathChain getPathToPose(Pose pose) {
        // Flip the pose if the alliance is blue
        if (alliance == Constants.Alliance.BLUE) {
            pose = pose.mirror();
        }

        // Return the PathChain
        return follower.pathBuilder()
                .addPath(new BezierLine(follower.getPose(), pose))
                .setLinearHeadingInterpolation(follower.getHeading(), pose.getHeading())
                .build();
    }

    @Override
    public void runOpMode() {
        // Get variables from Blackboard
        alliance = (Constants.Alliance) blackboard.getOrDefault("alliance", Constants.Alliance.RED);
        autoEndPose = (Pose) blackboard.getOrDefault("autoEndPose", new Pose(72, 8, Math.toRadians(90)));
        paths = (Constants.Paths) blackboard.getOrDefault("paths", null);

        // Initialize the Pedro Pathing follower and set the start pose to the autonomous ending pose
        follower = Constants.Pedro.createFollower(hardwareMap);
        follower.setStartingPose(autoEndPose);
        follower.update();

        // Initialize all utilities used in TeleOp
        Launcher launcher = new Launcher(hardwareMap);
        Intake intake = new Intake(hardwareMap);

        // Log completed initialization to Panels and driver station
        telemetry.addData("Status", "Initialized");
        telemetry.update(); // Update Panels and driver station after logging

        // Wait for the TeleOp period to start (driver presses START)
        waitForStart();
        runtime.reset();

        // Start with TeleOp (manual) drive
        follower.startTeleopDrive(brakeMode);

        while (opModeIsActive()) {
            // Update Pedro Pathing every iteration
            follower.update();
            currentPose = follower.getPose();

            // If the robot isn't being controlled by Pedro Pathing, send gamepad inputs to the robot
            if (!automatedDrive) {
                follower.setTeleOpDrive(
                        -gamepad1.left_stick_y * (slowMode ? slowModeMultiplier : nonSlowModeMultiplier),
                        -gamepad1.left_stick_x * (slowMode ? slowModeMultiplier : nonSlowModeMultiplier),
                        -gamepad1.right_stick_x * (slowMode ? slowModeMultiplier : nonSlowModeMultiplier),
                        robotCentric
                );
            } else if ((gamepad1.bWasPressed() || !follower.isBusy())) {
                // If auto drive is active, check if the follower is busy
                // B (on press): Stop auto drive
                follower.startTeleopDrive(brakeMode); // Restart manual control
                automatedDrive = false;
            }

            // Gamepad 1 when Y (When Pressed): Go to scoring position
            if (gamepad1.yWasPressed()) {
                follower.followPath(paths.buildPath(follower, follower.getPose(), paths.poses.score));
                automatedDrive = true; // Start auto drive
            }
            // Gamepad 1 when A (When Pressed): Go to human player position
            if (gamepad1.aWasPressed()) {
                follower.followPath(paths.buildPath(follower, follower.getPose(), paths.poses.loadingZone));
                automatedDrive = true; // Start auto drive
            }
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
            // When d-pad Up is pressed: enable / disable slow mode
            if (gamepad1.dpadRightWasPressed()) {
                if (slowMode) {
                    slowMode = true;
                }
                else {
                    slowMode = false;
                }
            }
            // When right trigger is pressed: Enable / Disable Launcher
            if (gamepad2.rightBumperWasPressed()) {
                if (gamepad1.leftBumperWasPressed() && launcherStatus.state == LauncherState.IDLE) {
                    launcher.speedUp(); // Speed up the launcher
                }

                // If the right bumper is pressed
                if (gamepad2.right_trigger > 0.1) {
                    launcher.launch(1); // Launch an artifact
                }

                // Use DPad buttons to control launcher speed
                if (gamepad2.dpadUpWasPressed()) {
                    launcher.setTargetRPM(launcher.getTargetRPM() + 25); // Increase target RPM by 25
                }
                else if (gamepad2.dpadDownWasPressed()) {
                    launcher.setTargetRPM(launcher.getTargetRPM() - 25); // Decrease target RPM by 25
                }

                // Update launcher
                launcherStatus = launcher.update();

                // Check if a launch cycle was completed
                if (launcherStatus.cycleCompleted) {
                    launchCycleTimer.reset();
                }
            }

            // Log status
            telemetry.addData("X: ", currentPose.getX());
            telemetry.addData("Y: ", currentPose.getY());
            telemetry.addData("Heading: ", currentPose.getHeading());
            telemetry.update();
        }
    }
}