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

/*
Gamepad Map for TeleOp (FTCPadMap file available in this programs folder)
Upload the file to https://barrow-robotics-ftc.github.io/FTCPadMap/ for an interactive view

Drive Coach: NAME
Human player: NAME
Gamepad 1 (Driver): NAME
    Right Trigger (When Pressed): Go to scoring position
    Left Trigger (When Pressed): Go to human player position
Gamepad 2 (Operator): NAME
 */

@TeleOp(name = "Basic TeleOp", group = "TeleOp")
@SuppressWarnings("FieldCanBeLocal") // Stop Android Studio from bugging about variables being predefined
public class BasicTeleOp extends LinearOpMode {
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
    private boolean intaking = false;

    // Other variables
    private final ElapsedTime runtime = new ElapsedTime();
    private Follower follower; // Pedro pathing follower
    private Constants.Paths paths; // Custom paths class
    private Pose currentPose; // Current pose of the robot
    private boolean automatedDrive = false; // Is Pedro Pathing driving?
    private boolean launcherActive = false; // Is the launcher update loop running?
    private boolean intakeActive = false; // Is the intake running?
    private boolean readyForLaunch = false; // Are we in position to allow the launcher to launch?

    // Go from the current position to any pose
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

            // Gamepad 1 Right Trigger (When Pressed): Go to scoring position
            if (gamepad1.right_trigger > 0.1) {
                follower.followPath(paths.buildPath(follower, follower.getPose(), paths.poses.score));
                automatedDrive = true; // Start auto drive
            }
            // Gamepad 1 Left Trigger (When Pressed): Go to human player position
            if (gamepad1.left_trigger > 0.1) {
                follower.followPath(paths.buildPath(follower, follower.getPose(), paths.poses.loadingZone));
                automatedDrive = true; // Start auto drive
            }
            // Initialize Intake
            if (gamepad2.leftBumperWasPressed()) {
            //
            }
            // Disable Intake
            if (gamepad2.rightBumperWasPressed()) {
                //
            }
            // Enable Slow Mode
            if (gamepad1.leftStickButtonWasPressed()) {
                //
            }
            // Disable Slow Mode
            if (gamepad1.rightStickButtonWasPressed()) {
                //
            }
            // Disable Launcher
            if (gamepad2.left_trigger < 0.1) {
                //
            }
            // Enable Launcher
            if (gamepad2.right_trigger > 0.1) {
                //
            }

            // Log status
            telemetry.addData("X: ", currentPose.getX());
            telemetry.addData("Y: ", currentPose.getY());
            telemetry.addData("Heading: ", currentPose.getHeading());
            telemetry.update();
        }
    }
}