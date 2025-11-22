package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.pedropathing.geometry.Pose;

import static org.firstinspires.ftc.teamcode.Constants.TeleOpConstants.*;
import org.firstinspires.ftc.teamcode.Constants.MovementVectors;
import org.firstinspires.ftc.teamcode.Constants.Alliance;
import org.firstinspires.ftc.teamcode.Constants.Poses;
import org.firstinspires.ftc.teamcode.Constants.Mode;
import org.firstinspires.ftc.teamcode.Robot;

/*
Gamepad Map for LM3 TeleOp

Drive Coach: Katy
Human Player: Cedar
Gamepad 1 (Driver): Dylan
    Left Stick X: Robot translation movement
    Left Stick Y: Robot axial movement
    Right Stick X: Robot rotational movement
    Left Bumper: Toggle slow mode
Gamepad 2 (Operator): Parley
    Left Bumper: Toggle intake
    Right Bumper: Toggle launcher speed up (will hold speed once sped up until you press this again)
    Left Trigger: Launch 1 artifact (position will be held automatically until launch completes)
    Right Trigger: Launch 3 artifacts (position will be held automatically until launch completes)
 */

@TeleOp(name = "LM3 TeleOp", group = "TeleOp")
@SuppressWarnings("FieldCanBeLocal") // Suppress pointless Android Studio warnings
public class LM3TeleOp extends LinearOpMode {
    // Values retrieved from blackboard
    private Pose autoEndPose; // End pose of the autonomous, start pose of TeleOp
    private Alliance alliance; // Alliance color

    // Driver controller variables
    private boolean slowMode = false;

    // Utilities
    private final ElapsedTime runtime = new ElapsedTime();
    private Robot robot; // Robot object

    // Variables
    private Pose currentPose; // Current pose of the robot
    private int artifactsToLaunch = 0; // Number of artifacts to launch
    private boolean liningUpWithGoal = false; // Is the robot currently lining up with the goal?
    private boolean launcherIsActive = false; // Is the launcher currently active (sped up)?
    private boolean launcherIsLaunching = false; // Is the launcher currently launching?

    private void startLaunch(int numArtifacts) {
        artifactsToLaunch = numArtifacts; // Indicate that we want to launch the specified number of artifacts
        liningUpWithGoal = true; // Start by lining up with the goal

        // Follow path to scoring pose
        robot.drivetrain.followPath(Poses.buildPath(robot.drivetrain, robot.poses.score));
    }

    @Override
    public void runOpMode() {
        // Get variables from Blackboard
        alliance = (Alliance) blackboard.getOrDefault("alliance", Alliance.BLUE);
        autoEndPose = (Pose) blackboard.getOrDefault("autoEndPose", new Pose(alliance == Alliance.BLUE ? 56 : 88, 8, Math.toRadians(90)));

        // Initialize robot
        robot = new Robot(hardwareMap, Mode.TELEOP);

        // Set starting pose to the end pose of auto
        robot.drivetrain.setStartingPose(autoEndPose);

        // Log completed initialization
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the TeleOp period to start (driver presses START)
        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            // Update drivetrain
            robot.update(gamepad1, gamepad2);
            currentPose = robot.drivetrain.getPose();

            // Check if we are lining up with the goal
            if (!liningUpWithGoal) {
                // Set movement vectors based on gamepad inputs
                robot.drivetrain.setMovementVectors(new MovementVectors(
                        -gamepad1.left_stick_y * (slowMode ? SLOW_MODE_MULTIPLIER : NORMAL_SPEED_MULTIPLIER),
                        -gamepad1.left_stick_x * (slowMode ? SLOW_MODE_MULTIPLIER : NORMAL_SPEED_MULTIPLIER),
                        -gamepad1.right_stick_x * (slowMode ? SLOW_MODE_MULTIPLIER : NORMAL_SPEED_MULTIPLIER)
                ));
            } else {
                // When the launch is started, we should be following a path to the scoring pose
                if (!robot.drivetrain.isDriving()) { // When we reach the scoring pose
                    liningUpWithGoal = false; // Finished lining up
                    robot.drivetrain.holdPose(robot.poses.score); // Hold the scoring pose
                }
            }

            // Gamepad 1 Left Bumper: Toggle slow mode
            if (gamepad1.leftBumperWasPressed()) {
                slowMode = !slowMode;
            }

            // Gamepad 2 Left Bumper: Toggle intake
            if (gamepad2.leftBumperWasPressed()) {
                if (robot.intake.isActive()) {
                    robot.intake.stop(); // Stop the intake
                } else {
                    robot.intake.run(); // Start the intake
                }
            }

            // Gamepad 2 Right Bumper: Toggle launcher speed up
            if (gamepad2.rightBumperWasPressed()) {
                if (launcherIsActive) {
                    robot.launcher.stop(); // Stop the launcher
                    launcherIsActive = false; // Set active flag to false
                } else {
                    robot.launcher.speedUp(); // Speed up the launcher
                    launcherIsActive = true; // Set active flag to true
                }
            }

            // Gamepad 2 Left Trigger: Launch 1 artifact
            if (robot.gamepad2LeftTriggerPressed(gamepad2) && !launcherIsLaunching) {
                startLaunch(1); // Indicate that we want to launch 1 artifact
            }

            // Gamepad 2 Right Trigger: Launch 3 artifacts
            if (robot.gamepad2RightTriggerPressed(gamepad2) && !launcherIsLaunching) {
                startLaunch(3); // Indicate that we want to launch 3 artifacts
            }

            // If we are in the process of launching artifacts
            if (artifactsToLaunch > 0) {
                if (!liningUpWithGoal) {
                    // Note: line up is handled above in drivetrain update
                    // Note: The launch command will call speedUp() if the launcher is idle, so no need to check here
                    robot.launcher.launch(artifactsToLaunch); // Start the launch of artifacts
                    launcherIsLaunching = true; // Indicate that the launcher is launching

                    // Reset flags
                    artifactsToLaunch = 0; // Reset the launch request
                }
            }

            // Check if a launch cycle was completed
            if (robot.launcher.didCompleteCycle()) { // If a launch cycle has completed
                launcherIsLaunching = false; // Indicate that the launcher is no longer launching
            }

            // Set Gamepad 1 lights (priority goes first to last)
            if (liningUpWithGoal) { // Lining up with goal: purple
                robot.setGamepad1Color(255, 0, 255);
            } else if (slowMode) { // Slow mode: blue
                robot.setGamepad1Color(0, 0, 255);
            } else { // None of the above conditions: off
                robot.setGamepad1Color(0, 0, 0);
            }

            // Set Gamepad 2 lights (priority goes first to last)
            if (liningUpWithGoal) { // Lining up with goal: purple
                robot.setGamepad2Color(255, 0, 255);
            } else if (launcherIsLaunching) { // Launcher is launching: green
                robot.setGamepad2Color(0, 255, 0);
            } else if (launcherIsActive) { // Launcher is active: blue
                robot.setGamepad2Color(0, 0, 255);
            } else { // None of the above conditions: off
                robot.setGamepad2Color(0, 0, 0);
            }

            // Log status
            telemetry.addData("Run Time: ", runtime.seconds());
            telemetry.addData("Automated Drive: ", robot.drivetrain.isDriving());
            telemetry.addData("Launcher State: ", robot.launcher.getState());
            telemetry.addData("Artifacts to launch", artifactsToLaunch);
            telemetry.addData("X: ", currentPose.getX());
            telemetry.addData("Y: ", currentPose.getY());
            telemetry.addData("Heading: ", Math.toDegrees(currentPose.getHeading()));
            telemetry.update();
        }

        // Stop the robot completely after TeleOp ends
        robot.stop(gamepad1, gamepad2);
    }
}
