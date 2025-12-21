package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.pedropathing.geometry.Pose;

import static org.firstinspires.ftc.teamcode.Constants.TeleOpConstants.*;
import org.firstinspires.ftc.teamcode.Constants.MovementVectors;
import org.firstinspires.ftc.teamcode.Constants.Alliance;
import org.firstinspires.ftc.teamcode.Constants.Mode;
import org.firstinspires.ftc.teamcode.Robot;

/*
Gamepad Map for LM4 TeleOp (Backup)

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

@TeleOp(name = "LM4 TeleOp (Backup)", group = "TeleOp")
@SuppressWarnings("FieldCanBeLocal") // Suppress pointless Android Studio warnings
public class LM4TeleOpBackup extends LinearOpMode {
    // Values retrieved from blackboard
    private Pose autoEndPose; // End pose of the autonomous, start pose of TeleOp
    private Alliance alliance; // Alliance color

    // Utilities
    private final ElapsedTime runtime = new ElapsedTime();
    private Robot robot; // Robot object

    // Variables
    private boolean slowMode = false;
    private Pose currentPose; // Current pose of the robot

    private void startLaunch(int numArtifacts) {
        robot.launcher.launch(numArtifacts); // Start the launch of artifacts
    }

    @Override
    public void runOpMode() {
        // Get variables from Blackboard
        alliance = (Alliance) blackboard.getOrDefault("alliance", Alliance.BLUE);
        autoEndPose = (Pose) blackboard.getOrDefault("autoEndPose", null);

        // Initialize robot
        robot = new Robot(hardwareMap, Mode.TELEOP, false);
        robot.buildPoses(alliance);

        // Set starting pose to the end pose of auto (if it doesn't exist, use audience start)
        robot.drivetrain.setPose(autoEndPose == null ? robot.poses.audienceStart : autoEndPose);

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

            // Set movement vectors based on gamepad inputs
            robot.drivetrain.setMovementVectors(new MovementVectors(
                    -gamepad1.left_stick_y * (slowMode ? SLOW_MODE_MULTIPLIER : NORMAL_SPEED_MULTIPLIER),
                    -gamepad1.left_stick_x * (slowMode ? SLOW_MODE_MULTIPLIER : NORMAL_SPEED_MULTIPLIER),
                    -gamepad1.right_stick_x * (slowMode ? SLOW_MODE_MULTIPLIER : NORMAL_SPEED_MULTIPLIER)
            ));

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
                if (robot.launcher.isActive()) {
                    robot.launcher.stop(); // Stop the launcher
                } else {
                    robot.launcher.speedUp(true); // Speed up the launcher
                }
            }

            // Gamepad 2 Left Trigger: Launch 1 artifact
            if (robot.gamepad2LeftTriggerPressed(gamepad2) && !robot.launcher.isLaunching()) {
                startLaunch(1); // Indicate that we want to launch 1 artifact
            }

            // Gamepad 2 Right Trigger: Launch 3 artifacts
            if (robot.gamepad2RightTriggerPressed(gamepad2) && !robot.launcher.isLaunching()) {
                startLaunch(3); // Indicate that we want to launch 3 artifacts
            }


            // Set Gamepad 1 lights (priority goes first to last)
            // Note when picking colors: Avoid red and blue as they are default colors
            if (robot.launcher.isLaunching()) { // Lining up with goal or launch process is running: purple
                robot.setGamepad1Color(150, 0, 255);
            } else if (slowMode) { // Slow mode: yellow
                robot.setGamepad1Color(255, 255, 0);
            } else { // None of the above conditions: default
                robot.setGamepad1Color(0, 0, 0);
            }

            // Set Gamepad 2 lights (priority goes first to last)
            if (robot.launcher.isLaunching()) { // Lining up with goal or launch process is running: purple
                robot.setGamepad2Color(150, 0, 255);
            } else if (robot.launcher.isActive()) { // Launcher is active: yellow
                robot.setGamepad2Color(255, 255, 0);
            } else { // None of the above conditions: default
                robot.setGamepad2Color(0, 0, 0);
            }

            // Log status
            telemetry.addData("Run Time: ", runtime.seconds());
            telemetry.addData("Automated Drive: ", robot.drivetrain.isDriving());
            telemetry.addData("Launcher State: ", robot.launcher.getState());
            telemetry.addData("X: ", currentPose.getX());
            telemetry.addData("Y: ", currentPose.getY());
            telemetry.addData("Heading: ", Math.toDegrees(currentPose.getHeading()));
            telemetry.update();
        }

        // Stop the robot completely after TeleOp ends
        robot.stop(gamepad1, gamepad2);
    }
}
