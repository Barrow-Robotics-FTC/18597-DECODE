package org.firstinspires.ftc.teamcode.teleop;

import com.pedropathing.ivy.commands.Infinite;
import com.pedropathing.ivy.commands.Instant;
import com.pedropathing.ivy.commands.Wait;
import com.pedropathing.ivy.groups.Sequential;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.pedropathing.geometry.Pose;

import static org.firstinspires.ftc.teamcode.Constants.TeleOpConstants.*;
import org.firstinspires.ftc.teamcode.Constants.MovementVectors;
import org.firstinspires.ftc.teamcode.Constants.Alliance;
import org.firstinspires.ftc.teamcode.Constants.Poses;
import org.firstinspires.ftc.teamcode.Constants.Mode;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.command.CommandOpMode;
import org.firstinspires.ftc.teamcode.command.FollowPath;

/**
 * LM3 TeleOp using Ivy Command Base
 * This maintains the exact same functionality as LM3TeleOp but uses commands
 * 
 * Gamepad Map for LM3 Ivy TeleOp
 * 
 * Drive Coach: Katy
 * Human Player: Cedar
 * Gamepad 1 (Driver): Dylan
 *     Left Stick X: Robot translation movement
 *     Left Stick Y: Robot axial movement
 *     Right Stick X: Robot rotational movement
 *     Left Bumper: Toggle slow mode
 *     Circle: Localize (set robot pose to human player corner)
 * Gamepad 2 (Operator): Parley
 *     Left Bumper: Toggle intake
 *     Right Bumper: Toggle launcher speed up (will hold speed once sped up until you press this again)
 *     Left Trigger: Launch 1 artifact (position will be held automatically until launch completes)
 *     Right Trigger: Launch 3 artifacts (position will be held automatically until launch completes)
 */
@TeleOp(name = "LM3 Ivy TeleOp", group = "TeleOp")
public class LM3IvyTeleOp extends CommandOpMode {
    // Values retrieved from blackboard
    private Pose autoEndPose;
    private Alliance alliance;

    // Robot
    private Robot robot;

    // Variables
    private boolean slowMode = false;
    private Pose currentPose;
    private int artifactsToLaunch = 0;
    private boolean liningUpWithGoal = false;

    @Override
    public void init() {
        // Get variables from Blackboard
        alliance = (Alliance) blackboard.getOrDefault("alliance", Alliance.BLUE);
        autoEndPose = (Pose) blackboard.getOrDefault("autoEndPose", null);

        // Initialize robot
        robot = new Robot(hardwareMap, Mode.TELEOP);
        robot.buildPoses(alliance);

        // Set starting pose to the end pose of auto (if it doesn't exist, use audience start)
        robot.drivetrain.setPose(autoEndPose == null ? robot.poses.audienceStart : autoEndPose);

        // Log completed initialization
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Schedule the main TeleOp loop
        schedule(
                new Sequential(
                        new Wait(1),
                        new Infinite(() -> {
                            // Update robot
                            robot.periodic();
                            currentPose = robot.drivetrain.getPose();

                            // Handle driver controls
                            handleDriverControls();

                            // Handle operator controls
                            handleOperatorControls();

                            // Set gamepad colors
                            setGamepadColors();

                            // Update telemetry
                            telemetry.addData("Run Time: ", getRuntime());
                            telemetry.addData("Automated Drive: ", robot.drivetrain.isDriving());
                            telemetry.addData("Launcher State: ", robot.launcher.getState());
                            telemetry.addData("X: ", currentPose.getX());
                            telemetry.addData("Y: ", currentPose.getY());
                            telemetry.addData("Heading: ", Math.toDegrees(currentPose.getHeading()));
                            telemetry.update();
                        })
                )
        );
    }

    @Override
    public void start() {
        // Start TeleOp driving
        robot.drivetrain.follower.startTeleOpDrive(BRAKE_MODE);
    }

    private void handleDriverControls() {
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
            }

            // Cancel lineup if triangle is pressed
            if (gamepad2.triangle) {
                liningUpWithGoal = false;
            }
        }

        // Gamepad 1 Left Bumper: Toggle slow mode
        if (gamepad1.left_bumper && !wasLeftBumperPressed) {
            slowMode = !slowMode;
        }
        wasLeftBumperPressed = gamepad1.left_bumper;

        /* Temporarily disabled to prevent accidental localization because it doesn't work
        // Gamepad 1 Circle: Localize
        if (gamepad1.circle && !wasCirclePressed) {
            // Set the robot pose to the localization pose (human player corner)
            robot.drivetrain.setPose(robot.poses.localize);
        }
        wasCirclePressed = gamepad1.circle;
        */
    }

    private void handleOperatorControls() {
        // Gamepad 2 Left Bumper: Toggle intake
        if (gamepad2.left_bumper && !wasLeftBumper2Pressed) {
            if (robot.intake.isActive()) {
                robot.intake.stop();
            } else {
                robot.intake.run();
            }
        }
        wasLeftBumper2Pressed = gamepad2.left_bumper;

        // Gamepad 2 Right Bumper: Toggle launcher speed up
        if (gamepad2.right_bumper && !wasRightBumper2Pressed) {
            if (robot.launcher.isActive()) {
                robot.launcher.stop();
            } else {
                robot.launcher.speedUp(true);
            }
        }
        wasRightBumper2Pressed = gamepad2.right_bumper;

        // Gamepad 2 Left Trigger: Launch 1 artifact
        if (gamepad2.left_trigger > 0.5 && !wasLeftTrigger2Pressed && !robot.launcher.isLaunching()) {
            startLaunch(1);
        }
        wasLeftTrigger2Pressed = gamepad2.left_trigger > 0.5;

        // Gamepad 2 Right Trigger: Launch 3 artifacts
        if (gamepad2.right_trigger > 0.5 && !wasRightTrigger2Pressed && !robot.launcher.isLaunching()) {
            startLaunch(3);
        }
        wasRightTrigger2Pressed = gamepad2.right_trigger > 0.5;

        // If we are in the process of launching artifacts
        if (artifactsToLaunch > 0) {
            // This won't run until we are lined up with the goal
            if (!liningUpWithGoal) {
                robot.launcher.launch(artifactsToLaunch);
                artifactsToLaunch = 0;
            }
        }
    }

    private void startLaunch(int numArtifacts) {
        artifactsToLaunch = numArtifacts;
        liningUpWithGoal = true;

        // Follow path to scoring pose (Slowed down as this should be used in the vicinity of the goal)
        robot.drivetrain.followPath(Poses.buildPath(robot.drivetrain, robot.poses.score), 0.75);
    }

    private void setGamepadColors() {
        // Set Gamepad 1 lights (priority goes first to last)
        if (robot.launcher.isLaunching()) { // Launch process is running: purple
            robot.setGamepad1Color(150, 0, 255);
        } else if (slowMode) { // Slow mode: yellow
            robot.setGamepad1Color(255, 255, 0);
        } else { // None of the above conditions: default
            robot.setGamepad1Color(0, 0, 0);
        }

        // Set Gamepad 2 lights (priority goes first to last)
        if (robot.launcher.isLaunching()) { // Launch process is running: purple
            robot.setGamepad2Color(150, 0, 255);
        } else if (robot.launcher.isActive()) { // Launcher is active: yellow
            robot.setGamepad2Color(255, 255, 0);
        } else { // None of the above conditions: default
            robot.setGamepad2Color(0, 0, 0);
        }

        // Apply gamepad colors
        gamepad1.setLedColor(gamepad1Color[0], gamepad1Color[1], gamepad1Color[2], -1);
        gamepad2.setLedColor(gamepad2Color[0], gamepad2Color[1], gamepad2Color[2], -1);
    }

    @Override
    public void stop() {
        // Stop the robot completely after TeleOp ends
        robot.stop(gamepad1, gamepad2);
        reset();
    }

    // Button state tracking (for edge detection)
    private boolean wasLeftBumperPressed = false;
    private boolean wasCirclePressed = false;
    private boolean wasLeftBumper2Pressed = false;
    private boolean wasRightBumper2Pressed = false;
    private boolean wasLeftTrigger2Pressed = false;
    private boolean wasRightTrigger2Pressed = false;

    // Gamepad color arrays
    private final int[] gamepad1Color = new int[]{0, 0, 0};
    private final int[] gamepad2Color = new int[]{0, 0, 0};
}
