package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.ivy.commands.Infinite;
import com.pedropathing.ivy.commands.Instant;
import com.pedropathing.ivy.commands.Wait;
import com.pedropathing.ivy.commands.WaitUntil;
import com.pedropathing.ivy.groups.Sequential;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.Constants.Alliance;
import org.firstinspires.ftc.teamcode.Constants.StartPosition;
import org.firstinspires.ftc.teamcode.Constants.Poses;
import org.firstinspires.ftc.teamcode.command.CommandOpMode;
import org.firstinspires.ftc.teamcode.command.FollowPath;

/**
 * LM3 Autonomous using Ivy Command Base
 * This maintains the exact same functionality as LM3Auto but uses commands instead of state machines
 */
@Autonomous(name = "LM3 Ivy Auto", group = "Autonomous")
public class LM3IvyAuto extends CommandOpMode {
    // Time to wait before starting the autonomous
    private static final double AUTO_START_DELAY = 0; // Milliseconds

    private Robot robot;
    private Alliance alliance;
    private StartPosition startPosition;
    private Pose currentPose;

    @Override
    public void init() {
        // Initialize robot
        robot = new Robot(hardwareMap, Constants.Mode.AUTO);

        // Prompt the driver to select an alliance and start position
        alliance = robot.selectAlliance(gamepad1, telemetry);
        startPosition = robot.selectStartPosition(gamepad1, telemetry);

        // Build poses based on alliance
        robot.buildPoses(alliance);

        // Set variables based on start position
        if (startPosition == StartPosition.GOAL_WALL) {
            robot.drivetrain.setPose(robot.poses.goalStart); // Set starting pose
        } else {
            robot.drivetrain.setPose(robot.poses.audienceStart);
        }
        robot.drivetrain.update(robot); // Update drivetrain to set starting pose

        // Log completed initialization
        telemetry.addData("Status", "Initialized");
        telemetry.addData("Alliance", alliance);
        telemetry.addData("Start Position", startPosition);
        telemetry.update();

        // Schedule commands for autonomous
        // This runs the exact same 9-artifact auto sequence as LM3Auto
        schedule(
                // Periodic update for all robot subsystems
                new Infinite(() -> {
                    robot.periodic();
                    currentPose = robot.drivetrain.getPose();
                }),
                // Telemetry update
                new Infinite(() -> {
                    telemetry.addData("Launcher State", robot.launcher.getState());
                    telemetry.addData("Launcher Left RPM", robot.launcher.getLeftRPM(robot));
                    telemetry.addData("Launcher Right RPM", robot.launcher.getRightRPM(robot));
                    telemetry.addData("Tapper State", robot.tapper.getState());
                    telemetry.addData("Intake State", robot.intake.getState());
                    telemetry.addData("X", currentPose.getX());
                    telemetry.addData("Y", currentPose.getY());
                    telemetry.addData("Heading", currentPose.getHeading());
                    telemetry.update();
                }),
                // Main autonomous sequence
                new Sequential(
                        // Wait for auto start delay
                        new Wait((long) AUTO_START_DELAY),
                        
                        // Start the launcher, it will stay active throughout the auto
                        robot.speedUpLauncherCommand(false),
                        
                        // 9 artifact auto sequence (same as LM3Auto stateList)
                        // Move from starting position to scoring position
                        new FollowPath(robot, Poses.buildPath(robot.drivetrain, robot.poses.score), 0.8),
                        
                        // Launch preloaded artifacts (3)
                        robot.launchCommand(3),
                        
                        // Move to GPP artifacts
                        new FollowPath(robot, Poses.buildPath(robot.drivetrain, robot.poses.GPPArtifacts)),
                        
                        // Intake artifacts from GPP row
                        robot.startIntakeCommand(),
                        new FollowPath(robot, Poses.buildPath(robot.drivetrain, robot.poses.GPPArtifactsEnd), 0.85),
                        robot.stopIntakeCommand(),
                        robot.speedUpLauncherCommand(false),
                        
                        // Move to scoring position
                        new FollowPath(robot, Poses.buildPath(robot.drivetrain, robot.poses.score), 0.8),
                        
                        // Launch artifacts
                        robot.launchCommand(3),
                        
                        // Move to PGP artifacts
                        new FollowPath(robot, Poses.buildPath(robot.drivetrain, robot.poses.PGPArtifacts)),
                        
                        // Intake artifacts from PGP row
                        robot.startIntakeCommand(),
                        new FollowPath(robot, Poses.buildPath(robot.drivetrain, robot.poses.PGPArtifactsEnd), 0.85),
                        robot.stopIntakeCommand(),
                        robot.speedUpLauncherCommand(false),
                        
                        // Move back to PGP (handles the double MOVE_TO_PGP in original)
                        new FollowPath(robot, Poses.buildPath(robot.drivetrain, robot.poses.PGPArtifacts)),
                        
                        // Move to scoring position
                        new FollowPath(robot, Poses.buildPath(robot.drivetrain, robot.poses.score), 0.8),
                        
                        // Launch artifacts
                        robot.launchCommand(3),
                        
                        // Move to PPG artifacts
                        new FollowPath(robot, Poses.buildPath(robot.drivetrain, robot.poses.PPGArtifacts)),
                        
                        // Intake artifacts from PPG row
                        robot.startIntakeCommand(),
                        new FollowPath(robot, Poses.buildPath(robot.drivetrain, robot.poses.PPGArtifactsEnd), 0.85),
                        robot.stopIntakeCommand(),
                        robot.speedUpLauncherCommand(false),
                        
                        // Move back to PPG (handles the double MOVE_TO_PPG in original)
                        new FollowPath(robot, Poses.buildPath(robot.drivetrain, robot.poses.PPGArtifacts))
                )
        );
    }

    @Override
    public void start() {
        // Auto is started via commands scheduled in init()
    }

    @Override
    public void stop() {
        // Save values for TeleOp
        blackboard.put("alliance", alliance);
        blackboard.put("autoEndPose", currentPose);
        
        // Stop the robot
        robot.stop(gamepad1, gamepad2);
        
        // Reset the scheduler
        reset();
    }
}
