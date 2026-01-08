package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.Constants.Mode;
import org.firstinspires.ftc.teamcode.Constants.Alliance;
import org.firstinspires.ftc.teamcode.Constants.StartPosition;
import org.firstinspires.ftc.teamcode.Constants.Poses;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

@Autonomous(name = "LM4 Auto", group = "Autonomous")
@SuppressWarnings("FieldCanBeLocal") // Suppress pointless Android Studio warnings
public class LM4Auto extends LinearOpMode {
    // Time to wait before starting the autonomous
    private static final double AUTO_START_DELAY = 0; // Milliseconds

    // 9 artifact auto
    final List<State> stateList = new ArrayList<>(Arrays.asList(
            State.MOVE_TO_SCORING_POSITION, // Move from starting position to scoring position
            State.LAUNCH, // Score preloaded artifacts
            State.MOVE_TO_GPP, // Move in front of the PPG artifact row
            State.INTAKE_ARTIFACT_ROW, // Intake the artifacts from the GPP row
            State.MOVE_TO_SCORING_POSITION,
            State.LAUNCH,
            State.MOVE_TO_PGP,
            State.INTAKE_ARTIFACT_ROW,
            State.MOVE_TO_PGP,
            State.MOVE_TO_SCORING_POSITION,
            State.LAUNCH,
            State.MOVE_TO_PPG,
            State.INTAKE_ARTIFACT_ROW,
            State.MOVE_TO_PPG,
            State.COMPLETED // End of autonomous
    ));

    /*
    // 12 artifact auto
    final List<State> stateList = new ArrayList<>(Arrays.asList(
            State.MOVE_TO_SCORING_POSITION, // Move from starting position to scoring position
            State.LAUNCH, // Score preloaded artifacts
            State.MOVE_TO_GPP, // Move in front of the PPG artifact row
            State.INTAKE_ARTIFACT_ROW, // Intake the artifacts from the GPP row
            State.MOVE_TO_SCORING_POSITION,
            State.LAUNCH,
            State.MOVE_TO_PGP,
            State.INTAKE_ARTIFACT_ROW,
            State.MOVE_TO_PGP,
            State.MOVE_TO_SCORING_POSITION,
            State.LAUNCH,
            State.MOVE_TO_PPG,
            State.INTAKE_ARTIFACT_ROW,
            State.MOVE_TO_SCORING_POSITION,
            State.LAUNCH,
            State.MOVE_TO_GATE_ZONE
            State.COMPLETED // End of autonomous
    ));
     */
    
    /* // Move off the line auto
    final List<State> stateList = new ArrayList<>(Arrays.asList(
            State.MOVE_OFF_LAUNCH_LINE, // Move off the launch line
            State.COMPLETED // End of autonomous
    ));
     */

    // Utilities
    private final ElapsedTime runtime = new ElapsedTime(); // Runtime elapsed timer
    private StateMachine stateMachine; // Custom autonomous state machine
    private Robot robot; // Robot object

    // Other variables
    private Alliance alliance; // Alliance of the robot
    private StartPosition startPosition; // Starting position selected by the user
    private Pose currentPose; // Current pose of the robot
    private Pose lastCommandedPose; // Last pose that was commanded to the drivetrain
    private State pathState; // Current state machine value

    @Override
    public void runOpMode() {
        // Initialize robot
        robot = new Robot(hardwareMap, Mode.AUTO);
        stateMachine = new StateMachine();

        // Prompt the driver to select an alliance and start position
        alliance = robot.selectAlliance(gamepad1, telemetry);
        startPosition = robot.selectStartPosition(gamepad1, telemetry);

        // Build poses based on alliance
        robot.buildPoses(alliance);

        // Set variables based on start position
        if (startPosition == StartPosition.GOAL_WALL) {
            robot.drivetrain.setPose(robot.poses.goalStart); // Set starting pose
            lastCommandedPose = robot.poses.goalStart; // Set last commanded pose
        } else {
            robot.drivetrain.setPose(robot.poses.audienceStart);
            lastCommandedPose = robot.poses.audienceStart;
        }
        robot.drivetrain.update(robot); // Update drivetrain to set starting pose

        // Log completed initialization
        telemetry.addData("Status", "Initialized");
        telemetry.addData("Alliance", alliance);
        telemetry.addData("Start Position", startPosition);
        telemetry.update();

        // Wait for the game to start (driver presses START)
        waitForStart();

        // Reset runtime timer
        runtime.reset();

        while (opModeIsActive()) {
            if (runtime.milliseconds() < AUTO_START_DELAY) {
                // Wait for the auto start delay to finish
                telemetry.addData("Status", "Waiting to start...");
                telemetry.update();
                continue;
            }

            // Update robot and current pose
            robot.update(gamepad1, gamepad2);
            currentPose = robot.drivetrain.getPose();

            // Update state machine
            pathState = stateMachine.update();

            // Log status
            telemetry.addData("Elapsed", runtime.toString());
            telemetry.addData("Path State", pathState);
            telemetry.addData("Path Index", stateMachine.statesIndex);
            telemetry.addData("Launcher State", robot.launcher.getState());
            telemetry.addData("Launcher Left RPM", robot.launcher.getLeftRPM(robot));
            telemetry.addData("Launcher Right RPM", robot.launcher.getRightRPM(robot));
            telemetry.addData("Tapper State", robot.tapper.getState());
            telemetry.addData("Intake State", robot.intake.getState());
            telemetry.addData("X", currentPose.getX());
            telemetry.addData("Y", currentPose.getY());
            telemetry.addData("Heading", currentPose.getHeading());
            telemetry.update();
        }

        // OpMode is ending, stop the robot
        stateMachine.stop();
        robot.stop(gamepad1, gamepad2);

        // Save values for TeleOp
        blackboard.put("alliance", alliance);
        blackboard.put("autoEndPose", currentPose);
    }

    public enum State {
        LAUNCH, // Launch 3 artifacts
        MOVE_TO_SCORING_POSITION, // Move from the current position to the scoring position
        MOVE_TO_PPG, // Move in front of the PPG artifact row
        MOVE_TO_PGP, // Move in front of the PGP artifact row
        MOVE_TO_GPP, // Move in front of the GPP artifact row
        INTAKE_ARTIFACT_ROW, // Intake the artifacts from the current artifact row (move to the PPG/PGP/GPP end position with intake running)
        MOVE_TO_GATE_ZONE, // Move in front of the gate zone to prepare for TeleOp
        MOVE_OFF_LAUNCH_LINE, // Move off the launch line (only used when not doing full auto)
        COMPLETED // Autonomous path is completed
    }

    class StateMachine {
        private int statesIndex; // Current index in states
        private State currentState; // Current state (only used in update for cleaner code)
        private boolean launchCommanded; // Has the launch been commanded by the LAUNCH state
        private final ElapsedTime actionTimer = new ElapsedTime(); // Timer to wait before performing an action

        public StateMachine() {
            this.statesIndex = 0;
        }

        private void nextState() {
            statesIndex += 1;
            actionTimer.reset(); // Reset action timer for the next state to use if needed
        }

        public void stop() {
            if (currentState != State.COMPLETED) { // If not already completed
                statesIndex = stateList.size(); // Set index to end (COMPLETED state)
            }
        }

        public State update() {
            if (!robot.drivetrain.isDriving()) { // If the drivetrain is driving, don't run the state machine
                currentState = stateList.get(statesIndex);

                // State machine switch
                switch (currentState) {
                    case LAUNCH:
                        if (!this.launchCommanded) { // If the launcher hasn't been commanded
                            robot.launcher.launch(3); // Command launcher to launch 3 artifacts
                            this.launchCommanded = true; // Launch has been commanded
                        } else if (robot.launcher.didCompleteCycle()) { // If the launch cycle is completed
                            this.launchCommanded = false; // Reset launch commanded flag
                            nextState();
                        }
                        break;
                    case MOVE_TO_SCORING_POSITION:
                        robot.drivetrain.followPath(Poses.buildPath(robot.drivetrain, robot.poses.score));
                        lastCommandedPose = robot.poses.score;
                        nextState();
                        break;
                    case MOVE_TO_PPG:
                        robot.drivetrain.followPath(Poses.buildPath(robot.drivetrain, robot.poses.PPGArtifacts));
                        lastCommandedPose = robot.poses.PPGArtifacts;
                        nextState();
                        break;
                    case MOVE_TO_PGP:
                        robot.drivetrain.followPath(Poses.buildPath(robot.drivetrain, robot.poses.PGPArtifacts));
                        lastCommandedPose = robot.poses.PGPArtifacts;
                        nextState();
                        break;
                    case MOVE_TO_GPP:
                        robot.drivetrain.followPath(Poses.buildPath(robot.drivetrain, robot.poses.GPPArtifacts));
                        lastCommandedPose = robot.poses.GPPArtifacts;
                        nextState();
                        break;
                    case INTAKE_ARTIFACT_ROW:
                        // If we haven't already commanded the intake end position
                        if (lastCommandedPose == robot.poses.PPGArtifacts || lastCommandedPose == robot.poses.PGPArtifacts || lastCommandedPose == robot.poses.GPPArtifacts) {
                            robot.intake.run(); // Spin up the intake

                            // Determine which end position to go to based on the last commanded pose
                            Pose intakeEndPose;
                            if (lastCommandedPose.equals(robot.poses.PPGArtifacts)) {
                                intakeEndPose = robot.poses.PPGArtifactsEnd;
                            } else if (lastCommandedPose.equals(robot.poses.PGPArtifacts)) {
                                intakeEndPose = robot.poses.PGPArtifactsEnd;
                            } else {
                                intakeEndPose = robot.poses.GPPArtifactsEnd;
                            }
                            robot.drivetrain.followPath(Poses.buildPath(robot.drivetrain, intakeEndPose), 0.6);
                            lastCommandedPose = intakeEndPose;
                        } else { // We have reached the intake end position
                            robot.intake.stop(); // Stop the intake
                            nextState();
                        }
                        break;
                    case MOVE_TO_GATE_ZONE:
                        robot.drivetrain.followPath(Poses.buildPath(robot.drivetrain, robot.poses.gateZoneNotPushed), 0.9);
                        lastCommandedPose = robot.poses.gateZoneNotPushed;
                        nextState();
                        break;
                    case MOVE_OFF_LAUNCH_LINE:
                        if (startPosition == StartPosition.GOAL_WALL) {
                            lastCommandedPose = robot.poses.moveOffLineGoal;
                        } else {
                            lastCommandedPose = robot.poses.moveOffLineAudience;
                        }
                        robot.drivetrain.followPath(Poses.buildPath(robot.drivetrain, lastCommandedPose), 0.75);
                    case COMPLETED:
                        break;
                }
            }
            return currentState;
        }
    }
}