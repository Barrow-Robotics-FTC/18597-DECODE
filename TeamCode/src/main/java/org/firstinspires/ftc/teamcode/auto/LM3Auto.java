package org.firstinspires.ftc.teamcode.auto;

// FTC SDK
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

// Pedro Pathing
import com.pedropathing.geometry.Pose;

// Local helper files
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.Constants.Mode;
import org.firstinspires.ftc.teamcode.Constants.Alliance;
import org.firstinspires.ftc.teamcode.Constants.Pattern;
import org.firstinspires.ftc.teamcode.Constants.StartPosition;
import org.firstinspires.ftc.teamcode.Constants.Poses;

// Java
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

@Autonomous(name = "LM3 Auto", group = "Autonomous")
@SuppressWarnings("FieldCanBeLocal") // Suppress pointless Android Studio warnings
public class LM3Auto extends LinearOpMode {
    // Editable variables
    List<State> stateList = null; // This will be set to one of the following based on start position and then updated based on pattern
    final List<State> goalStartingStateList = new ArrayList<>(Arrays.asList(
            // Start of the autonomous states for goal starting position
            State.MOVE_TO_SCORING_POSITION,
            State.LAUNCH,
            State.MOVE_TO_PATTERN_SCANNING_POSITION,
            State.DETECT_PATTERN
            // The rest of the states will be added dynamically based on the detected pattern
    ));
    final List<State> audienceStartingStateList = new ArrayList<>(Arrays.asList(
            // Start of the autonomous states for audience wall starting position
            State.DETECT_PATTERN,
            State.MOVE_TO_SCORING_POSITION,
            State.LAUNCH
            // The rest of the states will be added dynamically based on the detected pattern
    ));

    // Utilities
    private final ElapsedTime runtime = new ElapsedTime(); // Runtime elapsed timer
    private StateMachine stateMachine; // Custom autonomous state machine
    private Robot robot; // Robot object

    // Other variables
    private Alliance alliance; // Alliance of the robot
    private Pattern targetPattern; // Target pattern determined by obelisk April Tag
    private StartPosition startPosition; // Starting position selected by the user
    private Pose currentPose; // Current pose of the robot
    private Pose lastCommandedPose; // Last pose that was commanded to the drivetrain
    private State pathState; // Current state machine value

    @Override
    public void runOpMode() {
        // Initialize robot and subsystems
        robot = new Robot(hardwareMap, Mode.AUTO);
        stateMachine = new StateMachine();

        // Prompt the driver to select an alliance and start position
        alliance = robot.selectAlliance(gamepad1, telemetry);
        startPosition = robot.selectStartPosition(gamepad1, telemetry);

        // Build poses based on alliance
        robot.buildPoses(alliance);

        // Set variables based on start position
        if (startPosition == StartPosition.GOAL_WALL) {
            robot.drivetrain.setStartingPose(robot.poses.goalStartPose); // Set starting pose
            lastCommandedPose = robot.poses.goalStartPose; // Set last commanded pose
            stateList = goalStartingStateList; // Set starting state list
        } else {
            robot.drivetrain.setStartingPose(robot.poses.audienceStartPose);
            lastCommandedPose = robot.poses.audienceStartPose;
            stateList = audienceStartingStateList;
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

        // Start launcher speed up
        stateMachine.speedUpLauncher();

        while (opModeIsActive()) {
            // Update robot and current pose
            robot.update();
            currentPose = robot.drivetrain.getPose();

            // Update state machine
            pathState = stateMachine.update();

            // If the state machine is complete or time is almost up, exit this loop
            if (pathState == State.COMPLETED || runtime.milliseconds() > 29000) {
                break;
            }

            // Log status
            telemetry.addData("Elapsed", runtime.toString());
            telemetry.addData("Path State", pathState);
            telemetry.addData("Path Index", stateMachine.statesIndex);
            telemetry.addData("Launcher State", robot.launcher.getState());
            telemetry.addData("X", currentPose.getX());
            telemetry.addData("Y", currentPose.getY());
            telemetry.addData("Heading", currentPose.getHeading());
            telemetry.update();
        }

        // OpMode is ending, stop the robot
        stateMachine.stop();
        robot.stop();

        // Save values for TeleOp
        blackboard.put("alliance", alliance);
        blackboard.put("autoEndPose", currentPose);
    }

    public enum State {
        DETECT_PATTERN, // Detect obelisk April Tag pattern, compute the rest of the autonomous states based on that
        LAUNCH, // Launch 3 artifacts
        MOVE_TO_SCORING_POSITION, // Move from the current position to the scoring position
        MOVE_TO_PATTERN_SCANNING_POSITION, // Move to the position to scan the April Tag (only used on the depot starting position, not the audience wall)
        MOVE_TO_PPG, // Move in front of the PPG artifact row
        MOVE_TO_PGP, // Move in front of the PGP artifact row
        MOVE_TO_GPP, // Move in front of the GPP artifact row
        INTAKE_ARTIFACT_ROW, // Intake the artifacts from the current artifact row (move to the PPG/PGP/GPP end position with intake running)
        MOVE_TO_GATE_ZONE, // Move in front of the gate zone to prepare for TeleOp
        COMPLETED // Autonomous path is completed
    }

    class StateMachine {
        private int statesIndex; // Current index in states
        private State currentState; // Current state (only used in update for cleaner code)
        private boolean launchCommanded; // Has the launch been commanded by the LAUNCH state

        private void nextState() {
            statesIndex += 1;
        }

        public StateMachine() {
            this.statesIndex = 0;
        }

        public void speedUpLauncher() {
            robot.launcher.speedUp();
        }

        public void stop() {
            if (currentState != State.COMPLETED) { // If not already completed
                statesIndex = stateList.size(); // Set index to end (COMPLETED state)
            }
        }

        private void appendAfterIntakeStates() {
            stateList.add(State.INTAKE_ARTIFACT_ROW); // Intake the row that the robot is lined up with
            stateList.add(State.MOVE_TO_SCORING_POSITION); // Move back to scoring position
            stateList.add(State.LAUNCH); // Launch the artifacts
            // Now we're ready for the next round of intakes or to finish
        }

        public State update() {
            if (!robot.drivetrain.isDriving()) { // If the drivetrain is driving, don't run the state machine
                currentState = stateList.get(statesIndex);

                // State machine switch
                switch (currentState) {
                    case DETECT_PATTERN:
                        // Based on the detected pattern, append the appropriate states to the state list
                        // NOTE: None of this is actively happening in this state, it's just laying out what will happen
                        targetPattern = robot.camera.detectPattern(); // Detect the April Tag pattern

                        // First round of intakes (pattern row)
                        if (targetPattern == Pattern.GPP) {
                            stateList.add(State.MOVE_TO_GPP);
                        } else if (targetPattern == Pattern.PGP) {
                            stateList.add(State.MOVE_TO_PGP);
                        } else if (targetPattern == Pattern.PPG) {
                            stateList.add(State.MOVE_TO_PPG);
                        }
                        appendAfterIntakeStates(); // Append the states for the first round of intakes

                        // Second round of intakes (closest to goal of the two, not pattern)
                        if (targetPattern == Pattern.GPP) {
                            stateList.add(State.MOVE_TO_PGP);
                        } else if (targetPattern == Pattern.PGP) {
                            stateList.add(State.MOVE_TO_GPP);
                        } else if (targetPattern == Pattern.PPG) {
                            stateList.add(State.MOVE_TO_GPP);
                        }
                        appendAfterIntakeStates(); // Append the states for the second round of intakes

                        // Last (third) round of intakes (not pattern row)
                        if (targetPattern == Pattern.GPP) {
                            stateList.add(State.MOVE_TO_PPG);
                        } else if (targetPattern == Pattern.PGP) {
                            stateList.add(State.MOVE_TO_PPG);
                        } else if (targetPattern == Pattern.PPG) {
                            stateList.add(State.MOVE_TO_PGP);
                        }
                        appendAfterIntakeStates(); // Append the states for the last round of intakes

                        stateList.add(State.MOVE_TO_GATE_ZONE); // Move to gate zone to prepare for TeleOp
                        stateList.add(State.COMPLETED);
                        nextState();

                        break;
                    case LAUNCH:
                        if (!this.launchCommanded) { // If the launcher hasn't been commanded
                            robot.launcher.launch(3); // Command launcher to launch 3 artifacts
                            this.launchCommanded = true; // Launch has been commanded
                        }
                        if (robot.launcher.didCompleteCycle()) { // If the launch cycle is completed
                            this.launchCommanded = false; // Reset launch commanded flag
                            nextState();
                        }
                        break;
                    case MOVE_TO_SCORING_POSITION:
                        if (!lastCommandedPose.equals(robot.poses.goalStartPose)) { // If the robot is at the goal start pose
                            // Use the control point specifically for goal start to scoring position
                            robot.drivetrain.followPath(Poses.buildPath(robot.drivetrain, robot.poses.score, new Pose[]{robot.poses.goalStartToScoreCP}));
                        } else {
                            robot.drivetrain.followPath(Poses.buildPath(robot.drivetrain, robot.poses.score, new Pose[]{robot.poses.toScoreCP}));
                        }

                        lastCommandedPose = robot.poses.score;
                        nextState();
                        break;
                    case MOVE_TO_PATTERN_SCANNING_POSITION:
                        robot.drivetrain.followPath(Poses.buildPath(robot.drivetrain, robot.poses.patternScanPosition));
                        lastCommandedPose = robot.poses.patternScanPosition;
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
                            robot.drivetrain.followPath(Poses.buildPath(robot.drivetrain, intakeEndPose), 0.4);
                            lastCommandedPose = intakeEndPose;
                        } else { // We have reached the intake end position
                            robot.intake.stop(); // Stop the intake
                            nextState();
                        }
                        break;
                    case MOVE_TO_GATE_ZONE:
                        robot.drivetrain.followPath(Poses.buildPath(robot.drivetrain, robot.poses.gateZoneNotPushed), 0.4);
                        lastCommandedPose = robot.poses.gateZoneNotPushed;
                        nextState();
                        break;
                    case COMPLETED:
                        break;
                }
            }
            return currentState;
        }
    }
}