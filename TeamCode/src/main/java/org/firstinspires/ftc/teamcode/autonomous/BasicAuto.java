package org.firstinspires.ftc.teamcode.autonomous;

// FTC SDK
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

// Panels
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.bylazar.telemetry.PanelsTelemetry;

// Pedro Pathing
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;
import com.pedropathing.geometry.Pose;

// Java
import java.util.Arrays;
import java.util.Objects;
import java.util.List;

@Autonomous(name = "Basic Autonomous", group = "Autonomous")
@Configurable // Panels
@SuppressWarnings("FieldCanBeLocal") // Stop Android Studio from bugging about variables being predefined
public class BasicAuto extends LinearOpMode {
    // Editable variables
    final int HUMAN_PLAYER_WAIT_TIME = 3000; // Time that the human player has to fill the hopper (milliseconds)
    List<StateMachine.State> stateList = Arrays.asList( // Add autonomous states for the state machine here
            StateMachine.State.HOME_TO_INTAKE,
            StateMachine.State.INTAKE,
            StateMachine.State.INTAKE_TO_SCORE,
            StateMachine.State.LAUNCH,
            StateMachine.State.SCORE_TO_LOAD,
            StateMachine.State.WAIT_FOR_HUMAN_PLAYER,
            StateMachine.State.LOAD_TO_SCORE,
            StateMachine.State.SCORE_TO_LOAD,
            StateMachine.State.WAIT_FOR_HUMAN_PLAYER,
            StateMachine.State.LOAD_TO_SCORE,
            StateMachine.State.SCORE_TO_HOME
    );

    // Initialize elapsed timer
    private final ElapsedTime runtime = new ElapsedTime();

    // Other variables
    private Pose currentPose; // Current pose of the robot
    public Follower follower; // Pedro Pathing follower
    private StateMachine stateMachine; // Custom autonomous state machine
    private TelemetryManager panelsTelemetry; // Panels telemetry
    private StateMachine.State pathState; // Current state machine value
    public static Pose autonomousEndPose = new Pose(72, 8, Math.toRadians(90)); // Ending pose in autonomous, will be edited at the end

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize Panels telemetry
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        // Initialize Pedro Pathing follower
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(Poses.home);

        // Create and initialize state machine
        stateMachine = new StateMachine();

        // Log completed initialization to Panels and driver station
        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry); // Update Panels and driver station after logging

        // Wait for the game to start (driver presses START)
        waitForStart();

        // Reset runtime timer
        runtime.reset();

        /*
        The April tag obelisk is randomized after the OpMode is initialized, so right after we run the OpMode
        we'll need to immediately scan the April Tag and then initialize our paths and state machine.

        For now the obelisk pattern is hardcoded until we have April Tag scanning capability.
         */
        Paths.build(follower, "PPG");
        stateMachine.init(follower, stateList, HUMAN_PLAYER_WAIT_TIME);

        while (opModeIsActive()) {
            // Update Pedro Pathing and Panels every iteration
            follower.update();
            panelsTelemetry.update();
            currentPose = follower.getPose(); // Update the current pose

            // Run the state machine update loop
            pathState = stateMachine.update();

            // Log status to Panels and driver station
            panelsTelemetry.debug("Elapsed", runtime.toString());
            panelsTelemetry.debug("Path State", pathState);
            panelsTelemetry.debug("X", currentPose.getX());
            panelsTelemetry.debug("Y", currentPose.getY());
            panelsTelemetry.debug("Heading", currentPose.getHeading());
            panelsTelemetry.update(telemetry); // Update Panels and driver station after logging
        }

        // Set the ending pose to be used in TeleOp
        follower.update();
        autonomousEndPose = follower.getPose();
    }

    static class Poses {
        // Poses
        public static Pose home = new Pose(72, 8, Math.toRadians(90)); // Centered against the audience wall
        public static Pose score = new Pose(60, 83.5, Math.toRadians(135)); // Facing goal (close to the white line point)
        public static Pose GPPArtifacts = new Pose(37, 121, Math.toRadians(0)); // Upper artifact pickup (closest to goal)
        public static Pose PGPArtifacts = new Pose(43, 130, Math.toRadians(0)); // Middle artifact pickup
        public static Pose PPGArtifacts = new Pose(41, 36, Math.toRadians(180)); // Lower artifact pickup (closest to audience wall)
        public static Pose loadZone = new Pose(12, 12, Math.toRadians(0)); // Loading zone

        // Control points (splines)
        public static Pose intakeToLoadMidpoint = new Pose(70, 30); // Control point between loadZone and score poses to avoid hitting artifacts
    }

    static class Paths {
        public static PathChain homeToIntake; // Poses.home -> Poses.XXXArtifacts
        public static PathChain intakeToScore; // Poses.XXXArtifact -> Poses.score
        public static PathChain scoreToLoad; // Poses.score -> Poses.loadZone
        public static PathChain loadToScore; // Poses.loadZone -> Poses.score
        public static PathChain scoreToHome; // Poses.score -> Poses.home

        public static void build(Follower follower, String pattern) {
            // Select the correct intake pose based on pattern
            Pose patternIntakePose;
            if (Objects.equals(pattern, "PPG")) {
                patternIntakePose = Poses.PPGArtifacts;
            } else if (Objects.equals(pattern, "PGP")) {
                patternIntakePose = Poses.PGPArtifacts;
            } else if (Objects.equals(pattern, "GPP")) {
                patternIntakePose = Poses.GPPArtifacts;
            } else {
                // Default to the closest one if the pattern is invalid for some reason
                patternIntakePose = Poses.PPGArtifacts;
            }

            homeToIntake = follower.pathBuilder()
                    .addPath(new BezierLine(Poses.home, patternIntakePose))
                    .setLinearHeadingInterpolation(Poses.home.getHeading(), patternIntakePose.getHeading())
                    .build();

            intakeToScore = follower.pathBuilder()
                    .addPath(new BezierLine(patternIntakePose, Poses.score))
                    .setLinearHeadingInterpolation(patternIntakePose.getHeading(), Poses.score.getHeading())
                    .build();

            scoreToLoad = follower.pathBuilder()
                    .addPath(new BezierCurve(Poses.score, Poses.intakeToLoadMidpoint, Poses.loadZone))
                    .setLinearHeadingInterpolation(Poses.score.getHeading(), Poses.loadZone.getHeading())
                    .build();

            loadToScore = follower.pathBuilder()
                    .addPath(new BezierCurve(Poses.loadZone, Poses.intakeToLoadMidpoint, Poses.score))
                    .setLinearHeadingInterpolation(Poses.loadZone.getHeading(), Poses.score.getHeading())
                    .build();

            scoreToHome = follower.pathBuilder()
                    .addPath(new BezierLine(Poses.score, Poses.home))
                    .setLinearHeadingInterpolation(Poses.score.getHeading(), Poses.home.getHeading())
                    .build();
        }
    }

    static class StateMachine {
        private Follower follower; // Pedro Pathing follower (passed in init)
        private List<State> states; // List of states provided in init
        private int statesIndex; // Current index in states
        private State currentState; // current state (only used in update for cleaner code)
        private int HUMAN_PLAYER_WAIT_TIME; // Time that the human player has to fill the hopper (milliseconds)
        private final ElapsedTime HUMAN_PLAYER_WAIT_TIMER = new ElapsedTime();

        public enum State {
            INTAKE,
            LAUNCH,
            WAIT_FOR_HUMAN_PLAYER,
            HOME_TO_INTAKE,
            INTAKE_TO_SCORE,
            SCORE_TO_LOAD,
            LOAD_TO_SCORE,
            SCORE_TO_HOME
        }

        private void nextState() {
            statesIndex += 1;
            if (states.get(statesIndex) == State.WAIT_FOR_HUMAN_PLAYER) {
                HUMAN_PLAYER_WAIT_TIMER.reset(); // Restart the timer that waits for the human player
            }
        }

        public void init(Follower pedro_follower, List<State> state_list, int human_player_wait_time) {
            follower = pedro_follower;
            states = state_list;
            statesIndex = 0;
            HUMAN_PLAYER_WAIT_TIME = human_player_wait_time;
        }

        public State update() {
            currentState = states.get(statesIndex);
            if (!follower.isBusy()) { // If the follower is running, don't run the state machine
                switch (currentState) {
                    case INTAKE:
                        // Put intake logic here
                        nextState();
                        break;
                    case LAUNCH:
                        // Put launch logic here
                        nextState();
                        break;
                    case WAIT_FOR_HUMAN_PLAYER:
                        // Note: timer is reset upon state change (above)
                        if (HUMAN_PLAYER_WAIT_TIMER.milliseconds() >= HUMAN_PLAYER_WAIT_TIME) {
                            nextState();
                        }
                        break;
                    case HOME_TO_INTAKE:
                        follower.followPath(Paths.homeToIntake);
                        nextState(); // Calling this after follower.followPath will wait until the follower is completed to run th next state
                        break;
                    case INTAKE_TO_SCORE:
                        follower.followPath(Paths.intakeToScore);
                        nextState();
                        break;
                    case SCORE_TO_LOAD:
                        follower.followPath(Paths.scoreToLoad);
                        nextState();
                        break;
                    case LOAD_TO_SCORE:
                        follower.followPath(Paths.loadToScore);
                        nextState();
                        break;
                    case SCORE_TO_HOME:
                        follower.followPath(Paths.scoreToHome);
                        nextState();
                        break;
                }
            }
            return currentState;
        }
    }
}