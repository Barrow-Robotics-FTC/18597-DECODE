package org.firstinspires.ftc.teamcode.autonomous;

// FTC SDK
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

// Pedro Pathing
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;
import com.pedropathing.geometry.Pose;

// Local helper files
import org.firstinspires.ftc.teamcode.utils.Launcher;
import org.firstinspires.ftc.teamcode.utils.Intake;
import org.firstinspires.ftc.teamcode.utils.AllianceSelector;
import org.firstinspires.ftc.teamcode.utils.AprilTag;

// Java
import java.util.Arrays;
import java.util.List;

@Autonomous(name = "Basic Autonomous", group = "Autonomous")
@SuppressWarnings("FieldCanBeLocal") // Suppress pointless Android Studio warnings
public class BasicAuto extends LinearOpMode {
    // Editable variables
    List<StateMachine.State> stateList = Arrays.asList( // Add autonomous states for the state machine here
            StateMachine.State.HOME_TO_SCORE,
            StateMachine.State.LAUNCH,
            StateMachine.State.SCORE_TO_PATTERN_INTAKE,
            StateMachine.State.RUN_INTAKE,
            StateMachine.State.PATTERN_INTAKE_TO_END,
            StateMachine.State.STOP_INTAKE,
            StateMachine.State.PATTERN_INTAKE_END_TO_SCORE,
            StateMachine.State.LAUNCH,
            StateMachine.State.SCORE_TO_NON_PATTERN_INTAKE_1,
            StateMachine.State.RUN_INTAKE,
            StateMachine.State.NON_PATTERN_INTAKE_1_TO_END,
            StateMachine.State.STOP_INTAKE,
            StateMachine.State.NON_PATTERN_INTAKE_1_END_TO_SCORE,
            StateMachine.State.LAUNCH,
            StateMachine.State.SCORE_TO_NON_PATTERN_INTAKE_2,
            StateMachine.State.RUN_INTAKE,
            StateMachine.State.NON_PATTERN_INTAKE_2_TO_END,
            StateMachine.State.STOP_INTAKE,
            StateMachine.State.NON_PATTERN_INTAKE_2_END_TO_SCORE,
            StateMachine.State.LAUNCH,
            StateMachine.State.SCORE_TO_HOME
    );

    // Other variables
    private final ElapsedTime runtime = new ElapsedTime(); // Runtime elapsed timer
    private AllianceSelector.Alliance alliance; // Alliance of the robot
    private StateMachine stateMachine; // Custom autonomous state machine
    private Launcher launcher; // Custom launcher class
    private Intake intake; // Custom intake class
    private AprilTag aprilTag; // Custom April Tag class
    private Pose currentPose; // Current pose of the robot
    public Follower follower; // Pedro Pathing follower
    private StateMachine.State pathState; // Current state machine value
    private AprilTag.Pattern targetPattern; // Target pattern determined by obelisk April Tag

    @Override
    public void runOpMode() {
        // Initialize Pedro Pathing follower
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(Paths.getHome());

        // Initialize all utilities used in auto
        launcher = new Launcher(hardwareMap);
        intake = new Intake(hardwareMap);
        aprilTag = new AprilTag(hardwareMap);
        stateMachine = new StateMachine(follower, stateList, launcher, intake);

        // Prompt the driver to select an alliance
        alliance = AllianceSelector.run(gamepad1, telemetry);

        // Log completed initialization
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses START)
        waitForStart();

        // Reset runtime timer
        runtime.reset();

        /*
        The April tag obelisk is randomized after the OpMode is initialized, so right after we run the OpMode
        we'll need to immediately scan the April Tag and then initialize our poses and paths.
        */
        targetPattern = aprilTag.detectPattern();
        Paths.build(follower, targetPattern, alliance);

        while (opModeIsActive()) {
            // Update Pedro Pathing and Panels every iteration
            follower.update();
            currentPose = follower.getPose(); // Update the current pose

            // Run the state machine update loop
            pathState = stateMachine.update();

            // Log status
            telemetry.addData("Elapsed", runtime.toString());
            telemetry.addData("Path State", pathState);
            telemetry.addData("X", currentPose.getX());
            telemetry.addData("Y", currentPose.getY());
            telemetry.addData("Heading", currentPose.getHeading());
            telemetry.update();
        }

        // Save values for TeleOp
        blackboard.put("alliance", alliance);
        blackboard.put("autoEndPose", currentPose);
    }

    static class Paths {
        // Poses (assuming red alliance, last arg determines if the pose should be mirrored on blue alliance)
        private static Pose home; // Centered against the audience wall
        private static Pose score; // Facing goal (close to the white line point)
        private static Pose PPGArtifacts; // In front of upper artifacts
        private static Pose PGPArtifacts; // In front of middle artifacts
        private static Pose GPPArtifacts; // In front of lower artifacts
        private static Pose PPGArtifactsEnd; // 20 inches in front of PPGArtifacts
        private static Pose PGPArtifactsEnd; // 20 inches in front of PGPArtifacts
        private static Pose GPPArtifactsEnd; // 20 inches in front of GPPArtifacts

        // Blue alliance if true, red alliance if false
        private static boolean mirrorPoses;

        // Path chains
        // Cycle 1 (score preloaded)
        public static PathChain homeToScore; // Poses.home -> Poses.score

        // Cycle 2 (intake pattern row and score)
        public static PathChain scoreToPatternIntake; // score -> XXXArtifacts
        public static PathChain patternIntakeToEnd; // XXXArtifacts -> XXXArtifactsEnd
        public static PathChain patternIntakeEndToScore; // XXXArtifactsEnd -> score

        // Cycle 3 (intake first non-pattern row and score
        public static PathChain scoreToNonPatternIntake1; // score -> XXXArtifacts
        public static PathChain nonPatternIntake1ToEnd; // XXXArtifacts -> XXXArtifactsEnd
        public static PathChain nonPatternIntake1EndToScore; // XXXArtifactsEnd -> score

        // Cycle 4 (intake second non-pattern row and score)
        public static PathChain scoreToNonPatternIntake2; // score -> XXXArtifacts
        public static PathChain nonPatternIntake2ToEnd; // XXXArtifacts -> XXXArtifactsEnd
        public static PathChain nonPatternIntake2EndToScore; // XXXArtifactsEnd -> score

        // Back home
        public static PathChain scoreToHome; // score -> home

        public static void build(Follower follower, AprilTag.Pattern pattern, AllianceSelector.Alliance alliance) {
            mirrorPoses = (alliance == AllianceSelector.Alliance.BLUE); // Set if poses should be mirrored based on alliance

            // Define poses
            home = getHome();
            score = buildPose(60, 83.5, 135);
            PPGArtifacts = buildPose(40, 83.75, 180);
            PGPArtifacts = buildPose(40, 59.75, 180);
            GPPArtifacts = buildPose(40, 35.75, 180);
            PPGArtifactsEnd = buildPose(20, 83.75, 180);
            PGPArtifactsEnd = buildPose(20, 59.75, 180);
            GPPArtifactsEnd = buildPose(20, 35.75, 180);

            // Select the correct intake poses based on pattern (assume PPG initially)
            Pose patternIntakePose = PPGArtifacts;
            Pose nonPatternIntake1Pose = PGPArtifacts;
            Pose nonPatternIntake2Pose = GPPArtifacts;
            Pose patternIntakeEndPose = PPGArtifactsEnd;
            Pose nonPatternIntake1EndPose = PGPArtifactsEnd;
            Pose nonPatternIntake2EndPose = GPPArtifactsEnd;
            if (pattern == AprilTag.Pattern.PGP) { // If the pattern is PGP, swap PPG and PGP
                patternIntakePose = PGPArtifacts;
                nonPatternIntake1Pose = PPGArtifacts;
                patternIntakeEndPose = PGPArtifactsEnd;
                nonPatternIntake1EndPose = PPGArtifactsEnd;
            } else if (pattern == AprilTag.Pattern.GPP) { // If the pattern is GPP, rotate PPG, PGP, and GPP
                patternIntakePose = GPPArtifacts;
                nonPatternIntake1Pose = PPGArtifacts;
                nonPatternIntake2Pose = PGPArtifacts;
                patternIntakeEndPose = GPPArtifactsEnd;
                nonPatternIntake1EndPose = PPGArtifactsEnd;
                nonPatternIntake2EndPose = PGPArtifactsEnd;
            }

            homeToScore = buildPath(follower, home, score);
            scoreToPatternIntake = buildPath(follower, score, patternIntakePose);
            patternIntakeToEnd = buildPath(follower, patternIntakePose, patternIntakeEndPose);
            patternIntakeEndToScore = buildPath(follower, patternIntakeEndPose, score);
            scoreToNonPatternIntake1 = buildPath(follower, score, nonPatternIntake1Pose);
            nonPatternIntake1ToEnd = buildPath(follower, nonPatternIntake1Pose, nonPatternIntake1EndPose);
            nonPatternIntake1EndToScore = buildPath(follower, nonPatternIntake1EndPose, score);
            scoreToNonPatternIntake2 = buildPath(follower, score, nonPatternIntake2Pose);
            nonPatternIntake2ToEnd = buildPath(follower, nonPatternIntake2Pose, nonPatternIntake2EndPose);
            nonPatternIntake2EndToScore = buildPath(follower, nonPatternIntake2EndPose, score);
            scoreToHome = buildPath(follower, score, home);
        }

        @SuppressWarnings("SameParameterValue") // Suppress warning about mirrorIfNeeded always being true
        private static Pose buildPose(double x, double y, double heading, boolean mirrorIfNeeded) {
            Pose pose = new Pose(x, y, Math.toRadians(heading));
            if (mirrorPoses && mirrorIfNeeded) {
                pose = pose.mirror();
            }
            return pose;
        }

        private static Pose buildPose(double x, double y, double heading) {
            return buildPose(x, y, heading, true);
        }

        private static PathChain buildPath(Follower follower, Pose pose1, Pose pose2) {
            return follower.pathBuilder()
                    .addPath(new BezierLine(pose1, pose2))
                    .setLinearHeadingInterpolation(pose1.getHeading(), pose2.getHeading())
                    .build();
        }

        // TODO: CHANGE STARTING POSE
        public static Pose getHome() {
            return buildPose(72, 7, 90);
        }
    }

    static class StateMachine {
        private final Follower follower;
        private final List<State> states;
        private final Launcher launcher;
        private final Intake intake;
        private int statesIndex; // Current index in states
        private State currentState; // Current state (only used in update for cleaner code)

        public enum State {
            RUN_INTAKE,
            STOP_INTAKE,
            LAUNCH,
            HOME_TO_SCORE,
            SCORE_TO_PATTERN_INTAKE,
            PATTERN_INTAKE_TO_END,
            PATTERN_INTAKE_END_TO_SCORE,
            SCORE_TO_NON_PATTERN_INTAKE_1,
            NON_PATTERN_INTAKE_1_TO_END,
            NON_PATTERN_INTAKE_1_END_TO_SCORE,
            SCORE_TO_NON_PATTERN_INTAKE_2,
            NON_PATTERN_INTAKE_2_TO_END,
            NON_PATTERN_INTAKE_2_END_TO_SCORE,
            SCORE_TO_HOME
        }

        private void nextState() {
            statesIndex += 1;
        }

        public StateMachine(Follower pedro_follower, List<State> state_list, Launcher launcher_instance, Intake intake_instance) {
            follower = pedro_follower;
            launcher = launcher_instance;
            intake = intake_instance;
            states = state_list;
            statesIndex = 0;
        }

        public State update() {
            currentState = states.get(statesIndex);
            if (!follower.isBusy()) { // If the follower is running, don't run the state machine
                switch (currentState) {
                    case RUN_INTAKE:
                        intake.run();
                        nextState();
                        break;
                    case STOP_INTAKE:
                        intake.stop();
                        nextState();
                        break;
                    case LAUNCH:
                        /*
                        launcher.update() will run the launcher state machine to launch 3 artifacts.
                        The state will become IDLE when all 3 artifacts are launched.
                         */
                        if (launcher.update() == Launcher.State.IDLE) {
                            nextState();
                        }
                        break;
                    case HOME_TO_SCORE:
                        follower.followPath(Paths.homeToScore);
                        nextState();
                        break;
                    case SCORE_TO_PATTERN_INTAKE:
                        follower.followPath(Paths.scoreToPatternIntake);
                        nextState();
                        break;
                    case PATTERN_INTAKE_TO_END:
                        follower.followPath(Paths.patternIntakeToEnd);
                        nextState();
                        break;
                    case PATTERN_INTAKE_END_TO_SCORE:
                        follower.followPath(Paths.patternIntakeEndToScore);
                        nextState();
                        break;
                    case SCORE_TO_NON_PATTERN_INTAKE_1:
                        follower.followPath(Paths.scoreToNonPatternIntake1);
                        nextState();
                        break;
                    case NON_PATTERN_INTAKE_1_TO_END:
                        follower.followPath(Paths.nonPatternIntake1ToEnd);
                        nextState();
                        break;
                    case NON_PATTERN_INTAKE_1_END_TO_SCORE:
                        follower.followPath(Paths.nonPatternIntake1EndToScore);
                        nextState();
                        break;
                    case SCORE_TO_NON_PATTERN_INTAKE_2:
                        follower.followPath(Paths.scoreToNonPatternIntake2);
                        nextState();
                        break;
                    case NON_PATTERN_INTAKE_2_TO_END:
                        follower.followPath(Paths.nonPatternIntake2ToEnd);
                        nextState();
                        break;
                    case NON_PATTERN_INTAKE_2_END_TO_SCORE:
                        follower.followPath(Paths.nonPatternIntake2EndToScore);
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