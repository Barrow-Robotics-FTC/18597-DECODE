package org.firstinspires.ftc.teamcode.autonomous;

// FTC SDK
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

// Pedro Pathing
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;

// Local helper files
import org.firstinspires.ftc.teamcode.utils.Launcher;
import org.firstinspires.ftc.teamcode.utils.Intake;
import org.firstinspires.ftc.teamcode.utils.AllianceSelector;
import org.firstinspires.ftc.teamcode.utils.StartPositionSelector;
import org.firstinspires.ftc.teamcode.utils.AprilTag;
import org.firstinspires.ftc.teamcode.utils.Constants;

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
    private StartPositionSelector.StartPositions startPosition; // Start position of the robot (Not Pose)
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
        follower = Constants.Pedro.createFollower(hardwareMap);

        // Initialize all utilities used in auto
        launcher = new Launcher(hardwareMap);
        intake = new Intake(hardwareMap);
        aprilTag = new AprilTag(hardwareMap);
        stateMachine = new StateMachine(follower, stateList, launcher, intake);

        // Prompt the driver to select an alliance
        alliance = AllianceSelector.run(gamepad1, telemetry);

        // Prompt user to select start position and set starting pose
        startPosition = StartPositionSelector.run(gamepad1, telemetry);
        follower.setStartingPose(Constants.Paths.getHome(startPosition));

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
        Constants.Paths.build(follower, targetPattern, alliance, startPosition);

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
                        if (launcher.update(true) == Launcher.State.IDLE) {
                            nextState();
                        }
                        break;
                    case HOME_TO_SCORE:
                        follower.followPath(Constants.Paths.homeToScore);
                        nextState();
                        break;
                    case SCORE_TO_PATTERN_INTAKE:
                        follower.followPath(Constants.Paths.scoreToPatternIntake);
                        nextState();
                        break;
                    case PATTERN_INTAKE_TO_END:
                        follower.followPath(Constants.Paths.patternIntakeToEnd);
                        nextState();
                        break;
                    case PATTERN_INTAKE_END_TO_SCORE:
                        follower.followPath(Constants.Paths.patternIntakeEndToScore);
                        nextState();
                        break;
                    case SCORE_TO_NON_PATTERN_INTAKE_1:
                        follower.followPath(Constants.Paths.scoreToNonPatternIntake1);
                        nextState();
                        break;
                    case NON_PATTERN_INTAKE_1_TO_END:
                        follower.followPath(Constants.Paths.nonPatternIntake1ToEnd);
                        nextState();
                        break;
                    case NON_PATTERN_INTAKE_1_END_TO_SCORE:
                        follower.followPath(Constants.Paths.nonPatternIntake1EndToScore);
                        nextState();
                        break;
                    case SCORE_TO_NON_PATTERN_INTAKE_2:
                        follower.followPath(Constants.Paths.scoreToNonPatternIntake2);
                        nextState();
                        break;
                    case NON_PATTERN_INTAKE_2_TO_END:
                        follower.followPath(Constants.Paths.nonPatternIntake2ToEnd);
                        nextState();
                        break;
                    case NON_PATTERN_INTAKE_2_END_TO_SCORE:
                        follower.followPath(Constants.Paths.nonPatternIntake2EndToScore);
                        nextState();
                        break;
                    case SCORE_TO_HOME:
                        follower.followPath(Constants.Paths.scoreToHome);
                        nextState();
                        break;
                }
            }
            return currentState;
        }
    }
}