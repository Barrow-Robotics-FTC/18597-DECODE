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
import org.firstinspires.ftc.teamcode.utils.AllianceSelector;
import org.firstinspires.ftc.teamcode.utils.StartPositionSelector;
import org.firstinspires.ftc.teamcode.utils.AprilTag;
import org.firstinspires.ftc.teamcode.utils.Constants;

// Java
import java.util.Arrays;
import java.util.List;

@Autonomous(name = "LM1 Autonomous", group = "Autonomous")
@SuppressWarnings("FieldCanBeLocal") // Suppress pointless Android Studio warnings
public class LM1Auto extends LinearOpMode {
    // Editable variables
    final List<State> stateList = Arrays.asList( // Add autonomous states for the state machine here
            State.HOME_TO_SCORE, // Go to scoring pose
            State.LAUNCH, // Launch preloaded artifacts
            State.SCORE_TO_HOME // Park at home
            // TODO: DON'T GO BACK HOME, LOSS OF LEAVE POINTS
    );

    // Other variables
    private final ElapsedTime runtime = new ElapsedTime(); // Runtime elapsed timer
    private Constants.Alliance alliance; // Alliance of the robot
    private Constants.StartPositionConstants.StartSelection startPosition; // Start position of the robot
    private StateMachine stateMachine; // Custom autonomous state machine
    private Constants.Paths paths; // Custom paths class
    private Launcher launcher; // Custom launcher class
    private AprilTag aprilTag; // Custom April Tag class
    private Pose currentPose; // Current pose of the robot
    public Follower follower; // Pedro Pathing follower
    private State pathState; // Current state machine value
    private Constants.Pattern targetPattern; // Target pattern determined by obelisk April Tag
    private boolean holdingEndPoint = false; // Is the robot holding its end point?

    @Override
    public void runOpMode() {
        // Initialize Pedro Pathing follower
        follower = Constants.Pedro.createFollower(hardwareMap);

        // Initialize all utilities used in auto
        paths = new Constants.Paths();
        launcher = new Launcher(hardwareMap);
        aprilTag = new AprilTag(hardwareMap);

        // Prompt the driver to select an alliance
        alliance = AllianceSelector.run(gamepad1, telemetry);

        // Prompt user to select start position and set starting pose
        startPosition = StartPositionSelector.run(gamepad1, telemetry, (alliance == Constants.Alliance.BLUE));
        follower.setStartingPose(startPosition.pose);

        // Log completed initialization
        telemetry.addData("Status", "Initialized");
        telemetry.addData("Alliance", alliance);
        telemetry.addData("Start Position", startPosition.startPosition);
        telemetry.addData("Start Pose", startPosition.pose);
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

        // Build paths and initialize state machine with those paths
        paths.build(follower, targetPattern, alliance, startPosition.pose);
        stateMachine = new StateMachine();

        while (opModeIsActive()) {
            // Update Pedro Pathing and Panels every iteration
            follower.update();
            currentPose = follower.getPose(); // Update the current pose

            // Run the state machine update loop
            pathState = stateMachine.update();

            // If the state machine is complete or time is almost up, hold position to avoid penalties
            if (pathState == State.COMPLETED || runtime.milliseconds() > 28000) {
                if (!holdingEndPoint) { // If we aren't already holding the end point
                    follower.holdPoint(currentPose); // Hold position at the current pose
                    holdingEndPoint = true; // We are holding the end point
                }
            }

            // Log status
            telemetry.addData("Elapsed", runtime.toString());
            telemetry.addData("Path State", pathState);
            telemetry.addData("Path Index", stateMachine.statesIndex);
            telemetry.addData("X", currentPose.getX());
            telemetry.addData("Y", currentPose.getY());
            telemetry.addData("Heading", currentPose.getHeading());
            telemetry.update();
        }

        // OpMode is ending, stop all mechanisms
        stateMachine.stop(); // Stop the state machine
        follower.breakFollowing(); // Stop the position holding

        // Save values for TeleOp
        blackboard.put("alliance", alliance);
        blackboard.put("autoEndPose", currentPose);
        blackboard.put("paths", paths);
    }

    public enum State {
        LAUNCH,
        HOME_TO_SCORE,
        SCORE_TO_HOME,
        COMPLETED
    }

    class StateMachine {
        private int statesIndex; // Current index in states
        private State currentState; // Current state (only used in update for cleaner code)
        private boolean launchCommanded; // Has the launch been commanded by the LAUNCH state
        private boolean launcherActive; // Is the launcher currently active

        private void nextState() {
            statesIndex += 1;
        }

        public StateMachine() {
            this.statesIndex = 0;

            // Start launcher speed up
            launcher.speedUp();
            this.launcherActive = true; // Launcher is now active
        }

        public void stop() {
            if (currentState != State.COMPLETED) { // If not already completed
                statesIndex = stateList.size(); // Set index to end (COMPLETED state)
                update(); // Run update to stop mechanisms
            }
        }

        public State update() {
            if (!follower.isBusy()) { // If the follower is running, don't run the state machine
                // Handle out of bounds index
                if (statesIndex >= stateList.size()) {
                    currentState = State.COMPLETED;
                } else {
                    currentState = stateList.get(statesIndex);
                }

                // State machine switch
                switch (currentState) {
                    case LAUNCH:
                        if (!this.launchCommanded) { // Command the launcher to launch 3 artifacts
                            this.launchCommanded = true; // Launch has been commanded
                            launcher.launch(3); // Command launcher
                        }
                        if (launcher.update().cycleCompleted) {
                            this.launchCommanded = false; // Reset launch commanded flag
                            nextState();
                        }
                        break;
                    case HOME_TO_SCORE:
                        follower.followPath(paths.homeToScore);
                        nextState();
                        break;
                    case SCORE_TO_HOME:
                        follower.followPath(paths.scoreToHome);
                        nextState();
                        break;
                    case COMPLETED:
                        // If the launcher is currently active, stop it
                        if (this.launcherActive) {
                            launcher.stop();
                            this.launcherActive = false; // Launcher is no longer active
                        }
                        break;
                }
            }
            return currentState;
        }
    }
}