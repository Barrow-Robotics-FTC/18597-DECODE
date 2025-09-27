package org.firstinspires.ftc.teamcode.examples;

// FTC SDK
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.util.ElapsedTime;

// Panels
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.bylazar.telemetry.PanelsTelemetry;

// Pedro Pathing
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;
import com.pedropathing.geometry.Pose;

@Autonomous(name = "Example Auto", group = "Examples")
@Disabled // REMOVE THI LINE TO SEE ON DRIVER HUB
@Configurable // Panels
@SuppressWarnings("FieldCanBeLocal") // Stop Android Studio from bugging about variables being predefined
public class AutoExample extends LinearOpMode {
    // Initialize elapsed timer
    private final ElapsedTime runtime = new ElapsedTime();

    // Initialize poses
    private final Pose startPose = new Pose(28.5, 128, Math.toRadians(180)); // Start Pose of our robot.
    private final Pose scorePose = new Pose(60, 85, Math.toRadians(135)); // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.
    private final Pose pickup1Pose = new Pose(37, 121, Math.toRadians(0)); // Highest (First Set) of Artifacts from the Spike Mark.
    private final Pose pickup2Pose = new Pose(43, 130, Math.toRadians(0)); // Middle (Second Set) of Artifacts from the Spike Mark.
    private final Pose pickup3Pose = new Pose(49, 135, Math.toRadians(0)); // Lowest (Third Set) of Artifacts from the Spike Mark.

    // Initialize variables for paths
    private PathChain scorePreload;
    private PathChain grabPickup1;
    private PathChain scorePickup1;
    private PathChain grabPickup2;
    private PathChain scorePickup2;
    private PathChain grabPickup3;
    private PathChain scorePickup3;
    private PathChain returnHome;

    // Other variables
    private Pose currentPose; // Current pose of the robot
    private Follower follower; // Pedro Pathing follower
    private TelemetryManager panelsTelemetry; // Panels telemetry
    private int pathState; // Current state machine value

    // Custom logging function to support telemetry and Panels
    private void log(String caption, Object... text) {
        if (text.length == 1) {
            telemetry.addData(caption, text[0]);
            panelsTelemetry.debug(caption + ": " + text[0]);
        } else if (text.length >= 2) {
            StringBuilder message = new StringBuilder();
            for (int i = 0; i < text.length; i++) {
                message.append(text[i]);
                if (i < text.length - 1) message.append(" ");
            }
            telemetry.addData(caption, message.toString());
            panelsTelemetry.debug(caption + ": " + message);
        }
    }

    public void intakeArtifacts() {
        // Put your intake logic here
    }

    public void shootArtifacts() {
        // Put your shooting logic here
    }

    @Override
    public void runOpMode() {
        // Initialize Panels telemetry
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        // Initialize Pedro Pathing follower
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);

        // Create paths
        buildPaths();

        // Log completed initialization to Panels and driver station (custom log function)
        log("Status", "Initialized");
        telemetry.update(); // Update driver station after logging

        // Wait for the game to start (driver presses START)
        waitForStart();
        runtime.reset();

        // Set path state to 0 and reset runtime timer after start
        setPathState(0);
        runtime.reset();

        while (opModeIsActive()) {
            // Update Pedro Pathing and Panels every iteration
            follower.update();
            panelsTelemetry.update();
            currentPose = follower.getPose(); // Update the current pose

            // Update the state machine
            updateStateMachine();

            // Log to Panels and driver station (custom log function)
            log("Elapsed", runtime.toString());
            log("Path State", pathState);
            log("X", currentPose.getX());
            log("Y", currentPose.getY());
            log("Heading", currentPose.getHeading());
            telemetry.update(); // Update the driver station after logging
        }
    }

    public void buildPaths() {
        // Move to the scoring pose from the start pose
        scorePreload = follower.pathBuilder()
                .addPath(new BezierLine(startPose, scorePose))
                .setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading())
                .build();

        // Move to the first artifact pickup pose from the scoring pose
        grabPickup1 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, pickup1Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup1Pose.getHeading())
                .build();

        // Move to the scoring pose from the first artifact pickup pose
        scorePickup1 = follower.pathBuilder()
                .addPath(new BezierLine(pickup1Pose, scorePose))
                .setLinearHeadingInterpolation(pickup1Pose.getHeading(), scorePose.getHeading())
                .build();

        // Move to the second artifact pickup pose from the scoring pose
        grabPickup2 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, pickup2Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup2Pose.getHeading())
                .build();

        // Move to the scoring pose from the second artifact pickup pose
        scorePickup2 = follower.pathBuilder()
                .addPath(new BezierLine(pickup2Pose, scorePose))
                .setLinearHeadingInterpolation(pickup2Pose.getHeading(), scorePose.getHeading())
                .build();

        // Move to the third artifact pickup pose from the scoring pose
        grabPickup3 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, pickup3Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup3Pose.getHeading())
                .build();

        // Move to the scoring pose from the third artifact pickup pose
        scorePickup3 = follower.pathBuilder()
                .addPath(new BezierLine(pickup3Pose, scorePose))
                .setLinearHeadingInterpolation(pickup3Pose.getHeading(), scorePose.getHeading())
                .build();

        // Move to the starting pose from the scoring pose
        returnHome = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, startPose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), startPose.getHeading())
                .build();
    }

    public void updateStateMachine() {
        switch (pathState) {
            case 0:
                // Move to the scoring position from the start position
                follower.followPath(scorePreload);
                setPathState(1); // Move to the second path state
                break;
            case 1:

                /* Ways to check for path completion:
                - Follower State: "if(!follower.isBusy()) {}"
                - Time: "if(pathTimer.getElapsedTimeSeconds() > 1) {}"
                - Robot Position: "if(follower.getPose().getX() > 36) {}"
                */

                // Wait until we have passed all path constraints
                if (!follower.isBusy()) {
                    // Shoot preloaded artifacts
                    shootArtifacts();

                    // Move to the first artifact pickup location from the scoring position
                    follower.followPath(grabPickup1, true);
                    setPathState(2); // Move to the third path state
                }
                break;
            case 2:
                // Wait until we have passed all path constraints
                if (!follower.isBusy()) {
                    // Intake first set of artifacts
                    intakeArtifacts();

                    follower.followPath(scorePickup1, true);
                    setPathState(3); // Move to the fourth path state
                }
                break;
            case 3:
                // Wait until we have passed all path constraints
                if (!follower.isBusy()) {
                    // Shoot first round of collected artifacts
                    shootArtifacts();

                    follower.followPath(grabPickup2, true);
                    setPathState(4);
                }
                break;
            case 4:
                // Wait until we have passed all path constraints
                if (!follower.isBusy()) {
                    // Intake second set of artifacts
                    intakeArtifacts();

                    follower.followPath(scorePickup2, true);
                    setPathState(5);
                }
                break;
            case 5:
                // Wait until we have passed all path constraints
                if (!follower.isBusy()) {
                    // Shoot second round of collected artifacts
                    shootArtifacts();

                    follower.followPath(grabPickup3, true);
                    setPathState(6);
                }
                break;
            case 6:
                // Wait until we have passed all path constraints
                if (!follower.isBusy()) {
                    // Intake third set of artifacts
                    intakeArtifacts();

                    follower.followPath(scorePickup3, true);
                    setPathState(7);
                }
                break;
            case 7:
                // Wait until we have passed all path constraints
                if (!follower.isBusy()) {
                    // Shoot third round of collected artifacts
                    shootArtifacts();

                    follower.followPath(returnHome, true);
                    setPathState(8); // Set the path state to 8
                }
                break;
            case 8:
                // Wait until we have passed all path constraints
                if (!follower.isBusy()) {
                    setPathState(-1); // Set the path state to -1 to stop execution
                }
                break;
        }
    }

    // Change path state
    public void setPathState(int pathState) {
        this.pathState = pathState;
    }
}