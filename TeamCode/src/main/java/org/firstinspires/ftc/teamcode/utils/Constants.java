package org.firstinspires.ftc.teamcode.utils;

// FTC SDK
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

// Pedro Pathing
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

public class Constants {
    // Global
    public enum Alliance {
        RED,
        BLUE
    }
    public enum Pattern {
        PPG,
        PGP,
        GPP
    }

    // Omni movement vector class
    public static class MovementVectors {
        public double forward;
        public double strafe;
        public double turn;
        public Boolean moveCompleted;

        public MovementVectors(double forward, double strafe, double turn, Boolean moveCompleted) {
            this.forward = forward;
            this.strafe = strafe;
            this.turn = turn;
            this.moveCompleted = moveCompleted;
        }

        public MovementVectors(double forward, double strafe, double turn) {
            this(forward, strafe, turn, null);
        }
    }

    public static final double DISTANCE_FROM_APRIL_TAG = 64.7; // Distance to stop from April Tag when driving to it (inches)

    // Start position selector
    public static class StartPositionConstants {
        public static final double ROBOT_LENGTH = 14.0; // Robot length in inches
        public static final double START_POSE_Y = ROBOT_LENGTH / 2; // Y coordinate for start poses
        public static final double START_POSE_HEADING = 90.0; // Heading for start poses
        public enum StartPosition {
            RIGHT_LINE_OF_C, // Right edge of robot touching the C tiles right line
            LEFT_LINE_OF_C, // Left edge of robot touching the C tiles left line
            CENTER_OF_LEFT_LINE_OF_C // Center of robot over the C tiles left line
        }
        public static class StartSelection { // Return type for start position selector
            public final StartPosition startPosition;
            public final Pose pose;

            public StartSelection(StartPosition startPosition, Pose pose) {
                this.startPosition = startPosition;
                this.pose = pose;
            }
        }
    }

    // April Tags
    public static class AprilTagConstants {
        public static final int APRIL_TAG_CAMERA_DECIMATION = 2; // Higher value = farther detection range, lower detection rate
        public static final int APRIL_TAG_CAMERA_EXPOSURE = 6; // Camera exposure time (milliseconds)
        public static final int APRIL_TAG_CAMERA_GAIN = 250; // Camera gain
        public static final int BLUE_GOAL_TAG_ID = 20; // Tag ID for blue goal
        public static final int GPP_TAG_ID = 21; // Tag ID for GPP on the obelisk
        public static final int PGP_TAG_ID = 22; // Tag ID for PGP on the obelisk
        public static final int PPG_TAG_ID = 23; // Tag ID for PPG on the obelisk
        public static final int RED_GOAL_TAG_ID = 24; // Tag ID for red goal
        public static final double SPEED_GAIN = 0.02; // Forward speed gain (for driving to April Tag)
        public static final double STRAFE_GAIN = 0.015; // Strafe speed gain (for driving to April Tag)
        public static final double TURN_GAIN = 0.01; // Turn speed gain (for driving to April Tag)
        public static final double MAX_FORWARD_SPEED = 0.5; // Max forward vector speed (for driving to April Tag)
        public static final double MAX_STRAFE_SPEED = 0.5; // Max strafe vector speed (for driving to April Tag)
        public static final double MAX_TURN_SPEED = 0.3; // Max turn vector speed (for driving to April Tag)
        public static final double RANGE_ERROR_TOL = 1.0; // Inches
        public static final double YAW_ERROR_TOL = 1.5; // Degrees
        public static final double HEADING_ERROR_TOL = 2.0; // Degrees
    }

    // Intake
    public static class IntakeConstants {
        public static final double INTAKE_POWER = 1.0; // Power for intake servos
    }

    // Launcher
    public static class LauncherConstants {
        public static PIDFCoefficients LAUNCHER_PIDF_COEFFICIENTS = new PIDFCoefficients(300,0,0,10); // PIDF coefficients for launcher motors
        public static int TARGET_RPM = 900; // Target RPM for both launcher motors
        public static final int RPM_TOLERANCE = 100; // Launch RPM tolerance (must be within the range of target RPM +- tolerance)
        public static final int RPM_IN_RANGE_TIME = 200; // How long the launcher must be within the target RPM tolerance to launch (milliseconds)
        public static final int MIN_TIME_BETWEEN_LAUNCHES = 500; // Minimum time between launches (milliseconds)
        public static final int TAPPER_POSITIONING_TIME = 500; // Time to wait for the tapper to reach the pushed position (milliseconds)
        public static double TAPPER_PUSHED_POSITION = 0.65; // Position that the tapper goes to when pushing an artifact into the launcher
        public static final double TAPPER_HOME_POSITION = 0.1; // Position of the tapper when retracted
        public enum LauncherState {
            IDLE,
            SPEED_UP,
            LAUNCH
        }
        public static class LauncherReturnProps { // Props returned by Launcher.update()
            public final LauncherState state;
            public final boolean cycleCompleted;

            public LauncherReturnProps(LauncherState state, boolean cycleCompleted) {
                this.state = state;
                this.cycleCompleted = cycleCompleted;
            }
        }
    }

    public static class TeleOp {
        public static final boolean BRAKE_MODE = true; // Whether the motors should brake on stop (recommended)
        public static final boolean ROBOT_CENTRIC = true; // True for robot centric driving, false for field centric
        public static final double SLOW_MODE_MULTIPLIER = 0.5; // Multiplier for slow mode speed
        public static final double NORMAL_SPEED_MULTIPLIER = 1; // Multiplier for normal driving speed
    }

    // Pedro Pathing constants handler
    public static class Pedro {
        public static FollowerConstants followerConstants = new FollowerConstants()
                .mass(16);

        public static MecanumConstants driveConstants = new MecanumConstants()
                .maxPower(1)
                .rightFrontMotorName("front_right_drive")
                .rightRearMotorName("back_right_drive")
                .leftRearMotorName("back_left_drive")
                .leftFrontMotorName("front_left_drive")
                .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
                .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
                .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
                .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD);

        public static PinpointConstants localizerConstants = new PinpointConstants()
                .forwardPodY(4)
                .strafePodX(3)
                .distanceUnit(DistanceUnit.INCH)
                .hardwareMapName("pinpoint")
                .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
                .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
                .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD);

        public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);

        public static Follower createFollower(HardwareMap hardwareMap) {
            return new FollowerBuilder(followerConstants, hardwareMap)
                    .mecanumDrivetrain(driveConstants)
                    .pinpointLocalizer(localizerConstants)
                    .pathConstraints(pathConstraints)
                    .build();
        }
    }

    // Pedro Pathing poses
    public static class Poses {
        // Whether to mirror poses (true for red alliance, false for blue alliance)
        boolean mirrorPoses;

        // Poses (assuming blue alliance
        public Pose home; // Centered against the audience wall
        public Pose score; // Facing goal (close to the white line point)
        public Pose loadingZone; // In the loading zone (facing away from the human player)
        public Pose loadingZoneReverse; // In the loading zone (facing toward the human player))
        public Pose baseZone; // In the endgame zone (facing away from audience)
        public Pose PPGArtifacts; // In front of upper artifacts
        public Pose PGPArtifacts; // In front of middle artifacts
        public Pose GPPArtifacts; // In front of lower artifacts
        public Pose PPGArtifactsEnd; // 20 inches in front of PPGArtifacts
        public Pose PGPArtifactsEnd; // 20 inches in front of PGPArtifacts
        public Pose GPPArtifactsEnd; // 20 inches in front of GPPArtifacts

        public Poses(boolean mirrorPoses, Pose startPose) {
            // Set mirroring flag
            this.mirrorPoses = mirrorPoses;

            // Build the poses, see descriptions in definitions above
            // Optional last arg determines if the pose should be mirrored on red alliance)
            this.home = mirrorPoses ? startPose.mirror() : startPose;
            this.score = buildPose(60, 83.5, 135);
            this.loadingZone = buildPose(134, 10, 180);
            this.loadingZoneReverse = buildPose(134, 10, 0);
            this.baseZone = buildPose(106, 33, 90);
            this.PPGArtifacts = buildPose(104, 35.75, 0);
            this.PGPArtifacts = buildPose(104, 59.75, 0);
            this.GPPArtifacts = buildPose(104, 83.75, 0);
            this.PPGArtifactsEnd = buildPose(124, 35.75, 0);
            this.PGPArtifactsEnd = buildPose(124, 59.75, 0);
            this.GPPArtifactsEnd = buildPose(124, 83.75, 0);
        }

        public Pose[] getIntakePoses(Constants.Pattern pattern) {
            switch (pattern) {
                case PPG:
                    return new Pose[] {PPGArtifacts, PPGArtifactsEnd,
                            GPPArtifacts, GPPArtifactsEnd,
                            PGPArtifacts, PGPArtifactsEnd};
                case PGP:
                    return new Pose[] {PGPArtifacts, PGPArtifactsEnd,
                            GPPArtifacts, GPPArtifactsEnd,
                            PPGArtifacts, PPGArtifactsEnd};
                default: // GPP or unknown
                    return new Pose[] {GPPArtifacts, GPPArtifactsEnd,
                            PPGArtifacts, PPGArtifactsEnd,
                            PGPArtifacts, PGPArtifactsEnd};
            }
        }

        @SuppressWarnings("SameParameterValue") // Suppress warning about mirrorIfNeeded always being true
        private Pose buildPose(double x, double y, double heading, boolean mirrorIfNeeded) {
            Pose pose = new Pose(x, y, Math.toRadians(heading));
            if (this.mirrorPoses && mirrorIfNeeded) {
                pose = pose.mirror();
            }
            return pose;
        }

        private Pose buildPose(double x, double y, double heading) {
            return this.buildPose(x, y, heading, true);
        }

        @SuppressWarnings("SameParameterValue") // Suppress warning about heading always being a specific value
        static Pose externalBuildPose(double x, double y, double heading, boolean mirror) {
            Pose pose = new Pose(x, y, Math.toRadians(heading));
            if (mirror) {
                pose = pose.mirror();
            }
            return pose;
        }
    }

    // Pedro Pathing paths
    public static class Paths {
        // Poses for path building
        public Poses poses;

        // Cycle 1 (score preloaded)
        public PathChain homeToScore; // Poses.home -> Poses.score

        // Cycle 2 (intake pattern row and score)
        public PathChain scoreToPatternIntake; // score -> XXXArtifacts
        public PathChain patternIntakeToEnd; // XXXArtifacts -> XXXArtifactsEnd
        public PathChain patternIntakeEndToScore; // XXXArtifactsEnd -> score

        // Cycle 3 (intake first non-pattern row and score
        public PathChain scoreToNonPatternIntake1; // score -> XXXArtifacts
        public PathChain nonPatternIntake1ToEnd; // XXXArtifacts -> XXXArtifactsEnd
        public PathChain nonPatternIntake1EndToScore; // XXXArtifactsEnd -> score

        // Cycle 4 (intake second non-pattern row and score)
        public PathChain scoreToNonPatternIntake2; // score -> XXXArtifacts
        public PathChain nonPatternIntake2ToEnd; // XXXArtifacts -> XXXArtifactsEnd
        public PathChain nonPatternIntake2EndToScore; // XXXArtifactsEnd -> score

        // Temporary for LM1
        public PathChain scoreToPPGIntake; // score -> PPGArtifacts
        public PathChain PPGIntakeToEnd; // PPGArtifacts -> PPGArtifactsEnd
        public PathChain PPGIntakeEndToLoadingZoneReverse; // PPGArtifactsEnd -> loadingZoneReverse

        public void build(Follower follower, Constants.Pattern pattern, Constants.Alliance alliance, Pose startPosition) {
            // Build poses, mirror if red alliance
            this.poses = new Poses((alliance == Constants.Alliance.RED), startPosition);

            // Select the correct intake poses based on pattern
            Pose[] intakePoses = poses.getIntakePoses(pattern);
            Pose patternIntakePose = intakePoses[0];
            Pose patternIntakeEndPose = intakePoses[1];
            Pose nonPatternIntake1Pose = intakePoses[2];
            Pose nonPatternIntake1EndPose = intakePoses[3];
            Pose nonPatternIntake2Pose = intakePoses[4];
            Pose nonPatternIntake2EndPose = intakePoses[5];

            // Build paths
            this.homeToScore = buildPath(follower, poses.home, poses.score);
            this.scoreToPatternIntake = buildPath(follower, poses.score, patternIntakePose);
            this.patternIntakeToEnd = buildPath(follower, patternIntakePose, patternIntakeEndPose);
            this.patternIntakeEndToScore = buildPath(follower, patternIntakeEndPose, poses.score);
            this.scoreToNonPatternIntake1 = buildPath(follower, poses.score, nonPatternIntake1Pose);
            this.nonPatternIntake1ToEnd = buildPath(follower, nonPatternIntake1Pose, nonPatternIntake1EndPose);
            this.nonPatternIntake1EndToScore = buildPath(follower, nonPatternIntake1EndPose, poses.score);
            this.scoreToNonPatternIntake2 = buildPath(follower, poses.score, nonPatternIntake2Pose);
            this.nonPatternIntake2ToEnd = buildPath(follower, nonPatternIntake2Pose, nonPatternIntake2EndPose);
            this.nonPatternIntake2EndToScore = buildPath(follower, nonPatternIntake2EndPose, poses.score);

            // Temporary paths for LM1
            this.scoreToPPGIntake = buildPath(follower, poses.score, poses.PPGArtifacts);
            this.PPGIntakeToEnd = buildPath(follower, poses.PPGArtifacts, poses.PPGArtifactsEnd);
            this.PPGIntakeEndToLoadingZoneReverse = buildPath(follower, poses.PPGArtifactsEnd, poses.loadingZoneReverse);
        }

        public PathChain buildPath(Follower follower, Pose pose1, Pose pose2) {
            return follower.pathBuilder()
                    .addPath(new BezierLine(pose1, pose2))
                    .setLinearHeadingInterpolation(pose1.getHeading(), pose2.getHeading())
                    .build();
        }
    }
}