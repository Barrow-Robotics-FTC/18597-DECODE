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

public class Constants {
    public static class Pedro {
        public static FollowerConstants followerConstants = new FollowerConstants()
                .mass(18);

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

    public static class Paths {
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

        public static void build(Follower follower, AprilTag.Pattern pattern, AllianceSelector.Alliance alliance, StartPositionSelector.StartPositions startPosition) {
            // Set if poses should be mirrored based on alliance
            mirrorPoses = (alliance == AllianceSelector.Alliance.BLUE);

            // Poses (assuming red alliance, last arg determines if the pose should be mirrored on blue alliance)
            Pose home = getHome(startPosition); // Centered against the audience wall
            Pose score = buildPose(60, 83.5, 135); // Facing goal (close to the white line point)
            Pose PPGArtifacts = buildPose(40, 83.75, 180); // In front of upper artifacts
            Pose PGPArtifacts = buildPose(40, 59.75, 180); // In front of middle artifacts
            Pose GPPArtifacts = buildPose(40, 35.75, 180); // In front of lower artifacts
            Pose PPGArtifactsEnd = buildPose(20, 83.75, 180); // 20 inches in front of PPGArtifacts
            Pose PGPArtifactsEnd = buildPose(20, 59.75, 180); // 20 inches in front of PGPArtifacts
            Pose GPPArtifactsEnd = buildPose(20, 35.75, 180); // 20 inches in front of GPPArtifacts

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

        public static Pose getHome(StartPositionSelector.StartPositions startPosition) {
            if (startPosition == StartPositionSelector.StartPositions.TOUCHING_CENTER_LINE) {
                return buildPose(64, 7, 90);
            } else { // TOUCHING_OUTER_CENTER_LINE
                return buildPose(56, 7, 90);
            }
        }
    }
}