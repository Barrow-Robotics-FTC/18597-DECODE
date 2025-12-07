package org.firstinspires.ftc.teamcode;

import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.subsystem.Drivetrain;

public class Constants {
    public enum Mode {
        AUTO,
        TELEOP
    }

    public enum StartPosition {
        GOAL_WALL,
        AUDIENCE_WALL
    }

    public enum Alliance {
        RED,
        BLUE
    }

    public enum Pattern {
        PPG,
        PGP,
        GPP
    }

    public static class PVSCoefficients {
        public double p;
        public double v;
        public double s;

        public PVSCoefficients(double kP, double kV, double kS) {
            this.p = kP;
            this.v = kV;
            this.s = kS;
        }
    }

    // Robot dimensions
    public static double LEFT_SIDE_TO_CENTER_DIST = 8.05; // Distance from the left side of the robot to the center (inches)
    public static double BACK_TO_CENTER_DIST = 8.4; // Distance from the back of the robot to the center (inches)

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

    public static class TapperConstants {
        public static double POSITIONING_TIME = 350; // Time it takes for the tapper to move between positions (milliseconds)
        public static double PUSHED_POSITION = 0.65; // Position that the tapper goes to when pushing an artifact into the launcher
        public static double HOME_POSITION = 0.1; // Position of the tapper when retracted
        public enum TapperState {
            IDLE_COMMANDED,
            IDLE,
            PUSHED_COMMANDED,
            PUSHED
        }
    }

    public static class LauncherConstants {
        public static PIDFCoefficients leftLauncherCoefficients = new PIDFCoefficients(85, 0, 80, 13.2);
        public static PIDFCoefficients rightLauncherCoefficients = new PIDFCoefficients(85, 0, 80, 13.2);
        public static PVSCoefficients leftLauncherPVSCoefficients = new PVSCoefficients(0.0, 0.0, 0.0);
        public static PVSCoefficients rightLauncherPVSCoefficients = new PVSCoefficients(0.0, 0.0, 0.0);
        public static int TARGET_RPM = 700; // Target RPM for both launcher motors
        public static int RPM_TOLERANCE = 40; // Launch RPM tolerance (must be within the range of target RPM +- tolerance)
        public static int RPM_IN_RANGE_TIME = 250; // Time that the RPM must be within the tolerance before launching (milliseconds)
        public static double POWER_WHILE_INTAKING = -0.1; // Power for launcher wheels while intaking to prevent jamming
        public enum LauncherState {
            IDLE,
            SPEED_UP,
            LAUNCH
        }
    }

    public static class IntakeConstants {
        public static double INTAKE_POWER = 1.0; // Power for intake motor
        public static double RAMP_HOLD_POSITION = 1.0; // Position for the intake ramp that holds artifacts in the storage area
        public static double RAMP_INTAKE_POSITION = 0.0; // Position for the intake ramp that allows the intake to put artifacts in the storage area
        public enum IntakeState {
            RUNNING,
            STOPPED
        }
    }

    public static class CameraConstants {
        public static int DECIMATION = 2; // Higher value = farther detection range, lower detection rate
        public static int EXPOSURE = 2; // Camera exposure time (milliseconds)
        public static int GAIN = 350; // Camera gain
    }

    public static class AprilTagConstants {
        public static final int BLUE_GOAL_TAG_ID = 20; // Tag ID for blue goal
        public static final int GPP_TAG_ID = 21; // Tag ID for GPP on the obelisk
        public static final int PGP_TAG_ID = 22; // Tag ID for PGP on the obelisk
        public static final int PPG_TAG_ID = 23; // Tag ID for PPG on the obelisk
        public static final int RED_GOAL_TAG_ID = 24; // Tag ID for red goal

        public static final double DISTANCE_FROM_APRIL_TAG = 52; // Distance to stop from April Tag when driving to it (inches)
        public static final double DRIVE_GAIN = 0.035; // Forward speed gain (for driving to April Tag)
        public static final double STRAFE_GAIN = 0.025; // Strafe speed gain (for driving to April Tag)
        public static final double TURN_GAIN = 0.023; // Turn speed gain (for driving to April Tag)
        public static final double MAX_FORWARD_SPEED = 0.25; // Max forward vector speed (for driving to April Tag)
        public static final double MAX_STRAFE_SPEED = 0.25; // Max strafe vector speed (for driving to April Tag)
        public static final double MAX_TURN_SPEED = 0.15; // Max turn vector speed (for driving to April Tag)
        public static final double DRIVE_ERROR_TOL = 2; // Inches
        public static final double STRAFE_ERROR_TOL = 2; // Degrees
        public static final double TURN_ERROR_TOL = 2; // Degrees
    }

    public static class TeleOpConstants {
        public static final boolean BRAKE_MODE = true; // Whether the motors should brake on stop (recommended)
        public static final boolean ROBOT_CENTRIC = true; // True for robot centric driving, false for field centric
        public static final double SLOW_MODE_MULTIPLIER = 0.3; // Multiplier for slow mode speed
        public static final double NORMAL_SPEED_MULTIPLIER = 0.8; // Multiplier for normal driving speed
    }

    // Pedro Pathing poses
    public static class Poses {
        // Whether to mirror poses (true for red alliance, false for blue alliance)
        boolean mirrorPoses;

        // Poses (assuming blue alliance)
        public Pose goalStart; // Starting pose that is perpendicular with and touching the ramp wall, as far forward as possible touching depot line
        public Pose audienceStart; // Starting pose that is against the audience wall, left edge on C tile left line
        public Pose localize; // Back against the audience and alliance walls, robot is driven into this corner for localization
        public Pose score; // Facing goal (close to the white line point)
        public Pose gateZoneNotPushed; // Right beside the gate, but not pushing it (facing away from audience)
        public Pose PPGArtifacts; // In front of upper artifacts
        public Pose PGPArtifacts; // In front of middle artifacts
        public Pose GPPArtifacts; // In front of lower artifacts
        public Pose PPGArtifactsEnd; // In front of PPGArtifacts
        public Pose PGPArtifactsEnd; // In front of PGPArtifacts
        public Pose GPPArtifactsEnd; // In front of GPPArtifacts

        public Poses(Alliance alliance) {
            // Set mirroring flag based on alliance
            this.mirrorPoses = (alliance == Alliance.RED);

            // Build the poses, see descriptions in definitions above
            this.goalStart = buildPose(15.25, 111, 144, mirrorPoses);
            this.audienceStart = buildPose(56, BACK_TO_CENTER_DIST, 90, mirrorPoses);
            this.localize = buildPose(LEFT_SIDE_TO_CENTER_DIST, BACK_TO_CENTER_DIST, 0, mirrorPoses);
            this.score = buildPose(67, 87, 138, mirrorPoses);
            this.gateZoneNotPushed = buildPose(25, 69, 90, mirrorPoses);
            this.PPGArtifacts = buildPose(55, 47.5, 180, mirrorPoses);
            this.PGPArtifacts = buildPose(55, 71.5, 180, mirrorPoses);
            this.GPPArtifacts = buildPose(55, 95.5, 180, mirrorPoses);
            this.PPGArtifactsEnd = buildPose(17, 47.5, 180, mirrorPoses);
            this.PGPArtifactsEnd = buildPose(17, 71.5, 180, mirrorPoses);
            this.GPPArtifactsEnd = buildPose(23, 95.5, 180, mirrorPoses);
        }

        // Suppress warning about mirrorIfNeeded always being true
        private static Pose buildPose(double x, double y, double heading, boolean mirror) {
            Pose pose = new Pose(x, y, Math.toRadians(heading));
            if (mirror) {
                pose = pose.mirror();
            }
            return pose;
        }

        /**
         * Build an ordered array of poses that starts with pose1, then the controlPoints (if any),
         * and ends with pose2.
         */
        private static Pose[] createCurvePoints(Pose pose1, Pose pose2, Pose[] controlPoints) {
            Pose[] result = new Pose[controlPoints.length + 2];
            result[0] = pose1;
            System.arraycopy(controlPoints, 0, result, 1, controlPoints.length);
            result[result.length - 1] = pose2;
            return result;
        }

        /**
         * Build a path chain between two poses with control points to make a bezier curve.
         *
         * @param drivetrain    Drivetrain object to build the path with
         * @param pose1         Starting pose of the path
         * @param pose2         Ending pose of the path
         * @param controlPoints Control points for the bezier curve
         * @return The built path chain
         */
        public static PathChain buildPath(Drivetrain drivetrain, Pose pose1, Pose pose2, Pose[] controlPoints) {
            return drivetrain.follower.pathBuilder()
                    .addPath(new BezierCurve(createCurvePoints(pose1, pose2, controlPoints)))
                    .setLinearHeadingInterpolation(pose1.getHeading(), pose2.getHeading())
                    .build();
        }

        /**
         * Build a path chain between two poses to make a bezier line.
         *
         * @param drivetrain Drivetrain object to build the path with
         * @param pose1 Starting pose of the path
         * @param pose2 Ending pose of the path
         * @return The built path chain
         */
        public static PathChain buildPath(Drivetrain drivetrain, Pose pose1, Pose pose2) {
            return drivetrain.follower.pathBuilder()
                    .addPath(new BezierLine(pose1, pose2))
                    .setLinearHeadingInterpolation(pose1.getHeading(), pose2.getHeading())
                    .build();
        }

        /**
         * Build a path chain from the current position to the target pose with control points to
         * make a bezier curve.
         *
         * @param drivetrain Drivetrain object to build the path with
         * @param pose Ending pose of the path
         * @param controlPoints Control points for the bezier curve
         * @return The built path chain
         */
        public static PathChain buildPath(Drivetrain drivetrain, Pose pose, Pose[] controlPoints) {
            return buildPath(drivetrain, drivetrain.getPose(), pose, controlPoints);
        }

        /**
         * Build a path chain from the current position to the target pose to make a bezier line.
         *
         * @param drivetrain Drivetrain object to build the path with
         * @param pose Ending pose of the path
         * @return The built path chain
         */
        public static PathChain buildPath(Drivetrain drivetrain, Pose pose) {
            return buildPath(drivetrain, drivetrain.getPose(), pose);
        }
    }
}