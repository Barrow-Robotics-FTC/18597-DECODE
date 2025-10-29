package org.firstinspires.ftc.teamcode.utils;

// FTC SDK
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

// Pedro Pathing
import com.pedropathing.geometry.Pose;

// Local helper files
import static org.firstinspires.ftc.teamcode.utils.Constants.PinpointUtilConstants.*;

public class Pinpoint {
    private final GoBildaPinpointDriver pinpoint;
    private boolean prevDrivingTo = false;
    private double headingBeforeDriveTo = 0.0;

    public Pinpoint(HardwareMap hardwareMap) {
        // Initialize Pinpoint using Pedro Pathing constants
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, Constants.Pedro.localizerConstants.hardwareMapName);
        //noinspection SuspiciousNameCombination (Apparently Pedro reverses offsets?)
        pinpoint.setOffsets(Constants.Pedro.localizerConstants.forwardPodY, Constants.Pedro.localizerConstants.strafePodX,
                Constants.Pedro.localizerConstants.distanceUnit);
        pinpoint.setEncoderResolution(Constants.Pedro.localizerConstants.encoderResolution);
        pinpoint.setEncoderDirections(Constants.Pedro.localizerConstants.forwardEncoderDirection,
                Constants.Pedro.localizerConstants.strafeEncoderDirection);
        pinpoint.resetPosAndIMU();
    }

    /**
     * Update the Pinpoint. Should be called in every iteration of your program
     */
    public void update() {
        pinpoint.update();
    }

    /**
     * Set the current position of the robot
     *
     * @param pose current pose of the robot
     */
    public void setPosition(Pose pose) {
        pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, pose.getX(), pose.getY(),
                AngleUnit.RADIANS, pose.getHeading()));
    }

    /**
     * Get the current position of the robot
     *
     * @return current pose of the robot
     */
    public Pose getPosition() {
        Pose2D currentPose = pinpoint.getPosition();
        return new Pose(currentPose.getX(DistanceUnit.INCH), currentPose.getY(DistanceUnit.INCH),
                currentPose.getHeading(AngleUnit.RADIANS));
    }

    /**
     * Calculates movement vectors needed to drive to a specific Pose.
     * .moveCompleted is true if the robot is at the pose
     *
     * @param targetPose target pose to drive to
     * @return movement vectors to drive to the tag
     */
    public Constants.MovementVectors driveTo(Pose targetPose) {
        // Get current position
        Pose currentPose = this.getPosition();

        // Positional errors in the world frame
        double xError = targetPose.getX() - currentPose.getX();
        double yError = targetPose.getY() - currentPose.getY();
        double turnError = targetPose.getHeading() - currentPose.getHeading(); // Radians

        // Rotate world errors into the robot's local frame
        double sin = Math.sin(-currentPose.getHeading());
        double cos = Math.cos(-currentPose.getHeading());
        double forwardError = xError * cos - yError * sin; // Inches
        double strafeError = xError * sin + yError * cos; // Inches

        // Calculate drive vectors
        double forward = Range.clip(forwardError * FORWARD_GAIN, -MAX_FORWARD_SPEED, MAX_FORWARD_SPEED);
        double strafe = Range.clip(strafeError * STRAFE_GAIN, -MAX_STRAFE_SPEED, MAX_STRAFE_SPEED);
        double turn = Range.clip(turnError * TURN_GAIN, -MAX_TURN_SPEED, MAX_TURN_SPEED);

        // Check if position is within tolerances (using non-rotated errors for world position check)
        boolean forwardOk = Math.abs(xError) <= FORWARD_ERROR_TOL;
        boolean strafeOk = Math.abs(yError) <= STRAFE_ERROR_TOL;
        boolean turnOk = Math.abs(turnError) <= TURN_ERROR_TOL;
        boolean moveCompleted = forwardOk && strafeOk && turnOk;

        // Return movement vectors and completion status
        return new Constants.MovementVectors(forward, strafe, turn, moveCompleted);
    }

    /**
     * Calculates movement vectors needed to drive to a specific Pose.
     * This method first corrects X and Y position, then corrects heading.
     * .moveCompleted is true if the robot is at the pose
     *
     * @param targetPose target pose to drive to
     * @return movement vectors to drive to the tag
     */
    public Constants.MovementVectors backupDriveTo(Pose targetPose) {
        // Get current position
        Pose currentPose = this.getPosition();

        if (!prevDrivingTo) {
            headingBeforeDriveTo = currentPose.getHeading();
        }

        // Positional errors for X and Y (field-centric)
        double xError = targetPose.getX() - currentPose.getX(); // Inches
        double yError = targetPose.getY() - currentPose.getY(); // Inches

        // Check if translational movement is complete
        boolean xOk = Math.abs(xError) <= FORWARD_ERROR_TOL;
        boolean yOk = Math.abs(yError) <= STRAFE_ERROR_TOL;
        boolean translationComplete = xOk && yOk;

        double forward;
        double strafe;
        double turn;
        boolean moveCompleted;

        if (!translationComplete) {
            // If X/Y position is not correct, focus on moving translationally
            forward = Range.clip(xError * FORWARD_GAIN, -MAX_FORWARD_SPEED, MAX_FORWARD_SPEED);
            strafe = Range.clip(yError * STRAFE_GAIN, -MAX_STRAFE_SPEED, MAX_STRAFE_SPEED);

            // Maintain original heading until translation is complete
            double turnError = headingBeforeDriveTo - currentPose.getHeading(); // Radians
            turn = Range.clip(turnError * TURN_GAIN, -MAX_TURN_SPEED, MAX_TURN_SPEED);

            moveCompleted = false; // The move is not complete until translation is done
        } else {
            // If X/Y position is correct, focus on turning
            forward = 0;
            strafe = 0;

            double turnError = targetPose.getHeading() - currentPose.getHeading(); // Radians
            turn = Range.clip(turnError * TURN_GAIN, -MAX_TURN_SPEED, MAX_TURN_SPEED);

            // The move is complete only if the turn is also within tolerance
            moveCompleted = Math.abs(turnError) <= TURN_ERROR_TOL;
        }

        // Update previous driving state
        prevDrivingTo = !moveCompleted;

        // Return movement vectors and completion status
        return new Constants.MovementVectors(forward, strafe, turn, moveCompleted);
    }
}