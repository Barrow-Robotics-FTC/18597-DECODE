package org.firstinspires.ftc.teamcode.subsystem;

// FTC SDK
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.VisionPortal;

// Pedro Pathing
import com.pedropathing.ftc.InvertedFTCCoordinates;
import com.pedropathing.geometry.PedroCoordinates;
import com.pedropathing.ftc.PoseConverter;
import com.pedropathing.geometry.Pose;

// Local helper files
import org.firstinspires.ftc.teamcode.Robot;
import static org.firstinspires.ftc.teamcode.Constants.Pattern;
import static org.firstinspires.ftc.teamcode.Constants.CameraConstants.*;
import static org.firstinspires.ftc.teamcode.Constants.AprilTagConstants.*;

// Java
import java.util.Collection;
import java.util.Collections;
import java.util.concurrent.TimeUnit;

public class Camera {
    private final AprilTagProcessor aprilTagProcessor; // Manages April tag detection

    // Used to wait for camera operations
    private void sleep(int timeMS) {
        try {
            Thread.sleep(timeMS);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }

    public Camera(Robot robot) {
        // Create the AprilTag processor and set decimation
        aprilTagProcessor = new AprilTagProcessor.Builder()
                .setDrawTagOutline(true) // Draw tag on camera stream
                .build();
        aprilTagProcessor.setDecimation(DECIMATION);

        // Create the webcam handler (vision portal), attach the April tag processor
        VisionPortal visionPortal = new VisionPortal.Builder()
                .setCamera(robot.webcam)
                .addProcessor(aprilTagProcessor)
                .build();

        // Make sure camera is streaming before we try to set the exposure controls
        while ((visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
            sleep(20);
        }

        // Enable manual exposure
        ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
        if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
            exposureControl.setMode(ExposureControl.Mode.Manual);
            sleep(50);
        }

        // Set the exposure
        exposureControl.setExposure(EXPOSURE, TimeUnit.MILLISECONDS);
        sleep(20);

        // Set the gain
        GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
        gainControl.setGain(GAIN);
        sleep(20);
    }

    private Collection<AprilTagDetection> getDetections() {
        return aprilTagProcessor.getDetections();
    }

    /**
     * Returns the first April Tag detection which has an ID in the provided targetIds list.
     *
     * @param targetIds list of tag IDs to match
     * @return the first matching detection if found, otherwise null
     */
    public AprilTagDetection getAprilTag(Collection<Integer> targetIds) {
        for (AprilTagDetection detection : getDetections()) {
            if (targetIds.contains(detection.id)) {
                return detection; // Return the first matching tag
            }
        }
        return null; // No target tag found
    }

    /**
     * Returns the first April Tag detection which has an ID in the provided targetIDs list.
     *
     * @param targetId The ID of the April Tag to match
     * @return the matching detection if found, otherwise null
     */
    public AprilTagDetection getAprilTag(Integer targetId) {
        // Create a single index list and pass it to the other getTag method
        return this.getAprilTag(Collections.singletonList(targetId));
    }

    /**
     * Get the robot pose from an April Tag detection (Pedro Pathing pose)
     *
     * @param tag The detected April Tag
     * @return The robot pose in Pedro Pathing format
     */
    public Pose getRobotPoseFromAprilTag(AprilTagDetection tag) {
        // Get Pedro Pathing robot coordinates from April Tag output
        Pose2D apriltagPose = new Pose2D(DistanceUnit.INCH, tag.robotPose.getPosition().x,
                tag.robotPose.getPosition().y, AngleUnit.RADIANS, tag.robotPose.getOrientation().getYaw(AngleUnit.RADIANS));
        Pose ftcStandard = PoseConverter.pose2DToPose(apriltagPose, InvertedFTCCoordinates.INSTANCE);
        return ftcStandard.getAsCoordinateSystem(PedroCoordinates.INSTANCE); // Pedro Pathing format pose
    }

    /**
     * Returns the artifact scoring pattern by scanning for one of the 3 pattern April Tags on the obelisk
     *
     * @return the detected pattern if found, otherwise it will default to GPP
     */
    public Pattern detectPattern() {
        for (AprilTagDetection detection : this.getDetections()) {
            if (detection.metadata != null) {
                // Check for pattern tags
                if (detection.id == PPG_TAG_ID) {
                    return Pattern.PPG;
                } else if (detection.id == PGP_TAG_ID) {
                    return Pattern.PGP;
                } else if (detection.id == GPP_TAG_ID) {
                    return Pattern.GPP;
                }
            }
        }

        // If the target wasn't found, assume PPG
        return Pattern.GPP;
    }

    public void stop() {
        // Nothing to stop (stopping the camera crashes the control hub???)
    }

    public void update(Robot robot) {
        // Nothing to update
    }
}
