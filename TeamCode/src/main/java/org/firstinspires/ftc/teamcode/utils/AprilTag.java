package org.firstinspires.ftc.teamcode.utils;

// FTC SDK
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import com.qualcomm.robotcore.util.Range;

// AprilTag constants
import static org.firstinspires.ftc.teamcode.utils.Constants.Pattern;
import static org.firstinspires.ftc.teamcode.utils.Constants.AprilTagConstants.*;

// Java
import java.util.Collection;
import java.util.Collections;
import java.util.concurrent.TimeUnit;

@SuppressWarnings("FieldCanBeLocal") // Suppress pointless Android Studio warnings
public class AprilTag {
    private final VisionPortal visionPortal; // Manages webcam
    private final AprilTagProcessor aprilTagProcessor; // Manages April tag detection
    private boolean cameraStreaming = true; // Is the camera currently streaming?

    // Used to wait for camera operations
    private void sleep(int timeMS) {
        try {
            Thread.sleep(timeMS);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }

    public AprilTag(HardwareMap hardwareMap) {
        // Create the AprilTag processor and set decimation
        aprilTagProcessor = new AprilTagProcessor.Builder().setDrawTagOutline(true).build();
        aprilTagProcessor.setDecimation(APRIL_TAG_CAMERA_DECIMATION);

        // Create the webcam handler (vision portal), attach the April tag processor
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
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
        exposureControl.setExposure(APRIL_TAG_CAMERA_EXPOSURE, TimeUnit.MILLISECONDS);
        sleep(20);

        // Set the gain
        GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
        gainControl.setGain(APRIL_TAG_CAMERA_GAIN);
        sleep(20);
    }

    private Collection<AprilTagDetection> getDetections() {
        return aprilTagProcessor.getDetections();
    }

    private void ensureCameraStreaming() {
        if (!cameraStreaming) {
            startCamera();
        }
    }

    /**
     * Restarts the camera streaming for April Tag detection if it was previously stopped.
     */
    public void startCamera() {
        visionPortal.resumeStreaming();
        cameraStreaming = true;
    }

    /**
     * Stops the camera streaming for April Tag detection to save resources.
     */
    public void stopCamera() {
        visionPortal.stopStreaming();
        cameraStreaming = false;
    }

    /**
     * Returns the first April Tag detection which has an ID in the provided targetIds list.
     *
     * @param targetIds list of tag IDs to match
     * @return the first matching detection if found, otherwise null
     */
    public AprilTagDetection getTag(Collection<Integer> targetIds) {
        ensureCameraStreaming(); // Ensure the camera is streaming
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
    public AprilTagDetection getTag(Integer targetId) {
        // Create a single index list and pass it to the other getTag method
        return this.getTag(Collections.singletonList(targetId));
    }

    /**
     * Returns the first April Tag detection which has an ID in the provided targetIDs list.
     *
     *
     * @param tag The detected tag to drive to
     * @param distance The desired distance from the tag in inches
     * @return movement vectors to drive to the tag
     */
    public Constants.MovementVectors driveToAprilTag(AprilTagDetection tag, double distance) {
        // Positional errors: forward/back, strafe, turn
        double rangeError = tag.ftcPose.range - distance; // Inches
        double yawError = tag.ftcPose.yaw; // Degrees (left/right)
        double headingError = tag.ftcPose.bearing; // Degrees (turn)

        // Check if position is within tolerances
        boolean rangeOk = Math.abs(rangeError) <= RANGE_ERROR_TOL;
        boolean yawOk = Math.abs(yawError) <= YAW_ERROR_TOL;
        boolean headingOk = Math.abs(headingError) <= HEADING_ERROR_TOL;
        boolean moveCompleted = rangeOk && yawOk && headingOk;

        // Calculate drive vectors
        double forward = Range.clip(rangeError * SPEED_GAIN, -MAX_FORWARD_SPEED, MAX_FORWARD_SPEED);
        double strafe = Range.clip(yawError * STRAFE_GAIN, -MAX_STRAFE_SPEED, MAX_STRAFE_SPEED);
        double turn = Range.clip(headingError * TURN_GAIN, -MAX_TURN_SPEED, MAX_TURN_SPEED);

        // Return movement vectors and completion status
        return new Constants.MovementVectors(forward, strafe, turn, moveCompleted);
    }

    /**
     * Returns the artifact scoring pattern by scanning for one of the 3 pattern April Tags on the obelisk
     *
     * @return the detected pattern if found, otherwise null
     */
    public Pattern detectPattern() {
        ensureCameraStreaming(); // Ensure the camera is streaming
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
        return Pattern.PPG;
    }
}