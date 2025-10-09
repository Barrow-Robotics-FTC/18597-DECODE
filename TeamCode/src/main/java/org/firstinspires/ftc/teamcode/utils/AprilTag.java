package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.List;
import java.util.concurrent.TimeUnit;

@SuppressWarnings("FieldCanBeLocal") // Suppress pointless Android Studio warnings
public class AprilTag {
    final int APRIL_TAG_CAMERA_DECIMATION = 2; // Higher value = farther detection range, lower detection rate
    final int APRIL_TAG_CAMERA_EXPOSURE = 6; // Camera exposure time (milliseconds)
    final int APRIL_TAG_CAMERA_GAIN = 250; // Camera gain

    private static final int PPG_TAG_ID = 23; // Tag ID for PPG on the obelisk
    private static final int PGP_TAG_ID = 22; // Tag ID for PGP on the obelisk
    private static final int GPP_TAG_ID = 21; // Tag ID for GPP on the obelisk
    private VisionPortal visionPortal; // Manages webcam
    private AprilTagProcessor aprilTagProcessor; // Manages April tag detection

    public enum Pattern {
        PPG,
        PGP,
        GPP
    }

    private void sleep(int timeMS) {
        ElapsedTime timer = new ElapsedTime();
        while (timer.milliseconds() < timeMS) {
            assert true; // Do nothing
        }
    }

    public void init(HardwareMap hardwareMap) {
        // Create the AprilTag processor and set decimation
        aprilTagProcessor = new AprilTagProcessor.Builder().build();
        aprilTagProcessor.setDecimation(APRIL_TAG_CAMERA_DECIMATION);

        // Create the webcam handler (vision portal), attach the April tag processor
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTagProcessor)
                .build();

        // Wait for the camera to be open, then use the controls
        if (visionPortal == null) {
            return;
        }

        // Make sure camera is streaming before we try to set the exposure controls
        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            while ((visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
                sleep(20);
            }
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

    public Pattern detectPattern() {
        List<AprilTagDetection> currentDetections = aprilTagProcessor.getDetections();
        for (AprilTagDetection detection : currentDetections) {
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
