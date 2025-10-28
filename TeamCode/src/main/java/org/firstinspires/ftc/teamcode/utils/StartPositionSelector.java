package org.firstinspires.ftc.teamcode.utils;

// FTC SDK
import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.hardware.Gamepad;

// Pedro Pathing
import com.pedropathing.geometry.Pose;

// Start position selector utility
import static org.firstinspires.ftc.teamcode.utils.Constants.Poses;
import static org.firstinspires.ftc.teamcode.utils.Constants.StartPositionConstants.*;

public class StartPositionSelector {
    public static StartSelection run(Gamepad gamepad1, Telemetry telemetry, boolean mirrorPose) {
        StartPosition selectedStartPosition = null;
        Pose startPose = null;

        while (selectedStartPosition == null) {
            if (gamepad1.aWasPressed()) {
                selectedStartPosition = StartPosition.RIGHT_LINE_OF_C;
            } else if (gamepad1.yWasPressed()) {
                selectedStartPosition = StartPosition.LEFT_LINE_OF_C;
            } else if (gamepad1.bWasPressed()) {
                selectedStartPosition = StartPosition.CENTER_OF_LEFT_LINE_OF_C;
            }

            if (selectedStartPosition != null) { // Once a position in selected
                switch (selectedStartPosition) { // Create the Pose based on selection
                    case RIGHT_LINE_OF_C:
                        startPose = Poses.externalBuildPose(64, START_POSE_Y, START_POSE_HEADING, mirrorPose);
                        break;
                    case LEFT_LINE_OF_C:
                        startPose = Poses.externalBuildPose(56, START_POSE_Y, START_POSE_HEADING, mirrorPose);
                        break;
                    case CENTER_OF_LEFT_LINE_OF_C:
                        startPose = Poses.externalBuildPose(48, START_POSE_Y, START_POSE_HEADING, mirrorPose);
                        break;
                }
            }

            telemetry.addData("Start Position Selector", "Select Start Position");
            telemetry.addData("Press Cross (A)", "Right edge of robot touching the C tiles right line");
            telemetry.addData("Press Triangle (Y)", "Left edge of robot touching the C tiles left line");
            telemetry.addData("Press Circle (B)", "Center of robot over the C tiles left line");
            telemetry.update();
        }

        return new StartSelection(selectedStartPosition, startPose);
    }
}
