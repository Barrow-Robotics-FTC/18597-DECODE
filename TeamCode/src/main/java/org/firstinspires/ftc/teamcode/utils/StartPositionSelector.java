package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.robotcore.external.Telemetry;

// TODO: Revamp this file
// - Create poses here
// - More dynamic starting positions
// - Maybe some fancy logging
public class StartPositionSelector {
    public enum StartPositions {
        TOUCHING_CENTER_LINE,
        TOUCHING_OUTER_CENTER_LINE
    }

    public static StartPositions run(Gamepad gamepad1, Telemetry telemetry) {
        StartPositions selectedStartPosition = null;
        while (selectedStartPosition == null) {
            if (gamepad1.b) {
                selectedStartPosition = StartPositions.TOUCHING_CENTER_LINE;
            } else if (gamepad1.y) {
                selectedStartPosition = StartPositions.TOUCHING_OUTER_CENTER_LINE;
            }

            telemetry.addData("Start Position Selector", "Select Start Position");
            telemetry.addData("Touching Center Line", "Press B to select touching center line");
            telemetry.addData("Touching Outer Center Line", "Press Y to select touching outer center line");
            telemetry.update();
        }

        return selectedStartPosition;
    }
}
