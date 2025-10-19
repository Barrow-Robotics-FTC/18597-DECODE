package org.firstinspires.ftc.teamcode.utils;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.robotcore.external.Telemetry;

// TODO: Revamp this file
// - Create poses here
// - More dynamic starting positions
// - Maybe some fancy logging
public class StartPositionSelector {
    public static Pose run(Gamepad gamepad1, Telemetry telemetry) {
        Pose selectedStartPosition = null;
        while (selectedStartPosition == null) {
            if (gamepad1.b) {
                selectedStartPosition = new Pose();
            } else if (gamepad1.y) {
                selectedStartPosition = new Pose();
            }

            telemetry.addData("Start Position Selector", "Select Start Position");
            telemetry.addData("Touching Center Line", "Press B to select touching center line");
            telemetry.addData("Touching Outer Center Line", "Press Y to select touching outer center line");
            telemetry.update();
        }

        return selectedStartPosition;
    }
}
