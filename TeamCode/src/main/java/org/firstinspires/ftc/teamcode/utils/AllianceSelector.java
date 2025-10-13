package org.firstinspires.ftc.teamcode.utils;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.hardware.Gamepad;

public class AllianceSelector {
    public enum Alliance {
        RED,
        BLUE
    }

    public static Alliance run(Gamepad gamepad1, Telemetry telemetry) {
        Alliance selectedAlliance = null;
        while (selectedAlliance == null) {
            if (gamepad1.b) {
                selectedAlliance = Alliance.RED;
            } else if (gamepad1.y) {
                selectedAlliance = Alliance.BLUE;
            }

            telemetry.addData("Alliance Selector", "Select Alliance");
            telemetry.addData("Red Alliance", "Press B to select red alliance");
            telemetry.addData("Blue Alliance", "Press Y to select blue alliance");
            telemetry.update();
        }

        return selectedAlliance;
    }
}
