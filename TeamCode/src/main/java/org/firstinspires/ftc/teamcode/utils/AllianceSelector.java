package org.firstinspires.ftc.teamcode.utils;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.hardware.Gamepad;

public class AllianceSelector {
    public enum Alliance {
        RED,
        BLUE
    }

    public static Alliance run(Gamepad gamepad1, TelemetryManager panelsTelemetry, Telemetry telemetry) {
        Alliance selected_alliance = null;
        while (selected_alliance == null) {
            if (gamepad1.b) {
                selected_alliance = Alliance.RED;
            } else if (gamepad1.y) {
                selected_alliance = Alliance.BLUE;
            }

            panelsTelemetry.debug("Alliance Selector", "Select Alliance");
            panelsTelemetry.debug("Red Alliance", "Press B to select red alliance");
            panelsTelemetry.debug("Blue Alliance", "Press Y to select blue alliance");
            panelsTelemetry.update(telemetry);
        }

        return selected_alliance;
    }
}
