package org.firstinspires.ftc.teamcode.utils;

// FTC SDK
import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.hardware.Gamepad;

// Alliance constants
import org.firstinspires.ftc.teamcode.utils.Constants.Alliance;

public class AllianceSelector {
    public static Alliance run(Gamepad gamepad1, Telemetry telemetry) {
        Alliance selectedAlliance = null;
        while (selectedAlliance == null) {
            if (gamepad1.bWasPressed()) {
                selectedAlliance = Alliance.RED;
            } else if (gamepad1.xWasPressed()) {
                selectedAlliance = Alliance.BLUE;
            }

            telemetry.addData("Alliance Selector", "Select Alliance");
            telemetry.addData("Red Alliance", "Press Circle (B) to select red alliance");
            telemetry.addData("Blue Alliance", "Press Square (X) to select blue alliance");
            telemetry.update();
        }

        return selectedAlliance;
    }
}
