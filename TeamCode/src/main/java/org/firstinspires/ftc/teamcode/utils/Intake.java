package org.firstinspires.ftc.teamcode.utils;

// FTC SDK
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

// Intake constants
import static org.firstinspires.ftc.teamcode.utils.Constants.IntakeConstants.*;

public class Intake {
    CRServo intake_left;
    CRServo intake_right;

    public Intake(HardwareMap hardwareMap) {
        intake_left = hardwareMap.get(CRServo.class, "intake_left");
        intake_right = hardwareMap.get(CRServo.class, "intake_right");
    }

    public void run() {
        intake_left.setPower(INTAKE_POWER);
        intake_right.setPower(INTAKE_POWER);
    }

    public void stop() {
        intake_left.setPower(0);
        intake_right.setPower(0);
    }
}
