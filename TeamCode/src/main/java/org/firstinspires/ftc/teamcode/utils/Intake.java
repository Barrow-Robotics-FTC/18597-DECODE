package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake {
    final double INTAKE_POWER = 1.0;

    CRServo intake_left;
    CRServo intake_right;

    public void init(HardwareMap hardwareMap) {
        intake_left = hardwareMap.get(CRServo.class, "intake_left");
        intake_right = hardwareMap.get(CRServo.class, "intake_right");
    }

    public void run() {
        intake_left.setPower(INTAKE_POWER);
        intake_right.setPower(-INTAKE_POWER);
    }

    public void stop() {
        intake_left.setPower(0);
        intake_right.setPower(0);
    }
}
