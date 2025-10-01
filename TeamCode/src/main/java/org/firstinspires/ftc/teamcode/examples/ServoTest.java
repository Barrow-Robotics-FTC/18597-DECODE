package org.firstinspires.ftc.teamcode.examples;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@TeleOp(name = "Servo Tests", group = "Tests")
public class ServoTest extends LinearOpMode {
    CRServo left_intake;
    CRServo right_intake;

    @Override
    public void runOpMode() {
        left_intake = hardwareMap.get(CRServo.class, "intake_left");
        right_intake = hardwareMap.get(CRServo.class, "intake_right");


        waitForStart();

        while (opModeIsActive()) {
            left_intake.setPower(1);
            right_intake.setPower(-1);
        }
    }
}
