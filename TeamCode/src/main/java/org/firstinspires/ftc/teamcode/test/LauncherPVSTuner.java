package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.Range; // Import for clipping power

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;

@TeleOp(name="Launcher PVS Tuner Fixed", group="Test")
public class LauncherPVSTuner extends LinearOpMode {
    private final double MAX_RPM = 1000.0;

    private final int TARGET_RPM = 695;

    // Gains start at 0.0
    private double kp = 0.0;
    private double kv = 0.0;
    private double ks = 0.0; // Start Ks at 0 for tuning, or keep your 0.003 if you know it overcomes friction

    @Override
    public void runOpMode() throws InterruptedException {
        TelemetryManager telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        DcMotorEx leftMotor = hardwareMap.get(DcMotorEx.class, "launcher_left");
        DcMotorEx rightMotor = hardwareMap.get(DcMotorEx.class, "launcher_right");

        // Optional: Reset encoders if you want to ensure clean velocity readings on start
        leftMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        leftMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

        while (opModeIsActive()) {
            /*
            Tuning Controls
            Note: Since we are normalizing, you might want slightly larger increments
            for Kp and Kv (e.g., 0.01 or 0.005) compared to the raw version.
             */
            if (gamepad1.triangleWasPressed()) { kp += 0.005; }
            if (gamepad1.crossWasPressed())   { kp -= 0.005; }
            if (gamepad1.circleWasPressed())  { kv += 0.005; }
            if (gamepad1.squareWasPressed())  { kv -= 0.005; }
            if (gamepad1.dpadUpWasPressed())  { ks += 0.001; }
            if (gamepad1.dpadDownWasPressed()){ ks -= 0.001; }

            // 1. GET CURRENT VELOCITY
            double leftVelocity = leftMotor.getVelocity();
            double rightVelocity = rightMotor.getVelocity();

            // 2. NORMALIZE (Convert RPM to 0.0 - 1.0 scale)
            // This is the key fix. We divide the Target and Current RPM by the Max RPM.
            double targetNorm = TARGET_RPM / MAX_RPM;
            double leftVelNorm = leftVelocity / MAX_RPM;
            double rightVelNorm = rightVelocity / MAX_RPM;

            /*
            3. CALCULATE POWER
            Formula: u = Kp * error + Kv * reference + Ks * sign
            We use the NORMALIZED values here.
            */

            // Calculate raw PVS output
            double leftPowerRaw = (kp * (targetNorm - leftVelNorm) + kv * targetNorm + ks * Math.signum(targetNorm));
            double rightPowerRaw = (kp * (targetNorm - rightVelNorm) + kv * targetNorm + ks * Math.signum(targetNorm));

            // 4. APPLY DIRECTION AND CLIP
            // Your original code inverted the left motor power. We apply that here.
            // We also use Range.clip to ensure the telemetry sees valid -1 to 1 numbers.
            double leftPower = Range.clip(-leftPowerRaw, -1.0, 1.0);
            double rightPower = Range.clip(rightPowerRaw, -1.0, 1.0);

            // Set motor powers
            leftMotor.setPower(leftPower);
            rightMotor.setPower(rightPower);

            // Panels telemetry for a graph
            telemetryM.addData("target", TARGET_RPM);
            telemetryM.addData("rpm", leftVelocity);
            telemetryM.update();

            // Driver station telemetry
            telemetry.addData("--- Power ---", "");
            telemetry.addData("Left Power", "%.2f", leftPower);
            telemetry.addData("Right Power", "%.2f", rightPower);

            telemetry.addData("--- Velocity ---", "");
            telemetry.addData("Target RPM", TARGET_RPM);
            telemetry.addData("Left RPM", "%.0f", leftVelocity);
            telemetry.addData("Right RPM", "%.0f", rightVelocity);

            telemetry.addData("--- Gains ---", "");
            telemetry.addData("Kp", "%.4f", kp);
            telemetry.addData("Kv", "%.4f", kv);
            telemetry.addData("Ks", "%.4f", ks);
            telemetry.update();
        }
    }
}