package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Constants.Alliance;
import org.firstinspires.ftc.teamcode.Constants.StartPosition;
import org.firstinspires.ftc.teamcode.Constants.Poses;
import org.firstinspires.ftc.teamcode.subsystem.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystem.Launcher;
import org.firstinspires.ftc.teamcode.subsystem.Tapper;
import org.firstinspires.ftc.teamcode.subsystem.Intake;
import org.firstinspires.ftc.teamcode.subsystem.Camera;

public class Robot {
    // Hardware
    public DcMotorEx leftLauncherMotor;
    public DcMotorEx rightLauncherMotor;
    public DcMotor intakeMotor;
    public Servo tapperServo;
    public Servo rampServo;
    public WebcamName webcam;

    // Subsystems
    public Drivetrain drivetrain;
    public Launcher launcher;
    public Tapper tapper;
    public Intake intake;
    public Camera camera;
    public Poses poses;

    // The SDK doesn't support "pressed" state for triggers, so we have to track it manually
    private boolean lastGamepad1RightTriggerPressed = false;
    private boolean lastGamepad1LeftTriggerPressed = false;
    private boolean lastGamepad2RightTriggerPressed = false;
    private boolean lastGamepad2LeftTriggerPressed = false;

    // Gamepad color handler
    private final int[] gamepad1Color = new int[]{0, 0, 0};
    private final int[] gamepad2Color = new int[]{0, 0, 0};

    // Other variables
    public Constants.Mode mode;

    public Robot(HardwareMap hardwareMap, Constants.Mode mode, boolean useVision) {
        this.mode = mode;

        // NOTE: Drivetrain motors and Pinpoint are initialized and set up by Pedro Pathing
        leftLauncherMotor = hardwareMap.get(DcMotorEx.class, "launcher_left");
        rightLauncherMotor = hardwareMap.get(DcMotorEx.class, "launcher_right");
        intakeMotor = hardwareMap.get(DcMotor.class, "intake");
        tapperServo = hardwareMap.get(Servo.class, "tapper");
        rampServo = hardwareMap.get(Servo.class, "ramp");
        webcam = hardwareMap.get(WebcamName.class, "Webcam 1");

        // Launcher motor configuration
        leftLauncherMotor.setZeroPowerBehavior(BRAKE);
        leftLauncherMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        rightLauncherMotor.setDirection(REVERSE); // Reverse right motor
        rightLauncherMotor.setZeroPowerBehavior(BRAKE);
        rightLauncherMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        // Intake motor configuration
        intakeMotor.setZeroPowerBehavior(BRAKE);
        intakeMotor.setDirection(REVERSE);

        // Initialize subsystems (subsystems take a Robot object as a parameter)
        drivetrain = new Drivetrain(this, hardwareMap); // Drivetrain needs HardwareMap for Pedro Pathing
        launcher = new Launcher(this);
        tapper = new Tapper(this);
        intake = new Intake(this);
        camera = useVision ? new Camera(this) : null; // Only initialize camera if vision is used
    }

    public Robot(HardwareMap hardwareMap, Constants.Mode mode) {
        this(hardwareMap, mode, false); // Default to not using vision
    }

    /**
     * Select the alliance for the match
     *
     * @param gamepad1 The gamepad to use for selection
     * @param telemetry The telemetry to use for displaying information
     * @return The selected alliance
     */
    public Alliance selectAlliance(Gamepad gamepad1, Telemetry telemetry) {
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

    /**
     * Select the starting position for Autonomous
     *
     * @param gamepad1 The gamepad to use for selection
     * @param telemetry The telemetry to use for displaying information
     * @return The selected starting position
     */
    public StartPosition selectStartPosition(Gamepad gamepad1, Telemetry telemetry) {
        StartPosition selectedPosition = null;
        while (selectedPosition == null) {
            if (gamepad1.bWasPressed()) {
                selectedPosition = StartPosition.GOAL_WALL;
            } else if (gamepad1.xWasPressed()) {
                selectedPosition = StartPosition.AUDIENCE_WALL;
            }

            telemetry.addData("Start Position Selector", "Select Start Position:");
            telemetry.addData("Goal/Ramp Wall", "Press Circle (B) to select the goal/depot pose");
            telemetry.addData("Audience Wall", "Press Square (X) to select the audience wall pose");
            telemetry.update();
        }

        return selectedPosition;
    }

    /**
     * Check if gamepad 1s right trigger was pressed (not just held down)
     * The trigger is considered pressed if its value goes above 0.5
     *
     * @param gamepad1 Gamepad 1
     * @return True if the right trigger was pressed, false otherwise
     */
    public boolean gamepad1RightTriggerPressed(Gamepad gamepad1) {
        return gamepad1.right_trigger > 0.5 && !lastGamepad1RightTriggerPressed;
    }

    /**
     * Check if gamepad 1s left trigger was pressed (not just held down)
     * The trigger is considered pressed if its value goes above 0.5
     *
     * @param gamepad1 Gamepad 1
     * @return True if the left trigger was pressed, false otherwise
     */
    public boolean gamepad1LeftTriggerPressed(Gamepad gamepad1) {
        return gamepad1.left_trigger > 0.5 && !lastGamepad1LeftTriggerPressed;
    }

    /**
     * Check if gamepad 2s right trigger was pressed (not just held down)
     * The trigger is considered pressed if its value goes above 0.5
     *
     * @param gamepad2 Gamepad 2
     * @return True if the right trigger was pressed, false otherwise
     */
    public boolean gamepad2RightTriggerPressed(Gamepad gamepad2) {
        return gamepad2.right_trigger > 0.5 && !lastGamepad2RightTriggerPressed;
    }

    /**
     * Check if gamepad 2s left trigger was pressed (not just held down)
     * The trigger is considered pressed if its value goes above 0.5
     *
     * @param gamepad2 Gamepad 2
     * @return True if the left trigger was pressed, false otherwise
     */
    public boolean gamepad2LeftTriggerPressed(Gamepad gamepad2) {
        return gamepad2.left_trigger > 0.5 && !lastGamepad2LeftTriggerPressed;
    }

    /**
     * Set the color of gamepad 1 LEDs
     *
     * @param r Red value (0-255)
     * @param g Green value (0-255)
     * @param b Blue value (0-255)
     */
    public void setGamepad1Color(int r, int g, int b) {
        gamepad1Color[0] = r;
        gamepad1Color[1] = g;
        gamepad1Color[2] = b;
    }

    /**
     * Set the color of gamepad 2 LEDs
     *
     * @param r Red value (0-255)
     * @param g Green value (0-255)
     * @param b Blue value (0-255)
     */
    public void setGamepad2Color(int r, int g, int b) {
        gamepad2Color[0] = r;
        gamepad2Color[1] = g;
        gamepad2Color[2] = b;
    }

    /**
     * Build poses based on the selected alliance
     * Must be used in initialization before using poses
     *
     * @param alliance The selected alliance
     */
    public void buildPoses(Alliance alliance) {
        poses = new Poses(alliance);
    }

    /**
     * Update all subsystems of the robot
     *
     * @param gamepad1 The first gamepad
     * @param gamepad2 The second gamepad
     */
    public void update(Gamepad gamepad1, Gamepad gamepad2) {
        // Update gamepad trigger states
        lastGamepad1RightTriggerPressed = gamepad1.right_trigger > 0.5;
        lastGamepad1LeftTriggerPressed = gamepad1.left_trigger > 0.5;
        lastGamepad2RightTriggerPressed = gamepad2.right_trigger > 0.5;
        lastGamepad2LeftTriggerPressed = gamepad2.left_trigger > 0.5;

        // Set gamepad colors (R, G, B, -1), here -1 means infinite duration
        gamepad1.setLedColor(gamepad1Color[0], gamepad1Color[1], gamepad1Color[2], -1);
        gamepad2.setLedColor(gamepad2Color[0], gamepad2Color[1], gamepad2Color[2], -1);

        // Update all subsystems
        drivetrain.update(this);
        launcher.update(this);
        tapper.update(this);
        intake.update(this);
        if (camera != null) { camera.update(this); } // Only update camera if it is being used
    }

    /**
     * Stop all subsystems of the robot
     *
     * @param gamepad1 The first gamepad
     * @param gamepad2 The second gamepad
     */
    public void stop(Gamepad gamepad1, Gamepad gamepad2) {
        // Stop all subsystems
        drivetrain.stop();
        launcher.stop();
        tapper.stop();
        intake.stop();
        if (camera != null) { camera.stop(); } // Only stop camera if it is being used

        // Update all subsystems to apply the stop commands
        update(gamepad1, gamepad2);
    }
}
