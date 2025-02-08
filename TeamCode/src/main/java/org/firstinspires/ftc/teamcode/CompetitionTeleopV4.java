// Date: 2025-02-08
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.MovementLib.DriveWheels;

@TeleOp(name = "Competition TeleOp Unified", group = "Competition")
public class CompetitionTeleopUnified extends LinearOpMode {

    // Motors
    private DcMotor Front_Right, Front_Left, Back_Right, Back_Left, Slider;

    // Servos
    private Servo Arm, Arm_Lock, Claw, Wrist;

    // Constants
    private static final double ARM_DOWN_POSITION = 0.55;
    private static final double ARM_UP_POSITION = 0.31;
    private static final double CLAW_OPEN_POSITION = 0.65;
    private static final double CLAW_CLOSED_POSITION = 1;
    private static final double WRIST_VERTICAL_POSITION = 0;
    private static final double WRIST_HORIZONTAL_POSITION = 0.3;
    private static final int MAX_SLIDER_POS = 5000;

    // Movement Variables
    private final double joystickDeadzone = 0.1;
    private double robot_speed = 0.8;
    private double currentSpeed = 0.8;

    // Control Variables
    private int slider_position = 0;
    private boolean claw_open = false;
    private boolean wrist_rotated = false;

    // Emergency Stop Sequence
    private String controller1Sequence = "";
    private String controller2Sequence = "";
    private final String STOP_SEQUENCE_1 = "ABXY";
    private final String STOP_SEQUENCE_2 = "UDLR";

    @Override
    public void runOpMode() {
        initHardware();
        waitForStart();
        Unlock_Arm();

        while (opModeIsActive()) {
            if (checkStopSequence()) {
                stopProgram();
                return;
            }

            controlMovement();
            controlSlider();
            controlArm();
            toggleClaw();
            controlWrist();
            adjustSpeed();

            updateTelemetry();
        }
    }

    private void initHardware() {
        // Initialize Motors
        Front_Right = hardwareMap.get(DcMotor.class, "FrontRight");
        Front_Left = hardwareMap.get(DcMotor.class, "FrontLeft");
        Back_Right = hardwareMap.get(DcMotor.class, "BackRight");
        Back_Left = hardwareMap.get(DcMotor.class, "BackLeft");

        Front_Right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Front_Left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Back_Right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Back_Left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        Front_Right.setDirection(DcMotorSimple.Direction.REVERSE);
        Back_Right.setDirection(DcMotorSimple.Direction.REVERSE);

        // Drive System
        new DriveWheels(Front_Right, Front_Left, Back_Right, Back_Left);

        // Initialize Slider
        Slider = hardwareMap.get(DcMotor.class, "Slide Arm");
        Slider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Slider.setTargetPosition(0);
        Slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Slider.setPower(0);

        // Initialize Servos
        Arm = hardwareMap.get(Servo.class, "Arm");
        Arm_Lock = hardwareMap.get(Servo.class, "Arm Lock");
        Claw = hardwareMap.get(Servo.class, "Claw");
        Wrist = hardwareMap.get(Servo.class, "Wrist");
    }

    private void controlMovement() {
        double speedDifference = robot_speed - currentSpeed;
        if (Math.abs(speedDifference) > 0.01) {
            if (speedDifference > 0) currentSpeed += 0.07 * speedDifference;
            else currentSpeed += 0.15 * speedDifference;
        }

        DriveWheels.Omni_Move(
            applyDeadzone(gamepad1.left_stick_y + gamepad2.left_stick_y),
            applyDeadzone(gamepad1.left_stick_x + gamepad2.left_stick_x),
            applyDeadzone(gamepad1.right_stick_x + gamepad2.right_stick_x),
            currentSpeed
        );
    }

    private void controlSlider() {
        if (gamepad2.dpad_down) slider_position = 0;
        if (gamepad2.dpad_left) slider_position = 2201;
        if (gamepad2.dpad_up) slider_position = MAX_SLIDER_POS;

        slider_position = Math.min(MAX_SLIDER_POS, Math.max(0, slider_position));
        int error = Math.abs(slider_position - Slider.getCurrentPosition());
        double power = (error > 500) ? 1.0 : (error > 200) ? 0.5 : 0.25;
        Slider.setTargetPosition(slider_position);
        Slider.setPower(power);

        if (gamepad2.start) {
            Slider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            Slider.setTargetPosition(0);
            Slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
    }

    private void controlArm() {
        if (gamepad2.right_bumper) {
            Arm.setPosition(ARM_UP_POSITION - gamepad2.right_trigger / 5.0);
        } else {
            Arm.setPosition(ARM_DOWN_POSITION);
        }
    }

    private void toggleClaw() {
        if (gamepad2.b) claw_open = !claw_open;
        if (claw_open) Open_Claw();
        else Close_Claw();
    }

    private void controlWrist() {
        if (gamepad2.x) wrist_rotated = true;
        else if (gamepad2.y) wrist_rotated = false;

        if (wrist_rotated) Wrist_Horizontal();
        else Wrist_Vertical();
    }

    private void adjustSpeed() {
        if (!gamepad2.right_bumper && Slider.getCurrentPosition() < 100 && !gamepad1.left_bumper) {
            robot_speed = 0.5;
        }
        if (gamepad1.left_bumper) {
            robot_speed = 1;
        }
    }

    private void Lock_Arm() {
        Arm_Lock.setPosition(0);
    }

    private void Unlock_Arm() {
        Arm_Lock.setPosition(0.5);
    }

    private void Open_Claw() {
        Claw.setPosition(CLAW_OPEN_POSITION);
    }

    private void Close_Claw() {
        Claw.setPosition(CLAW_CLOSED_POSITION);
    }

    private void Wrist_Vertical() {
        Wrist.setPosition(WRIST_VERTICAL_POSITION);
    }

    private void Wrist_Horizontal() {
        Wrist.setPosition(WRIST_HORIZONTAL_POSITION);
    }

    private double applyDeadzone(double value) {
        return Math.abs(value) < joystickDeadzone ? 0 : value;
    }

    private boolean checkStopSequence() {
        updateController1Sequence();
        updateController2Sequence();
        return controller1Sequence.equals(STOP_SEQUENCE_1) && controller2Sequence.equals(STOP_SEQUENCE_2);
    }

    private void updateController1Sequence() {
        if (gamepad1.a) controller1Sequence += "A";
        if (gamepad1.b) controller1Sequence += "B";
        if (gamepad1.x) controller1Sequence += "X";
        if (gamepad1.y) controller1Sequence += "Y";
        if (controller1Sequence.length() > 4) controller1Sequence = controller1Sequence.substring(controller1Sequence.length() - 4);
    }

    private void updateController2Sequence() {
        if (gamepad2.dpad_up) controller2Sequence += "U";
        if (gamepad2.dpad_down) controller2Sequence += "D";
        if (gamepad2.dpad_left) controller2Sequence += "L";
        if (gamepad2.dpad_right) controller2Sequence += "R";
        if (controller2Sequence.length() > 4) controller2Sequence = controller2Sequence.substring(controller2Sequence.length() - 4);
    }

    private void updateTelemetry() {
        telemetry.addData("Slider Position", Slider.getCurrentPosition());
        telemetry.addData("Target Speed", robot_speed);
        telemetry.addData("Current Speed", currentSpeed);
        telemetry.update();
    }
}