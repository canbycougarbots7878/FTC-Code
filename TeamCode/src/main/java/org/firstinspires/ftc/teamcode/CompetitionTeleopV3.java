package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.MovementLib.DriveWheels;
@Disabled
@TeleOp(name = "Competition TeleOp V3", group = "Competition")
public class CompetitionTeleopV3 extends LinearOpMode {
    int MAX_SLIDER_POS = 5000;
    double ARM_DOWN_POSITION = 0.21;
    double ARM_UP_POSITION = 0.55;
    Servo Arm = null;
    DcMotor Slider = null;
    Servo Arm_Lock = null;
    Servo Claw = null;
    Servo Wrist = null;
    @Override
    public void runOpMode() {
        // Initialize Motors
        DcMotor Front_Right = hardwareMap.get(DcMotor.class, "FrontRight");
        DcMotor Front_Left = hardwareMap.get(DcMotor.class, "FrontLeft");
        DcMotor Back_Right = hardwareMap.get(DcMotor.class, "BackRight");
        DcMotor Back_Left = hardwareMap.get(DcMotor.class, "BackLeft");
        Front_Right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Front_Left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Back_Right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Back_Left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        DriveWheels Wheels = new DriveWheels(Front_Right, Front_Left, Back_Right, Back_Left);
        Front_Right.setDirection(DcMotorSimple.Direction.REVERSE);
        Back_Right.setDirection(DcMotorSimple.Direction.REVERSE);
        // Initialize Slider
        Slider = hardwareMap.get(DcMotor.class, "Slide Arm");
        Slider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Slider.setTargetPosition(0);
        Slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Slider.setPower(0);
        // Initialize Arm
        Arm = hardwareMap.get(Servo.class, "Arm");
        // Initialize Arm Lock
        Arm_Lock = hardwareMap.get(Servo.class, "Arm Lock");
        // Initialize Claw
        Claw = hardwareMap.get(Servo.class, "Claw");
        Wrist = hardwareMap.get(Servo.class, "Wrist");
        // Variables
        int slider_position = 0;

        boolean claw_open = false;
        boolean arm_down;
        boolean wrist_rotated = false;

        double robot_speed = .8;


        waitForStart();
        Unlock_Arm();
        while(opModeIsActive()) {
            Wheels.Omni_Move(gamepad1.left_stick_y + gamepad2.left_stick_y, gamepad1.left_stick_x + gamepad2.left_stick_x, gamepad1.right_stick_x + gamepad2.right_stick_x, robot_speed);

            if (gamepad2.dpad_up) {
                slider_position += 1;
            }
            else if (gamepad2.dpad_down) {
                slider_position -= 1;
            }
            slider_position = Math.min(MAX_SLIDER_POS, Math.max(0, slider_position)); // Keep sliderpos in an acceptable range

            arm_down = gamepad2.right_bumper;

            if(gamepad2.a) { claw_open = false; }
            if(gamepad2.b) { claw_open = true; }

            if (gamepad2.x) { wrist_rotated = true; }
            else if (gamepad2.y){ wrist_rotated = false; }

            if (gamepad2.start) {
                Slider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                Slider.setTargetPosition(0);
                Slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
            if (!gamepad2.right_bumper && Slider.getCurrentPosition() < 100 && !gamepad1.left_bumper) {
                robot_speed = .5;
            }
            if (gamepad1.left_bumper) { robot_speed = 1; }
            if (gamepad2.back) { Lock_Arm(); }

                 if (arm_down) { Arm.setPosition(ARM_DOWN_POSITION); } //
                          else { Arm.setPosition(ARM_UP_POSITION); }
                if (claw_open) { Open_Claw(); }
                          else { Close_Claw(); }
            if (wrist_rotated) { Wrist_Horizontal(); }
                          else { Wrist_Vertical(); }
            if (slider_position < 100) { Slider.setPower(0); }
                                  else { Slider.setPower(1); }
            telemetry.addData("Slider Position", Slider.getCurrentPosition());
            telemetry.addData("Robot Speed", robot_speed);

            telemetry.update();
        }
    }
    private void Lock_Arm() {
        Arm_Lock.setPosition(0);
    }
    private void Unlock_Arm() {
        Arm_Lock.setPosition(0.5);
    }
    private void Open_Claw() {
        Claw.setPosition(0.65);
    }
    private void Close_Claw() {
        Claw.setPosition(1);
    }
    private void Wrist_Vertical() { Wrist.setPosition(0); }
    private void Wrist_Horizontal() { Wrist.setPosition((0.3)); }
}