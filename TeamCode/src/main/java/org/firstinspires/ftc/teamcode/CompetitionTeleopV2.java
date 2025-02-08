package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.MovementLib.*;
@TeleOp(name = "Competition TeleOp", group = "Competition")
public class CompetitionTeleopV2 extends LinearOpMode {
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
        int arm_good = 0;
        boolean Arm_down = false;
        int Arm_pos = 0;
        int Arm_speed = 0;
        double robot_speed = .8;
        waitForStart();
        Unlock_Arm();
        while(opModeIsActive()) {
            Wheels.Omni_Move(gamepad1.left_stick_y + gamepad2.left_stick_y, gamepad1.left_stick_x + gamepad2.left_stick_x, gamepad1.right_stick_x + gamepad2.right_stick_x, robot_speed);
            if (gamepad2.dpad_down) {
                slider_position = 0;
            }
            if (gamepad2.dpad_left) {
                slider_position = 1;
            }
            if (gamepad2.dpad_up) {
                slider_position = 2;
            }
            if (slider_position == 0) {
                Slider.setTargetPosition(0);
                if (Slider.getCurrentPosition() < 100) {
                    Slider.setPower(0);
                    if (Slider.getCurrentPosition() < 30) {
                        Slider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        Slider.setTargetPosition(0);
                        Slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    }
                }
                else {
                    Slider.setPower(1);
                    robot_speed = .125;
                }
            }
            else if (slider_position == 1) {
                Slider.setTargetPosition(2201);
                Slider.setPower(1);
                robot_speed = .125;
                Arm_down = false;
            }
            else {
                Slider.setTargetPosition(5000);
                Slider.setPower(1);
                robot_speed = .125;
                Arm_down = false;
            }
            if(gamepad2.right_bumper) {
                robot_speed = .25;
                Arm.setPosition(0.31 - gamepad2.right_trigger / 5.0f);
            }
            else {
                Arm.setPosition(0.55);
            }
            if(gamepad2.a) { claw_open = false; }
            if(gamepad2.b) { claw_open = true; }
            if (gamepad2.x) {
                Wrist_Vertical();
            }
            else if (gamepad2.y){
                Wrist_Horizontal();
            }
            if (claw_open) {
                Open_Claw();
            }
            else {
                Close_Claw();
            }
            if (gamepad2.start) {
                Slider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                Slider.setTargetPosition(0);
                Slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
            if (gamepad1.back && gamepad2.back) {
                Slider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                Slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
            if (!gamepad2.right_bumper && Slider.getCurrentPosition() < 100 && !gamepad1.left_bumper) {
                robot_speed = .5;
            }
            if (gamepad1.left_bumper) {
                robot_speed = 1;
            }
            if (gamepad2.back) {
                Lock_Arm();
            }
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