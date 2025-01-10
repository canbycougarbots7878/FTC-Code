package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.MovementLib.*;
@TeleOp(name = "Competition TeleOp", group = "Competition")
public class CompetitionTeleopV2 extends LinearOpMode {
    DcMotor Arm = null;
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
        Arm = hardwareMap.get(DcMotor.class, "Extending Arm");
        Arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Arm.setTargetPosition(0);
        Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Arm.setPower(0);
        // Initialize Arm Lock
        Arm_Lock = hardwareMap.get(Servo.class, "Arm Lock");
        // Initialize Claw
        Claw = hardwareMap.get(Servo.class, "Claw");
        //Wrist = hardwareMap.get(Servo.class, "Wrist");
        // Variables
        int slider_position = 0;
        int arm_position = 0;
        int last = 0;
        int armspeed = 0;
        double armpwr = 0;
        waitForStart();
        Arm_Lock.setPosition(.5);
        while(opModeIsActive()) {
            Wheels.Omni_Move(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x, 1);
            if (gamepad2.dpad_down) {
                slider_position = 0;
            }
            if (gamepad2.dpad_up) {
                slider_position = 1;
            }
            if (slider_position == 0) {
                Slider.setTargetPosition(0);
                if (Slider.getCurrentPosition() < 100) {
                    Slider.setPower(0);
                }
                else {
                    Slider.setPower(1);
                }
            }
            else {
                Slider.setTargetPosition(4500);
                Slider.setPower(1);
            }
            if (gamepad2.a) {
                Arm.setTargetPosition(0);
                if (Arm.getCurrentPosition() > 0) {
                    Arm.setPower(0);
                }
                else {
                    Arm.setPower(1);
                }
            }
            else if (gamepad2.b) {
                Arm.setTargetPosition(-50);
                Arm.setPower(1);
            }
            else {
                armpwr = 0;
            }
            Claw.setPosition(1 - gamepad2.right_trigger);
            telemetry.addData("arm position", Arm.getCurrentPosition());
            telemetry.addData("arm speed", armspeed);
            telemetry.addData("arm power", armpwr);
            telemetry.addData("slider position", Slider.getCurrentPosition());
            telemetry.update();
        }
    }
}