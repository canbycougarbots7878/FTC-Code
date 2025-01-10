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
        // Initialize Arm
        Arm = hardwareMap.get(DcMotor.class, "Extending Arm");
        Arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Arm.setTargetPosition(0);
        Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Arm.setPower(0);
        // Initialize Slider
        Slider = hardwareMap.get(DcMotor.class, "Slide Arm");
        Slider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Slider.setTargetPosition(0);
        Slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Slider.setPower(0);
        // Initialize Claw
        Claw = hardwareMap.get(Servo.class, "Claw");
        //Wrist = hardwareMap.get(Servo.class, "Wrist");
        // Variables
        boolean debounce = true;
        boolean extend = false;
        boolean armdown = false;
        waitForStart();
        while(opModeIsActive()) {
            Wheels.Omni_Move(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x, 1);
            if (gamepad2.y) {
                if(debounce) {
                    debounce = false;
                    extend = !extend;
                }
            }
            else if (gamepad2.x) {
                if(debounce) {
                    debounce = false;
                    armdown = !armdown;
                }
            }
            else {
                debounce = true;
            }
            if (extend) {
                Slider.setTargetPosition(4500);
                Slider.setPower(1);
            }
            else {
                Slider.setTargetPosition(0);
                if (Slider.getCurrentPosition() < 100) {
                    Slider.setPower(0);
                }
                else {
                    Slider.setPower(-.5);
                }
            }
            if (armdown) {
                Arm.setTargetPosition(-50);
                Arm.setPower(1);
            }
            else {
                Arm.setTargetPosition(0);
                if (Arm.getCurrentPosition() > 0) {
                    Arm.setPower(0);
                }
                else {
                    Arm.setPower(1);
                }
            }
            Claw.setPosition(1 - gamepad2.right_trigger);
        }
    }
}
