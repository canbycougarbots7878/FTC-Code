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
        //Arm = hardwareMap.get(DcMotor.class, "Extending Arm");
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
        waitForStart();
        while(opModeIsActive()) {
            Wheels.Omni_Move(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x, 1);
            if (gamepad1.dpad_up) {
                if(debounce) {
                    debounce = false;
                    extend = !extend;
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
            Claw.setPosition(1 - gamepad1.right_trigger);
        }
    }
}
