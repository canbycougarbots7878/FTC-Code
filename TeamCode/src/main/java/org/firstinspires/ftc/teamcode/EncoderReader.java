package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.MovementLib.*;
@Disabled
@TeleOp(name = "Encoder Reader", group = "Testing")
public class EncoderReader extends LinearOpMode {
    Servo Arm = null;
    DcMotor Slider = null;
    Servo Arm_Lock = null;
    Servo Claw = null;
    Servo Wrist = null;
    @Override
    public void runOpMode() {
        // Initialize Slider
        Slider = hardwareMap.get(DcMotor.class, "Slide Arm");
        Slider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Slider.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // Initialize Arm
        Arm = hardwareMap.get(Servo.class, "Arm");
        // Initialize Arm Lock
        Arm_Lock = hardwareMap.get(Servo.class, "Arm Lock");
        // Initialize Claw
        Claw = hardwareMap.get(Servo.class, "Claw");
        Wrist = hardwareMap.get(Servo.class, "Wrist");
        Wrist.setPosition(0);
        // Variables
        int slider_position = 0;
        int arm_position = 0;
        int arm_good = 0;
        waitForStart();
        Arm_Lock.setPosition(.5);
        while(opModeIsActive()) {
            if (gamepad1.a) {
                Claw.setPosition(gamepad1.right_trigger);
            }
            else if (gamepad1.b) {
                Wrist.setPosition(gamepad1.right_trigger);
            }
            else if (gamepad1.x) {
                Arm_Lock.setPosition(gamepad1.right_trigger);
            }
            else if (gamepad1.y) {
                Arm.setPosition(gamepad1.right_trigger);
            }
            Slider.setPower(gamepad1.left_trigger);
            telemetry.addData("Slider position", Slider.getCurrentPosition());
            telemetry.addData("Right trigger", gamepad1.right_trigger);
            telemetry.update();
        }
    }
}