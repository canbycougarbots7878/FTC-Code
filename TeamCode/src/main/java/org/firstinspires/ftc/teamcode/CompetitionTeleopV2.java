package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
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
        DcMotor Front_Right = hardwareMap.get(DcMotor.class, "Front Right");
        DcMotor Front_Left = hardwareMap.get(DcMotor.class, "Front Left");
        DcMotor Back_Right = hardwareMap.get(DcMotor.class, "Back Right");
        DcMotor Back_Left = hardwareMap.get(DcMotor.class, "Back Left");
        DriveWheels Wheels = new DriveWheels(Front_Right, Front_Left, Back_Right, Back_Left);
        // Initialize Arm
        //Arm = hardwareMap.get(DcMotor.class, "Extending Arm");
        //Slider = hardwareMap.get(DcMotor.class, "Slide Arm");
        // Initialize Claw
        //Claw = hardwareMap.get(Servo.class, "Claw");
        //Wrist = hardwareMap.get(Servo.class, "Wrist");
        waitForStart();
        while(opModeIsActive()) {
            Wheels.Omni_Move(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x, 1);
        }
    }
}
