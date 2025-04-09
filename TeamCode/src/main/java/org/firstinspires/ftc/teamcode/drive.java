package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.teamcode.MovementLib.*;
@TeleOp(name = "Drive", group = "Competition")
@Disabled
public class drive extends LinearOpMode {

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

        double robot_speed = .5;
        waitForStart();

        while(opModeIsActive()) {
            Wheels.Omni_Move(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, robot_speed);
            telemetry.addData("Robot Speed", robot_speed);
            telemetry.update();
            if (gamepad1.right_bumper) {
                robot_speed = 1;
            } else if (gamepad1.left_bumper) {
                robot_speed = 0.25;
            }else {
                robot_speed = 0.5;
            }
            if (gamepad1.right_trigger == 1){
                Wheels.Omni_Move(1, 0, 0, 1);
            } else if (gamepad1.left_trigger == 1) {
                Wheels.Omni_Move(-1, 0, 0, 1);
            }
        }
        Wheels.Omni_Move(0,0,0,0);
    }
}