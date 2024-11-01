package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "COMPETITION: Manual Drive", group = "Concept")
public class CompetitionRobotCode extends LinearOpMode {
    public void runOpMode() {
        DcMotor motorFL = hardwareMap.get(DcMotor.class, "FrontLeft");
        DcMotor motorBL = hardwareMap.get(DcMotor.class, "BackLeft");
        DcMotor motorFR = hardwareMap.get(DcMotor.class, "FrontRight");
        DcMotor motorBR = hardwareMap.get(DcMotor.class, "BackRight");

        DcMotor arm = hardwareMap.get(DcMotor.class, "Arm1");

        motorFR.setDirection(DcMotor.Direction.REVERSE);
        motorFL.setDirection(DcMotor.Direction.FORWARD);
        motorBR.setDirection(DcMotor.Direction.REVERSE);
        motorBL.setDirection(DcMotor.Direction.FORWARD);

        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();

        while (opModeIsActive()) {

            double axial   = -gamepad1.left_stick_y;
            double lateral =  gamepad1.left_stick_x;
            double yaw     =  gamepad1.right_stick_x;
            double speed = .5;
            if (gamepad1.right_bumper) {
                speed = 1;
            }
            else if (gamepad1.left_bumper) {
                speed = .25;
            }

            double leftFrontPower  = axial + lateral - yaw;
            double rightFrontPower = axial - lateral + yaw;
            double leftBackPower   = axial - lateral - yaw;
            double rightBackPower  = axial + lateral + yaw;

            motorFR.setPower(speed * leftFrontPower);
            motorFL.setPower(speed * rightFrontPower);
            motorBR.setPower(speed * leftBackPower);
            motorBL.setPower(speed * rightBackPower);
            telemetry.addData("Arm Position", arm.getCurrentPosition());
            telemetry.update();
            if (gamepad1.a) {
                arm.setTargetPosition(0);
                arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                arm.setPower(.5);
            }
            if (gamepad1.b) {
                arm.setTargetPosition(-400);
                arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                arm.setPower(.5);
            }
        }
    }
}
