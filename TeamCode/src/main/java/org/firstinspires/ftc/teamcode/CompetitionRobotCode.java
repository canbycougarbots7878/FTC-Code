package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.motors.RevRoboticsCoreHexMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Competition Code", group = "Concept")
public class CompetitionRobotCode extends LinearOpMode {

    public void runOpMode() {
        DcMotor motorFL = hardwareMap.get(DcMotor.class, "FrontLeft");
        DcMotor motorBL = hardwareMap.get(DcMotor.class, "BackLeft");
        DcMotor motorFR = hardwareMap.get(DcMotor.class, "FrontRight");
        DcMotor motorBR = hardwareMap.get(DcMotor.class, "BackRight");

        motorFR.setDirection(DcMotor.Direction.FORWARD);
        motorFL.setDirection(DcMotor.Direction.REVERSE);
        motorBR.setDirection(DcMotor.Direction.FORWARD);
        motorBL.setDirection(DcMotor.Direction.REVERSE);

        CRServo CRServo0  = hardwareMap.get(CRServo.class,"arm");
        CRServo CRServo1  = hardwareMap.get(CRServo.class,"grabber");

        DcMotor RevRoboticsCoreHexMotor0 = hardwareMap.get(DcMotor.class,"Lift");

        waitForStart();

        while (opModeIsActive()) {

            double axial   = -gamepad1.left_stick_y;
            double lateral =  gamepad1.left_stick_x;
            double yaw     =  gamepad1.right_stick_x;
            double speed   =  1 + gamepad1.right_trigger;

            double leftFrontPower  = axial + lateral - yaw;
            double rightFrontPower = axial - lateral + yaw;
            double leftBackPower   = axial - lateral - yaw;
            double rightBackPower  = axial + lateral + yaw;

            motorFR.setPower(leftFrontPower / speed);
            motorFL.setPower(rightFrontPower / speed);
            motorBR.setPower(leftBackPower / speed);
            motorBL.setPower(rightBackPower / speed);

            if (gamepad1.x) {
                CRServo0.setPower(gamepad1.right_trigger - gamepad1.left_trigger);
            }
            if (gamepad1.y) {
                CRServo1.setPower(gamepad1.right_trigger - gamepad1.left_trigger);
            }
            if (gamepad1.a) {
                RevRoboticsCoreHexMotor0.setPower(gamepad1.right_trigger - gamepad1.left_trigger);
            }
            if (gamepad1.b) {
                double position = 0.5 + ((gamepad1.right_trigger) - (gamepad1.left_trigger)) / 2;
            }
        }
    }
}
