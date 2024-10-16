package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.motors.RevRoboticsCoreHexMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Manual Drive", group = "Concept")
public class CompetitionRobotCode extends LinearOpMode {
    private double Speed = .5;
    public void runOpMode() {
        DcMotor motorFL = hardwareMap.get(DcMotor.class, "FrontLeft");
        DcMotor motorBL = hardwareMap.get(DcMotor.class, "BackLeft");
        DcMotor motorFR = hardwareMap.get(DcMotor.class, "FrontRight");
        DcMotor motorBR = hardwareMap.get(DcMotor.class, "BackRight");

        motorFR.setDirection(DcMotor.Direction.REVERSE);
        motorFL.setDirection(DcMotor.Direction.FORWARD);
        motorBR.setDirection(DcMotor.Direction.REVERSE);
        motorBL.setDirection(DcMotor.Direction.FORWARD);

        CRServo CRServo0  = hardwareMap.get(CRServo.class,"arm");
        Servo Servo1  = hardwareMap.get(Servo.class,"grabber");

        DcMotor RevRoboticsCoreHexMotor0 = hardwareMap.get(DcMotor.class,"Lift");

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

            CRServo0.setPower(1);
            if (gamepad1.x) {
                CRServo0.setPower(gamepad1.right_trigger - gamepad1.left_trigger);
            }
            if (gamepad1.y) {
                double position = 0.5 + ((gamepad1.right_trigger) - (gamepad1.left_trigger)) / 2;
                Servo1.setPosition(position);
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
