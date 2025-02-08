package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

//TODO

//Notes
// 1. 90 degrees counterclockwise is 450 mm


@TeleOp(name = "Speed Test", group = "Concept")
public class speedTest extends LinearOpMode {

    DcMotor motorFR = null;
    DcMotor motorBR = null;
    DcMotor motorBL = null;
    DcMotor motorFL = null;

    public void runOpMode() {
        motorFR = hardwareMap.get(DcMotor.class, "FrontRight");
        motorBR = hardwareMap.get(DcMotor.class, "BackRight");
        motorBL = hardwareMap.get(DcMotor.class, "BackLeft");
        motorFL = hardwareMap.get(DcMotor.class, "FrontLeft");

        motorFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motorFR.setDirection(DcMotor.Direction.REVERSE);
        motorBR.setDirection(DcMotor.Direction.REVERSE);
        motorBL.setDirection(DcMotor.Direction.FORWARD);
        motorFL.setDirection(DcMotor.Direction.FORWARD);


        waitForStart();

        telemetry.addData("ID", gamepad1.id);
        telemetry.update();

        while (opModeIsActive()) {
            if (gamepad1.a) {
                motorFR.setPower(-1);
                motorBR.setPower(-1);
                motorBL.setPower(1);
                motorFL.setPower(1);
                sleep(1000);
                motorFR.setPower(0);
                motorBR.setPower(0);
                motorBL.setPower(0);
                motorFL.setPower(0);
            }
            if (gamepad1.b) {
                motorFR.setPower(-0.75);
                motorBR.setPower(-0.75);
                motorBL.setPower(0.75);
                motorFL.setPower(0.75);
                sleep(1000);
                motorFR.setPower(0);
                motorBR.setPower(0);
                motorBL.setPower(0);
                motorFL.setPower(0);
            }
            if (gamepad1.x) {
                motorFR.setPower(-0.5);
                motorBR.setPower(-0.5);
                motorBL.setPower(0.5);
                motorFL.setPower(0.5);
                sleep(1000);
                motorFR.setPower(0);
                motorBR.setPower(0);
                motorBL.setPower(0);
                motorFL.setPower(0);
            }
            if (gamepad1.y) {
                motorFR.setPower(-0.25);
                motorBR.setPower(-0.25);
                motorBL.setPower(0.25);
                motorFL.setPower(0.25);
                sleep(1000);
                motorFR.setPower(0);
                motorBR.setPower(0);
                motorBL.setPower(0);
                motorFL.setPower(0);
            }
        }
    }
}