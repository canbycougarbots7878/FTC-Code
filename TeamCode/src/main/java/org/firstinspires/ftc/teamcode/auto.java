package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;

//TODO
// 4. Create code for arm
// 5. input code for arm

//Notes
// 1. 90 degrees counterclockwise is 470 mm


@Autonomous(name = "COMPETITION: Auto Drive", group = "Concept")
public class auto extends LinearOpMode {
    double Tau = Math.PI * 2;
    DcMotor motorFL = null;
    DcMotor motorBL = null;
    DcMotor motorFR = null;
    DcMotor motorBR = null;

    /*
    DcMotor arm1 = null;
    Servo arm2 = null;
    CRServo claw = null;
     */

    public void runOpMode() {
        motorFR = hardwareMap.get(DcMotor.class, "FrontRight");
        motorBR = hardwareMap.get(DcMotor.class, "BackRight");
        motorBL = hardwareMap.get(DcMotor.class, "BackLeft");
        motorFL = hardwareMap.get(DcMotor.class, "FrontLeft");

        /*
        arm1 = hardwareMap.get(DcMotor.class, "Arm1");
        arm2 = hardwareMap.get(Servo.class, "Arm2");
        claw = hardwareMap.get(CRServo.class, "claw");
         */

        motorFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motorFR.setDirection(DcMotor.Direction.REVERSE);
        motorBR.setDirection(DcMotor.Direction.FORWARD);
        motorBL.setDirection(DcMotor.Direction.REVERSE);
        motorFL.setDirection(DcMotor.Direction.REVERSE);

        motorFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //arm1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //arm1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();

        // Print starting numbers and move forward
        TelemetryPosition();
        //Home();

        Forward(500, 0.75);

        sleep(1000);

        Turn(-450, 0.75);

        sleep(1000);

        Forward(620, 0.75);

        sleep(1000);

        Turn(-450, 0.75);

        sleep(1000);

        Forward(100, 0.75);

        sleep(1000);

        Telemetry(1);

        sleep(1000);

        Telemetry(2);

        sleep(1000);

        Telemetry(1);

        sleep(1000);

        Forward(-100, 0.75);

        sleep(1000);

        Turn(-450, 0.75);

        sleep(1000);

        Forward(1100, 0.75);

        sleep(1000);

        Turn(-450, 0.75);

        sleep(1000);

        Telemetry(3);

        sleep(1000);

        Forward(70, 0.75);

        sleep(1000);

        Telemetry(3);

        sleep(1000);

        Forward(-70, 0.75);

        sleep(1000);

        Turn(-450, 0.75);

        Forward(1100, 0.75);

        Turn(450, 0.75);

        // Print ending numbers
        TelemetryPosition();

        motorFR.setPower(0);
        motorBR.setPower(0);
        motorBL.setPower(0);
        motorFL.setPower(0);

        // Leave the data on the screen so we have time to write it down
        while (opModeIsActive()) {
            // Do nothing, just wait
        }
    }

    private void Forward(double TargetDistance, double Power) {
        motorFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        double Circumference = 52 * Tau;
        double CountsPerMillimeter = 1440 / Circumference;

        motorFR.setTargetPosition((int) (TargetDistance * CountsPerMillimeter));
        motorBR.setTargetPosition((int) (TargetDistance * CountsPerMillimeter));
        motorBL.setTargetPosition((int) (TargetDistance * CountsPerMillimeter));
        motorFL.setTargetPosition((int) (TargetDistance * CountsPerMillimeter));

        motorFR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorFL.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motorFR.setPower(Power);
        motorBR.setPower(Power);
        motorBL.setPower(Power);
        motorFL.setPower(Power);

        // Wait until the motors are done spinning
        while (motorFL.isBusy() && motorBL.isBusy() && motorBR.isBusy() && motorFR.isBusy()) {
            // Do nothing, just wait
        }

    }

    private void TelemetryPosition() {
        telemetry.addData("Front Right", motorFR.getCurrentPosition());
        telemetry.addData("Back Right", motorBR.getCurrentPosition());
        telemetry.addData("Back Left", motorBL.getCurrentPosition());
        telemetry.addData("Front Left", motorFL.getCurrentPosition());
        telemetry.update();
    }

    private void Telemetry(int Word) {
        if (Word == 1) {
            telemetry.addData("", "Arm");
            telemetry.update();
        } else if (Word == 2) {
            telemetry.addData("", "Claw");
            telemetry.update();
        } else if (Word == 3) {
            telemetry.addData("", "Lifter");
            telemetry.update();
        }
    }

    private void Turn(double TargetRadians, double Power) {
        motorFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        double Circumference = 52 * Tau;
        double CountsPerMillimeter = 1440 / Circumference;

        motorFR.setTargetPosition((int) (TargetRadians * CountsPerMillimeter));
        motorBR.setTargetPosition((int) (TargetRadians * CountsPerMillimeter));
        motorBL.setTargetPosition((int) (-TargetRadians * CountsPerMillimeter));
        motorFL.setTargetPosition((int) (-TargetRadians * CountsPerMillimeter));

        motorFR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorFL.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motorFR.setPower(Power);
        motorBR.setPower(Power);
        motorBL.setPower(Power);
        motorFL.setPower(Power);

        // Wait until the motors are done spinning
        while (motorFL.isBusy() && motorBL.isBusy() && motorBR.isBusy() && motorFR.isBusy()) {
            // Do nothing, just wait
        }

    }

    /*
    private void Home() {
            arm1.setTargetPosition(0);
            arm1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            arm1.setPower(.1);
            arm2.setPosition(0);
    }
     */
}
