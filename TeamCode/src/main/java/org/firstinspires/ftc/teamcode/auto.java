package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;

//TODO

//Notes
// 1. 90 degrees counterclockwise is 450 mm


@Autonomous(name = "COMPETITION: Auto Drive", group = "Concept")
public class auto extends LinearOpMode {
    double Tau = Math.PI * 2;
    DcMotor motorFL = null;
    DcMotor motorBL = null;
    DcMotor motorFR = null;
    DcMotor motorBR = null;

    DcMotor arm      = null;
    DcMotor slidearm = null;

    Servo claw  = null;
    Servo wrist = null;
    public void runOpMode() {
        motorFR = hardwareMap.get(DcMotor.class, "FrontRight");
        motorBR = hardwareMap.get(DcMotor.class, "BackRight");
        motorBL = hardwareMap.get(DcMotor.class, "BackLeft");
        motorFL = hardwareMap.get(DcMotor.class, "FrontLeft");

        slidearm = hardwareMap.get(DcMotor.class, "Slide Arm");
        arm = hardwareMap.get(DcMotor.class, "Extending Arm");

        wrist = hardwareMap.get(Servo.class, "Wrist");
        claw = hardwareMap.get(Servo.class, "Claw");

        motorFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motorFR.setDirection(DcMotor.Direction.FORWARD);
        motorBR.setDirection(DcMotor.Direction.REVERSE);
        motorBL.setDirection(DcMotor.Direction.FORWARD);
        motorFL.setDirection(DcMotor.Direction.FORWARD);

        slidearm.setTargetPosition(100);
        slidearm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //slidearm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slidearm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motorFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //arm1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //arm1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();

        // Print starting numbers and move forward
        TelemetryPosition();
        int rest = 1000;
        Home();

        Forward(500, 0.75);

        // Wait until the motors are done spinning
        while (motorFL.isBusy() && motorBL.isBusy() && motorBR.isBusy() && motorFR.isBusy()) {
            // Do nothing, just wait
        }

        sleep(rest);

        Turn(450, 0.75);

        sleep(rest);

        Forward(304.8, 0.75);

        sleep(rest);

        Turn(-450, 0.75);

        sleep(rest);

        Reach();

        sleep(rest);

        Forward(70, 0.75);

        sleep(rest);

        Basket();

        sleep(rest);

        claw.setPosition(.5);

        sleep(rest);

        Home();

        sleep(rest);

        Forward(-70, 0.75);

        sleep(rest);

        Turn(-450, 0.75);

        sleep(rest);

        Forward(1100, 0.75);

        sleep(rest);

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

    private void SetSliderPosition(int target) {
        slidearm.setTargetPosition(target);
        slidearm.setPower(.5);
    }

    private void SetSliderSegment(int segment) {
        //slidearm.setPower(1);
        switch(segment) {
            case(0): SetSliderPosition(100); break;
            case(1): SetSliderPosition(1437); break;
            case(2): SetSliderPosition(2971); break;
            case(3): SetSliderPosition(4498); break;
            case(4): SetSliderPosition(6025); break;
        }
    }

    private void SetArmPosition(int position) {
        boolean direction = (arm.getCurrentPosition() < position);
        int proximity = Math.abs(arm.getCurrentPosition() - position);
        double speed = ProximityRunge(proximity);
        if(direction) {
            arm.setPower(speed);
        }
        else {
            arm.setPower(-speed);
        }
    }

    private double ProximityRunge(int position) {
        double x = (double)position / 4;
        return 1 - 1/(1+x*x);
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

        //I am going to do acceleration for the wheels

        motorFR.setPower(Power);
        motorBR.setPower(Power);
        motorBL.setPower(Power);
        motorFL.setPower(Power);

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

    private void Home() {
        SetSliderSegment(0);
        SetArmPosition(0);
    }

    private void Deploy() {
        SetSliderSegment(0);
        SetArmPosition(-70);
    }

    private void Basket() {
        SetSliderSegment(2);
    }

    private void Reach() {
        SetSliderSegment(3);
    }


}
