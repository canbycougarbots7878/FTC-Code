package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;

//TODO:
// - ARM1 Positions using SelfCheck.java
// - ARM2 Positions
// -

@Autonomous(name = "COMPETITION: Auto Drive", group = "Concept")
public class auto extends LinearOpMode {
    DcMotor motorFL = null;
    DcMotor motorBL = null;
    DcMotor motorFR = null;
    DcMotor motorBR = null;
    //DcMotor arm1 = null;
    //DcMotor arm2 = null;

    public void runOpMode() {
        motorFL = hardwareMap.get(DcMotor.class, "FrontLeft");
        motorBL = hardwareMap.get(DcMotor.class, "BackLeft");
        motorFR = hardwareMap.get(DcMotor.class, "FrontRight");
        motorBR = hardwareMap.get(DcMotor.class, "BackRight");

        motorFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

       // arm1 = hardwareMap.get(DcMotor.class, "Arm1");
       // arm2 = hardwareMap.get(DcMotor.class, "Arm2");
       // CRServo clawRotation = hardwareMap.get(CRServo.class, "clawRotation");
        //CRServo claw = hardwareMap.get(CRServo.class, "claw");

        motorFR.setDirection(DcMotor.Direction.FORWARD);
        motorFL.setDirection(DcMotor.Direction.REVERSE);
        motorBR.setDirection(DcMotor.Direction.FORWARD);
        motorBL.setDirection(DcMotor.Direction.REVERSE);

        motorFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //arm1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //arm2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //arm1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //arm2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();

        TelemetryPosition();
        Forward(1000, 0.5);
        TelemetryPosition();
        Turn((int)(Math.PI/2), 0.5);
        while (motorFL.isBusy() && motorBL.isBusy() && motorBR.isBusy() && motorFR.isBusy()){

        }
        motorFL.setPower(0);
        motorBL.setPower(0);
        motorFR.setPower(0);
        motorBR.setPower(0);
        //12.762720155208534in forward?
        while (opModeIsActive()){

        }

    }

    private void Turn(int TR, double Power) {
        double C = 104 * Math.PI;
        double CountsPerMillimeter = 1440/C;
        int Millimeter = (int)(CountsPerMillimeter);
        motorFL.setTargetPosition(-TR * Millimeter);
        motorBL.setTargetPosition(-TR * Millimeter);
        motorFR.setTargetPosition(TR * Millimeter);
        motorBR.setTargetPosition(TR * Millimeter);

        motorFL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorFR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motorFL.setPower(Power);
        motorBL.setPower(Power);
        motorFR.setPower(Power);
        motorBR.setPower(Power);

    }

    private void Forward(int TD, double Power) {
        double C = 104 * Math.PI;
        double CountsPerMillimeter = 1440/C;
        int Millimeter = (int)(CountsPerMillimeter);
        motorFL.setTargetPosition(TD * Millimeter);
        motorBL.setTargetPosition(TD * Millimeter);
        motorFR.setTargetPosition(TD * Millimeter);
        motorBR.setTargetPosition(TD * Millimeter);

        motorFL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorFR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motorFL.setPower(Power);
        motorBL.setPower(Power);
        motorFR.setPower(Power);
        motorBR.setPower(Power);

        while (motorFL.isBusy() && motorBL.isBusy() && motorBR.isBusy() && motorFR.isBusy()){

        }
    }


    private void TelemetryPosition() {
        telemetry.addData("Front Right", motorFR.getCurrentPosition());
        telemetry.addData("Front Left", motorFL.getCurrentPosition());
        telemetry.addData("Back Right", motorBR.getCurrentPosition());
        telemetry.addData("Back Left", motorBL.getCurrentPosition());
        telemetry.update();
    }
}