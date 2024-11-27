package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;

//TODO
// 1. find out why "motorFR.getCurrentPosition()" is negative not positive
// 2. fix "motorFR.getCurrentPosition()" to be positive
// 3. Create movement to all places on the field
// 4. Create code for arm
// 5. input code for arm

@Autonomous(name = "COMPETITION: Auto Drive", group = "Concept")
public class auto extends LinearOpMode {
    DcMotor motorFL = null;
    DcMotor motorBL = null;
    DcMotor motorFR = null;
    DcMotor motorBR = null;

    public void runOpMode() {
        motorFR = hardwareMap.get(DcMotor.class, "FrontRight");
        motorBR = hardwareMap.get(DcMotor.class, "BackRight");
        motorBL = hardwareMap.get(DcMotor.class, "BackLeft");
        motorFL = hardwareMap.get(DcMotor.class, "FrontLeft");

        motorFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motorFR.setDirection(DcMotor.Direction.FORWARD);
        motorBR.setDirection(DcMotor.Direction.FORWARD);
        motorBL.setDirection(DcMotor.Direction.REVERSE);
        motorFL.setDirection(DcMotor.Direction.REVERSE);

        motorFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        waitForStart();

        TelemetryPosition();
        Forward(500, 0.5);
        TelemetryPosition();
        //Turn((int)(Math.PI/2), 0.5);
        while (motorFL.isBusy() && motorBL.isBusy() && motorBR.isBusy() && motorFR.isBusy()){
        }
        motorFR.setPower(0);
        motorBR.setPower(0);
        motorBL.setPower(0);
        motorFL.setPower(0);
        while (opModeIsActive()){
        }
    }

    private void Turn(int TargetRadius, double Power) {
        double Circumference       = 104 * Math.PI;
        double CountsPerMillimeter = 1440/Circumference;
        int Millimeter             = (int)(CountsPerMillimeter);

        motorFR.setTargetPosition(TargetRadius  * Millimeter);
        motorBR.setTargetPosition(TargetRadius  * Millimeter);
        motorBL.setTargetPosition(-TargetRadius * Millimeter);
        motorFL.setTargetPosition(-TargetRadius * Millimeter);

        motorFR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorFL.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motorFR.setPower(Power);
        motorBR.setPower(Power);
        motorBL.setPower(Power);
        motorFL.setPower(Power);

    }

    private void Forward(int TargetDistance, double Power) {
        double Circumference       = 104 * Math.PI;
        double CountsPerMillimeter = 1440/Circumference;
        int Millimeter             = (int)(CountsPerMillimeter);

        motorFR.setTargetPosition(TargetDistance * Millimeter);
        motorBR.setTargetPosition(TargetDistance * Millimeter);
        motorBL.setTargetPosition(TargetDistance * Millimeter);
        motorFL.setTargetPosition(TargetDistance * Millimeter);

        motorFR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorFL.setMode(DcMotor.RunMode.RUN_TO_POSITION);

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
}
