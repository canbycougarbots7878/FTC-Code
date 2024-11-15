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
    DcMotor arm1 = null;
    DcMotor arm2 = null;

    public void runOpMode() {
        motorFL = hardwareMap.get(DcMotor.class, "FrontLeft");
        motorBL = hardwareMap.get(DcMotor.class, "BackLeft");
        motorFR = hardwareMap.get(DcMotor.class, "FrontRight");
        motorBR = hardwareMap.get(DcMotor.class, "BackRight");

        arm1 = hardwareMap.get(DcMotor.class, "Arm1");
        arm2 = hardwareMap.get(DcMotor.class, "Arm2");
        CRServo clawRotation = hardwareMap.get(CRServo.class, "clawRotation");
        CRServo claw = hardwareMap.get(CRServo.class, "claw");

        motorFR.setDirection(DcMotor.Direction.FORWARD);
        motorFL.setDirection(DcMotor.Direction.REVERSE);
        motorBR.setDirection(DcMotor.Direction.FORWARD);
        motorBL.setDirection(DcMotor.Direction.REVERSE);

        arm1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        arm1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();

        while (opModeIsActive()) {
            Forward(1,0.5);
            Turn(1,0.5);
        }

    }

    private void Turn(int TR, double Power) {
        motorFL.setTargetPosition(TR * 1440);
        motorBL.setTargetPosition(TR * 1440);
        motorFR.setTargetPosition(-TR * 1440);
        motorBR.setTargetPosition(-TR * 1440);

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
        motorFL.setTargetPosition(TD*1440);
        motorBL.setTargetPosition(TD*1440);
        motorFR.setTargetPosition(TD*1440);
        motorBR.setTargetPosition(TD*1440);

        motorFL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorFR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motorFL.setPower(Power);
        motorBL.setPower(Power);
        motorFR.setPower(Power);
        motorBR.setPower(Power);

    }
}