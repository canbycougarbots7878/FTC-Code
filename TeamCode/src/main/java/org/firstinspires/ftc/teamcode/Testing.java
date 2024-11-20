package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
@TeleOp(name = "Arm Testing", group = "Concept")
public class Testing extends LinearOpMode {
    public void runOpMode() {
        DcMotor motorFL = hardwareMap.get(DcMotor.class, "FrontLeft");
        DcMotor motorBL = hardwareMap.get(DcMotor.class, "BackLeft");
        DcMotor motorFR = hardwareMap.get(DcMotor.class, "FrontRight");
        DcMotor motorBR = hardwareMap.get(DcMotor.class, "BackRight");
        DcMotor arm1 = hardwareMap.get(DcMotor.class, "Arm1");
        DcMotor arm2 = hardwareMap.get(DcMotor.class, "Arm2");
        Servo wrist = hardwareMap.get(Servo.class, "wrist");
        Servo claw = hardwareMap.get(Servo.class, "claw");
        motorFR.setDirection(DcMotor.Direction.FORWARD);
        motorFL.setDirection(DcMotor.Direction.REVERSE);
        motorBR.setDirection(DcMotor.Direction.FORWARD);
        motorBL.setDirection(DcMotor.Direction.REVERSE);
        arm1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        waitForStart();
        while (opModeIsActive()) {
            telemetry.addData("Arm1 Position", arm1.getCurrentPosition());
            telemetry.addData("Arm2 Position", arm2.getCurrentPosition());
            telemetry.update();
            if(gamepad1.a) {
                //Home
                arm1.setTargetPosition(0);
                arm1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                arm2.setTargetPosition(0);
                arm2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                arm1.setPower(.3);
                arm2.setPower(1);
            }
            else if(gamepad1.b) {
                //Deploy
                arm1.setTargetPosition(-112);
                arm1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                arm2.setTargetPosition(-77);
                arm2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                arm1.setPower(.5);
                arm2.setPower(.5);
            }
            else if(gamepad1.y) {
                //Up
                arm1.setTargetPosition(-1127);
                arm1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                arm2.setTargetPosition(-128);
                arm2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                arm1.setPower(.5);
                arm2.setPower(.5);
            }
            else if(gamepad1.x) {
                //Bucket
                arm1.setTargetPosition(-1127);
                arm1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                arm2.setTargetPosition(-160);
                arm2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                arm1.setPower(.5);
                arm2.setPower(.5);
            }
            if(gamepad1.right_bumper) {
                claw.setPosition(0);
            }
            else {
                claw.setPosition(1);
            }
        }
    }
}
