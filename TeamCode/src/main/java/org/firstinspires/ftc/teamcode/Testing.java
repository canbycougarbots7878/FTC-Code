package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
@TeleOp(name = "Arm Testing", group = "Concept")
public class Testing extends LinearOpMode {
    DcMotor FrontRight = null;
    DcMotor FrontLeft = null;
    DcMotor BackRight = null;
    DcMotor BackLeft = null;
    DcMotor Arm = null;

    public void runOpMode() {
        FrontRight = hardwareMap.get(DcMotor.class, "FrontRight");
        BackRight = hardwareMap.get(DcMotor.class, "BackRight");
        FrontLeft = hardwareMap.get(DcMotor.class, "FrontLeft");
        BackLeft = hardwareMap.get(DcMotor.class, "BackLeft");
        Arm = hardwareMap.get(DcMotor.class, "Extending Arm");
        FrontRight.setDirection(DcMotor.Direction.REVERSE);
        BackRight.setDirection(DcMotor.Direction.REVERSE);
        FrontLeft.setDirection(DcMotor.Direction.FORWARD);
        BackLeft.setDirection(DcMotor.Direction.FORWARD);
        FrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        waitForStart();
        while (opModeIsActive()) {
            SetArmPosition(-30);
            telemetry.addData("Arm position", Arm.getCurrentPosition());
            telemetry.update();
        }
    }
    private void SetAllMotors(double speed) {
        FrontRight.setPower(speed);
        BackRight.setPower(speed);
        FrontLeft.setPower(speed);
        BackLeft.setPower(speed);
    }
    private void StopAllMotors() {
        SetAllMotors(0);
    }
    private void MoveMM(int distance) {
        SetAllMotors(.25);
        sleep(distance/310);
        StopAllMotors();
    }
    private void SetArmPosition(int target) {
        int x = Arm.getCurrentPosition() - target;
        double speed = 1 / (1 + Math.pow(10, x));
        telemetry.addData("Speed", speed);
        Arm.setPower(speed);
    }
}
