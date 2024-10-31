package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Set;
@TeleOp
public class EncoderOpmode extends LinearOpMode {
    DcMotor motorFR = null;
    DcMotor motorFL = null;
    DcMotor motorBR = null;
    DcMotor motorBL = null;
    @Override
    public void runOpMode() {
        motorFR = hardwareMap.get(DcMotor.class, "FrontRight");
        motorFL = hardwareMap.get(DcMotor.class, "FrontLeft");
        motorBR = hardwareMap.get(DcMotor.class, "BackRight");
        motorBL = hardwareMap.get(DcMotor.class, "BackLeft");
        motorFR.setDirection(DcMotor.Direction.FORWARD);
        motorFL.setDirection(DcMotor.Direction.REVERSE);
        motorBR.setDirection(DcMotor.Direction.FORWARD);
        motorBL.setDirection(DcMotor.Direction.REVERSE);
        motorFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        waitForStart();
        while (opModeIsActive()) {
            TelemetryPosition();
            if (gamepad1.a) {
                motorFR.setTargetPosition(0);
                motorFL.setTargetPosition(0);
                motorBR.setTargetPosition(0);
                motorBL.setTargetPosition(0);
                motorFR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motorFL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motorBR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motorBL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                SetWheels(1,1,1,1);
            }
        }
    }
    private void motorFRsetTargetPosition(int targetPos) {
        int currentPos = - motorFR.getCurrentPosition();
        if (currentPos < targetPos) {
            motorFR.setPower((- motorFR.getCurrentPosition()) / 100);
            while(- motorFR.getCurrentPosition() < targetPos) {}
        }
        if (currentPos > targetPos) {
            motorFR.setPower((motorFR.getCurrentPosition()) / 100);
            while(- motorFR.getCurrentPosition() > targetPos) {}
        }
        motorFR.setPower(0);
    }
    private void TelemetryPosition() {
        telemetry.addData("Front Right", motorFR.getCurrentPosition());
        telemetry.addData("Front Left", motorFL.getCurrentPosition());
        telemetry.addData("Back Right", motorBR.getCurrentPosition());
        telemetry.addData("Back Left", motorBL.getCurrentPosition());
        telemetry.update();
    }
    private void SetWheels(double FR, double FL, double BR, double BL) {
        motorFR.setPower(FR);
        motorFL.setPower(FL);
        motorBR.setPower(BR);
        motorBL.setPower(BL);
    }

    private void StopWheels() {
        SetWheels(0,0,0,0);
    } private void StopWheels(long wait) { sleep(wait); StopWheels(); }

    private void TurnRobot(double amount) {
        SetWheels(-amount, amount, -amount, amount);
    } private void TurnRobot(double amount, long time) { TurnRobot(amount); StopWheels(time); }

    private void MoveRobot(double forward, double right, double turn) {
        SetWheels(forward + right - turn, forward - right + turn, forward - right - turn, forward + right + turn);
    } private void MoveRobot(double forward, double right) { MoveRobot(forward, right, 0);}

    private void Display(String Message) {
        telemetry.addLine(Message);
        telemetry.update();
    }
}
