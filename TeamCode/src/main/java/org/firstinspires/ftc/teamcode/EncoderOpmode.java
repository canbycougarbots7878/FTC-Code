package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

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
        motorFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); motorFR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); motorFL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); motorBR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); motorBL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        waitForStart();
        while (opModeIsActive()) {
            MoveWheels(1000);
            sleep(500);
            MoveWheels(0);
            sleep(500);
        }
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
    private int sign(int n) {
        if(n==0){return 0;}
        return (n > 0) ? 1 : -1;
    }
    private void MoveWheels(int position) {
        motorFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFR.setTargetPosition(position);
        motorFL.setTargetPosition(position);
        motorBR.setTargetPosition(position);
        motorBL.setTargetPosition(position);
        motorFR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorFL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        MoveRobot(1, 0);
        while((motorFR.isBusy() || motorFL.isBusy() || motorBR.isBusy() || motorBL.isBusy()) && !isStopRequested()) {

        }
        StopWheels();
    }
}
