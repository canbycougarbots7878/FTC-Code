package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import java.util.Set;

@TeleOp(name = "Test Wheels", group = "Concept")
public class WheelTest extends LinearOpMode {
    DcMotor motorFR = null;
    DcMotor motorFL = null;
    DcMotor motorBR = null;
    DcMotor motorBL = null;
    public void runOpMode() {
        motorFR = hardwareMap.get(DcMotor.class, "FrontRight");
        motorFL = hardwareMap.get(DcMotor.class, "FrontLeft");
        motorBR = hardwareMap.get(DcMotor.class, "BackRight");
        motorBL = hardwareMap.get(DcMotor.class, "BackLeft");
        motorFR.setDirection(DcMotor.Direction.REVERSE);
        motorFL.setDirection(DcMotor.Direction.FORWARD);
        motorBR.setDirection(DcMotor.Direction.REVERSE);
        motorBL.setDirection(DcMotor.Direction.FORWARD);
        waitForStart();
        while (opModeIsActive()) {
            SetWheels(1,0,0,0); StopWheels(500);
            SetWheels(0,1,0,0); StopWheels(500);
            SetWheels(0,0,1,0); StopWheels(500);
            SetWheels(0,0,0,1); StopWheels(500);
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

    private void MoveRobot(double forward, double right) {
        SetWheels(forward + right, forward - right, forward - right, forward + right);
    } private void MoveRobot(double forward, double right, long time) { MoveRobot(forward, right); StopWheels(time);}

}
