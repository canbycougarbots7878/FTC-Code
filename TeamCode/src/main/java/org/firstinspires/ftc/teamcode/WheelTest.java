package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
@Disabled
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
        motorFR.setDirection(DcMotor.Direction.FORWARD);
        motorFL.setDirection(DcMotor.Direction.REVERSE);
        motorBR.setDirection(DcMotor.Direction.FORWARD);
        motorBL.setDirection(DcMotor.Direction.REVERSE);
        waitForStart();
        while (opModeIsActive()) {
            Display("[-] [#]\n[-] [-]"); SetWheels(1,0,0,0); StopWheels(500);
            Display("[#] [-]\n[-] [-]"); SetWheels(0,1,0,0); StopWheels(500);
            Display("[-] [-]\n[-] [#]"); SetWheels(0,0,1,0); StopWheels(500);
            Display("[-] [-]\n[#] [-]"); SetWheels(0,0,0,1); StopWheels(500);
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
}
