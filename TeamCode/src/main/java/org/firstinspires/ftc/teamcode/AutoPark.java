package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import java.util.Set;

@Autonomous(name = "Autonomous", group = "Concept")
public class AutoPark extends LinearOpMode {
    private DcMotor motorFR = null;
    private DcMotor motorFL = null;
    private DcMotor motorBR = null;
    private DcMotor motorBL = null;
    @Override
    public void runOpMode() {
        motorFL = hardwareMap.get(DcMotor.class, "FrontLeft");
        motorBL = hardwareMap.get(DcMotor.class, "BackLeft");
        motorFR = hardwareMap.get(DcMotor.class, "FrontRight");
        motorBR = hardwareMap.get(DcMotor.class, "BackRight");

        motorFR.setDirection(DcMotor.Direction.REVERSE);
        motorFL.setDirection(DcMotor.Direction.FORWARD);
        motorBR.setDirection(DcMotor.Direction.REVERSE);
        motorBL.setDirection(DcMotor.Direction.FORWARD);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        MoveDistance(10);
        //Stop();
    }
    private void SetMotors(double FR, double FL, double BR, double BL) {
        motorFR.setPower(FR);
        motorFL.setPower(FL);
        motorBR.setPower(BR);
        motorBL.setPower(BL);
    }
    private void StopMotors() {
        SetMotors(0,0,0,0);
    }
    private void Stop() {
        DcMotor stopprogram = hardwareMap.get(DcMotor.class, "Program Succesfully Stopped");
    }
    private void MoveDistance(double distance) {
        motorFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        long time;
        double DISTANCECONSTANT = 5.5;
        time = (long) ((500 * Math.abs(distance)) / DISTANCECONSTANT);
        if (distance > 0) {
            SetMotors(.5,.5,.5,.5);
        }
        if (distance < 0) {
            SetMotors(-.5,-.5,-.5,-.5);
        }
        sleep(time);
        StopMotors();
    }
}