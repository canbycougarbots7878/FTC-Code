// March 18, 2025, 5:13- Simon Nation

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "COMPETITION: Auto Drive", group = "Concept")
public class auto extends LinearOpMode {
    DcMotor motorFL = null;
    DcMotor motorBL = null;
    DcMotor motorFR = null;
    DcMotor motorBR = null;

    double accelFactor = 0.02;  // Acceleration increment per loop
    double decelFactor = accelFactor;  // Deceleration decrement per loop

    double millimeterConstant = 2.337349112 * Math.pow(10, -4);
    double degreeConstant = 4.740853373 * Math.pow(10, -5);

    double gearRatio = 19.2/1;

    double diameterOfWheel = 104;
    double pi = Math.PI;
    double circumferenceOfWheel = pi * diameterOfWheel;

    double currentRobotMillimeterConstant = millimeterConstant * gearRatio * circumferenceOfWheel;
    double currentRobotDegreeConstant = degreeConstant * gearRatio * circumferenceOfWheel;

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

        waitForStart();

        int rest = 100;
        Forward(1000, 0.75);
        sleep(rest);

        while (opModeIsActive()) {
            // Future tasks can be added here
        }
        stopNow();
    }

    private void Forward(double TargetDistance, double MaxPower) {
        double time = TargetDistance / (currentRobotMillimeterConstant * MaxPower);
        double power = 0;  // Start at no power
        long numberOfLoops = 0;
        double distance = 0;

        while (power < MaxPower && distance < TargetDistance/2) {
            power += accelFactor;
            setDrivePower(power);
            sleep(50);
            numberOfLoops += 1;
            distance += (currentRobotMillimeterConstant * 50 * power);
            telemetry.addData("Number of loops:", numberOfLoops);
            telemetry.update();
        }

        if (!(distance >= TargetDistance/2)) {
            double timeSinceStart = numberOfLoops * 50;
            double constantPowerTime = Math.abs(time - (2 * timeSinceStart));
            sleep((long) constantPowerTime);  // Run at max speed for most of the time
            distance += (currentRobotMillimeterConstant * constantPowerTime * power);
        }

        while (power > 0 && distance < TargetDistance) {
            power -= decelFactor;
            setDrivePower(power);
            sleep(50);
            distance += (currentRobotMillimeterConstant * 50 * power);
        }
        stopNow();
    }

    private void Turn(double TargetDegrees, double MaxTurnPower) {
        double time = TargetDegrees / (currentRobotDegreeConstant * MaxTurnPower);
        double power = 0;  // Start at no power
        long numberOfLoops = 0;
        double degree = 0;

        while (power < MaxTurnPower && degree < TargetDegrees/2) {
            power += accelFactor;
            setDrivePower(power);
            sleep(50);
            numberOfLoops += 1;
            degree += (currentRobotDegreeConstant * 50 * power);
            telemetry.addData("Number of loops:", numberOfLoops);
            telemetry.update();
        }

        if (!(degree >= TargetDegrees/2)) {
            double timeSinceStart = numberOfLoops * 50;
            double constantPowerTime = Math.abs(time - (2 * timeSinceStart));
            sleep((long) constantPowerTime);  // Run at max speed for most of the time
            degree += (currentRobotDegreeConstant * constantPowerTime * power);
        }

        while (power > 0 && degree < TargetDegrees) {
            power -= decelFactor;
            setDrivePower(power);
            sleep(50);
            degree += (currentRobotDegreeConstant * 50 * power);
        }
        stopNow();
    }

    private void setDrivePower(double power) {
        motorFR.setPower(power);
        motorBR.setPower(power);
        motorBL.setPower(power);
        motorFL.setPower(power);
    }

    private void stopNow(){
        motorFR.setPower(0);
        motorBR.setPower(0);
        motorBL.setPower(0);
        motorFL.setPower(0);
    }

    private void setTurnPower(double power) {
        motorFR.setPower(-power);
        motorBR.setPower(-power);
        motorBL.setPower(power);
        motorFL.setPower(power);
    }


}
