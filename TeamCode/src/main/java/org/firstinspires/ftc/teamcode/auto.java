// February 8, 2025, 11:02 - Simon N.

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "COMPETITION: Auto Drive", group = "Concept")
public class auto extends LinearOpMode {
    DcMotor motorFL = null;
    DcMotor motorBL = null;
    DcMotor motorFR = null;
    DcMotor motorBR = null;

    DcMotor arm      = null;
    DcMotor slidearm = null;

    Servo claw  = null;
    Servo wrist = null;

    public void runOpMode() {
        motorFR = hardwareMap.get(DcMotor.class, "FrontRight");
        motorBR = hardwareMap.get(DcMotor.class, "BackRight");
        motorBL = hardwareMap.get(DcMotor.class, "BackLeft");
        motorFL = hardwareMap.get(DcMotor.class, "FrontLeft");

        slidearm = hardwareMap.get(DcMotor.class, "Slide Arm");
        arm = hardwareMap.get(DcMotor.class, "Extending Arm");

        wrist = hardwareMap.get(Servo.class, "Wrist");
        claw = hardwareMap.get(Servo.class, "Claw");

        motorFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motorFR.setDirection(DcMotor.Direction.REVERSE);
        motorBR.setDirection(DcMotor.Direction.REVERSE);
        motorBL.setDirection(DcMotor.Direction.FORWARD);
        motorFL.setDirection(DcMotor.Direction.FORWARD);

        slidearm.setTargetPosition(100);
        slidearm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slidearm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();

        int rest = 100;  // Reduced unnecessary wait times

        Home();
        Forward(530, 0.75);
        sleep(rest);

        Turn(-90, 0.75);  // Using degrees for clarity
        sleep(rest);

        Forward(304.8, 0.75);
        sleep(rest);

        // Start slider movement while turning
        SetSliderPosition(1939);
        Turn(90, 0.75);
        sleep(rest * 7);

        Forward(700, 0.5);  // Increased speed from 0.25 to 0.5
        sleep(rest);

        SetSliderPosition(1231);
        sleep(rest / 2);

        claw.setPosition(0.75);
        sleep(rest);

        Home();
        sleep(rest);

        Forward(-70, 0.75);
        sleep(rest / 2);

        Turn(-90, 0.75);
        sleep(rest / 2);

        Forward(11200, 0.75);
        sleep(rest);

        Turn(90, 0.75);
        sleep(rest);

        Forward(140, 0.5);  // Increased speed from 0.25 to 0.5
        sleep(rest / 2);

        Turn(90, 0.75);
        sleep(rest);

        // Start slider movement while moving forward
        SetSliderPosition(1939);
        Forward(80, 0.5);  // Increased speed from 0.25 to 0.5
        sleep(rest);

        SetSliderPosition(1157);

        while (opModeIsActive()) {} // Keep program alive
    }

    private void SetSliderPosition(int target) {
        int currentPosition = slidearm.getCurrentPosition();
        int distance = target - currentPosition;
        double speed = SmoothSliderSpeed(Math.abs(distance));
        slidearm.setPower(speed);
        slidearm.setTargetPosition(target);
    }

    private double SmoothSliderSpeed(int distance) {
        // Using exponential easing to smooth slider speed
        double maxSpeed = 1.0;
        double exponent = 2.0;  // Adjust the exponent to control the smoothness
        return maxSpeed * Math.pow(distance / 1000.0, exponent);  // Normalize the distance
    }

    private void SetSliderSegment(int segment) {
        // Modularized function for selecting slider positions based on segments
        switch(segment) {
            case(0): SetSliderPosition(100); break;
            case(1): SetSliderPosition(1437); break;
            case(2): SetSliderPosition(2971); break;
            case(3): SetSliderPosition(4498); break;
            case(4): SetSliderPosition(6025); break;
        }
    }

    private void SetArmPosition(int position) {
        boolean direction = (arm.getCurrentPosition() < position);
        int proximity = Math.abs(arm.getCurrentPosition() - position);
        double speed = ProximityRunge(proximity);
        if(direction) {
            arm.setPower(speed);
        }
        else {
            arm.setPower(-speed);
        }
    }

    private double ProximityRunge(int position) {
        double x = (double)position / 4;
        return 1 - 1/(1+x*x);
    }

    private void Forward(double TargetDistance, double MaxPower) {
        MoveWithRamp(TargetDistance, MaxPower, true);  // Forward movement with ramping
    }

    private void Turn(double TargetDegrees, double MaxTurnPower) {
        MoveWithRamp(TargetDegrees, MaxTurnPower, false);  // Turning with ramping
    }

    private void MoveWithRamp(double targetDistance, double maxPower, boolean isForward) {
        double totalTime = Math.abs(targetDistance / 1.528 / maxPower);
        totalTime = Math.max(totalTime, 500);  // Ensure minimum movement time
        double accelTime = Math.min(totalTime * 0.3, 500);  // Cap acceleration to 500ms
        double decelTime = accelTime;
        double cruiseTime = totalTime - (accelTime + decelTime);

        moveRamp(accelTime, maxPower, targetDistance, isForward);  // Separate function for ramp-up
        if (cruiseTime > 0) {
            setDrivePower(maxPower * Math.signum(targetDistance)); // Constant speed
            sleep((long) cruiseTime);
        }
        moveRamp(decelTime, maxPower, targetDistance, isForward);  // Separate function for ramp-down
    }

    private void moveRamp(double rampTime, double maxPower, double targetDistance, boolean isForward) {
        for (double t = 0; t < rampTime; t += 50) {
            double power = (t / rampTime) * maxPower;
            setDrivePower(isForward ? power : -power);  // Forward or backward
            sleep(50);
        }
    }

    // Helper function for setting motor power
    private void setDrivePower(double power) {
        motorFR.setPower(power);
        motorBR.setPower(power);
        motorBL.setPower(power);
        motorFL.setPower(power);
    }

    private void setTurnPower(double power) {
        motorFR.setPower(-power);
        motorBR.setPower(-power);
        motorBL.setPower(power);
        motorFL.setPower(power);
    }

    private void Home() {
        SetSliderSegment(0);
        SetArmPosition(0);
    }

    private void MoveClawToPosition(double position) {
        claw.setPosition(position);  // Set servo position gradually
        sleep(100);  // Give time for the servo to reach the target position
    }

    private void StopMotors() {
        motorFR.setPower(0);
        motorBR.setPower(0);
        motorBL.setPower(0);
        motorFL.setPower(0);
        sleep(50);
    }
}