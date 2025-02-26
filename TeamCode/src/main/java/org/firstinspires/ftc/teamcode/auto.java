// February 25, 2025, 5:32- Simon Nation

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

    DcMotor arm = null;
    DcMotor slidearm = null;

    Servo claw = null;
    Servo wrist = null;

    double accelFactor = 0.02;  // Acceleration increment per loop
    double decelFactor = accelFactor;  // Deceleration decrement per loop

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

        slidearm.setTargetPosition(0);
        slidearm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slidearm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();

        int rest = 100;
        Home();
        Forward(1000, 0.75);
        sleep(rest);

        while (opModeIsActive()) {
            // Future tasks can be added here
        }
        stopNow();
    }

    private void Forward(double TargetDistance, double MaxPower) {
        double time = TargetDistance / (1.528 * MaxPower);
        double power = 0;  // Start at no power
        long numberOfLoops = 0;
        double distance = 0;

        while (power < MaxPower && distance < TargetDistance/2) {
            power += accelFactor;
            setDrivePower(power);
            sleep(50);
            numberOfLoops += 1;
            distance += (1.528 * 50 * power);
        }

        if (!(distance >= TargetDistance/2)) {
            double timeSinceStart = numberOfLoops * 50;
            double constantPowerTime = Math.abs(time - (2 * timeSinceStart));
            sleep((long) constantPowerTime);  // Run at max speed for most of the time
            distance += (1.528 * constantPowerTime * power);
        }

        while (power > 0 && distance < TargetDistance) {
            power -= decelFactor;
            setDrivePower(power);
            sleep(50);
            distance += (1.528 * 50 * power);
        }
        stopNow();
    }

    private void Turn(double TargetDegrees, double MaxTurnPower) {
        double time = TargetDegrees / (0.3139 * MaxTurnPower);
        double power = 0.1;
        long numberOfLoops = 0;

        while (power < MaxTurnPower) {
            setTurnPower(power);
            power += accelFactor;
            sleep(50);
            numberOfLoops += 1;
        }

        setTurnPower(MaxTurnPower);
        sleep(Math.abs((long) (time - (numberOfLoops * 2 * 50))));

        while (power > 0) {
            setTurnPower(power);
            power -= decelFactor;
            sleep(50);
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

    private void Home() {
        SetSliderPosition(100);
        SetArmPosition(0);
    }

    private void SetSliderPosition(int target) {
        slidearm.setTargetPosition(target);
        if (slidearm.getCurrentPosition() != target) {
            slidearm.setPower(1.0);
        } else {
            slidearm.setPower(0);
        }
    }

    private void SetArmPosition(int position) {
        arm.setTargetPosition(position);
        if (arm.getCurrentPosition() != position) {
            arm.setPower(1.0);
        } else {
            arm.setPower(0);
        }
    }
}
