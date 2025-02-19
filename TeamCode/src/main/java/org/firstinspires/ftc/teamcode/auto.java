// February 18, 2025, 5:40- Simon Nation

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
    double decelFactor = 0.02;  // Deceleration decrement per loop

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

        slidearm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slidearm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();

        int rest = 100;
        Home();
        Forward(530, 0.75);
        sleep(rest);

        while (opModeIsActive()) {
            // Future tasks can be added here
        }
        stopNow();
    }

    private void Forward(double TargetDistance, double MaxPower) {
        double time = TargetDistance / (1.528 * MaxPower);
        double power = 0.1;  // Start at low power

        while (power < MaxPower) {
            setDrivePower(power);
            power += accelFactor;
            sleep(50);
        }

        sleep((long) (time - (time * 0.3)));  // Run at max speed for most of the time

        while (power > 0) {
            setDrivePower(power);
            power -= decelFactor;
            sleep(50);
        }
        stopNow();
    }

    private void Turn(double TargetDegrees, double MaxTurnPower) {
        double time = TargetDegrees / (0.3139 * MaxTurnPower);
        double power = 0.1;

        while (power < MaxTurnPower) {
            setTurnPower(power);
            power += accelFactor;
            sleep(50);
        }

        sleep((long) (time - (time * 0.3)));

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
