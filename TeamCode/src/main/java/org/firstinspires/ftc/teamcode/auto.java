//February 8, 2025 - Simon N.

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
        sleep(rest * 6);
    
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
    
        Forward(1100, 0.75);
        sleep(rest);
    
        Turn(90, 0.75);
        sleep(rest);
    
        Forward(140, 0.5);  // Increased speed from 0.25 to 0.5
        sleep(rest / 2);
    
        Turn(90, 0.75);
        sleep(rest);
    
        // Start slider movement while moving forward
        SetSliderPosition(1939);
        Forward(70, 0.5);  // Increased speed from 0.25 to 0.5
        sleep(rest);
    
        SetSliderPosition(1157);

        while (opModeIsActive()) {} // Keep program alive
    }


    private void SetSliderPosition(int target) {
        slidearm.setTargetPosition(target);
        slidearm.setPower(.5);
    }

    private void SetSliderSegment(int segment) {
        //slidearm.setPower(1);
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
        double totalTime = Math.abs(TargetDistance / 1.528 / MaxPower);

        // Ensure minimum movement time
        if (totalTime < 500) totalTime = 500;

        double accelTime = Math.min(totalTime * 0.3, 500);  // Cap acceleration to 500ms
        double decelTime = accelTime; // Symmetric deceleration
        double cruiseTime = totalTime - (accelTime + decelTime);

        // Acceleration phase (Quadratic ramp)
        for (double t = 0; t < accelTime; t += 50) {
            double power = (t / accelTime) * MaxPower;  // Smooth acceleration
            setDrivePower(power * Math.signum(TargetDistance));
            sleep(50);
        }

        // Constant speed phase
        if (cruiseTime > 0) {
            setDrivePower(MaxPower * Math.signum(TargetDistance));
            sleep((long) cruiseTime);
        }

        // Deceleration phase (Quadratic ramp down)
        for (double t = decelTime; t >= 0; t -= 50) {
            double power = (t / decelTime) * MaxPower;
            setDrivePower(power * Math.signum(TargetDistance));
            sleep(50);
        }

        StopMotors();
    }

    // Helper function for setting motor power
    private void setDrivePower(double power) {
        motorFR.setPower(power);
        motorBR.setPower(power);
        motorBL.setPower(power);
        motorFL.setPower(power);
    }

    private void Turn(double TargetDegrees, double MaxTurnPower) {
        double turnSpeedFactor = 0.01; // Adjust this empirically based on robot testing
        double totalTime = Math.abs(TargetDegrees * turnSpeedFactor / MaxTurnPower);
    
        if (totalTime < 400) totalTime = 400; // Reduced minimum time
    
        double accelTime = Math.min(totalTime * 0.3, 400);
        double decelTime = accelTime;
        double cruiseTime = totalTime - (accelTime + decelTime);
        
        // Acceleration phase
        for (double t = 0; t < accelTime; t += 50) {
            double power = (t / accelTime) * MaxTurnPower;
            setTurnPower(power * Math.signum(TargetDegrees));
            sleep(50);
        }
        
        // Constant turn phase
        if (cruiseTime > 0) {
            setTurnPower(MaxTurnPower * Math.signum(TargetDegrees));
            sleep((long) cruiseTime);
        }
        
        // Deceleration phase
        for (double t = decelTime; t >= 0; t -= 50) {
            double power = (t / decelTime) * MaxTurnPower;
            setTurnPower(power * Math.signum(TargetDegrees));
            sleep(50);
        }
        
        StopMotors();
    }

    // Helper function for setting turn power
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

    private void StopMotors() {
        motorFR.setPower(0);
        motorBR.setPower(0);
        motorBL.setPower(0);
        motorFL.setPower(0);
        sleep(50);
    }

}
