package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;

//TODO:
// - ARM1 Positions using SelfCheck.java
// - ARM2 Positions
// -
@TeleOp(name = "COMPETITION: Manual Drive", group = "Competition")
public class CompetitionRobotCode extends LinearOpMode {
    DcMotor arm = null;
    DcMotor slidearm = null;
    public void runOpMode() {
		// Init all the motors
        DcMotor motorFR = hardwareMap.get(DcMotor.class, "FrontRight");
        DcMotor motorBR = hardwareMap.get(DcMotor.class, "BackRight");
        DcMotor motorBL = hardwareMap.get(DcMotor.class, "BackLeft");
        DcMotor motorFL = hardwareMap.get(DcMotor.class, "FrontLeft");

        slidearm = hardwareMap.get(DcMotor.class, "Slide Arm");
        arm = hardwareMap.get(DcMotor.class, "Extending Arm");
        Servo wrist = hardwareMap.get(Servo.class, "Wrist");
        Servo claw = hardwareMap.get(Servo.class, "Claw");
        Servo ArmLock = hardwareMap.get(Servo.class, "Arm Lock");
		// Set each motor's direction
        motorFR.setDirection(DcMotor.Direction.FORWARD);
        motorBR.setDirection(DcMotor.Direction.FORWARD);
        motorBL.setDirection(DcMotor.Direction.REVERSE);
        motorFL.setDirection(DcMotor.Direction.REVERSE);
        motorFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //
        slidearm.setTargetPosition(100);
        slidearm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slidearm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        int segment = 0;
        String state = "Home";
        waitForStart();
        /*arm.setPower(.5);
        sleep(3000);
        arm.setPower(0);*/
        arm.setTargetPosition(10);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        ArmLock.setPosition(.5);
        double speed   = .5;
        while (opModeIsActive()) {

            double axial   = gamepad1.left_stick_y;
            double lateral = gamepad1.left_stick_x;
            double yaw     = -gamepad1.right_stick_x;

			// Setup quick speed modifications for precision driving

            double rightFrontPower = axial - lateral + yaw;
            double rightBackPower  = axial + lateral + yaw;
            double leftBackPower   = axial - lateral - yaw;
            double leftFrontPower  = axial + lateral - yaw;

            motorFR.setPower(speed * leftFrontPower);
            motorBR.setPower(speed * leftBackPower);
            motorBL.setPower(speed * rightBackPower);
            motorFL.setPower(speed * rightFrontPower);

            //telemetry.addData("Slide Position", slidearm.getCurrentPosition());
            telemetry.addData("Slide Position", slidearm.getCurrentPosition());
            telemetry.addData("Arm Position", arm.getCurrentPosition());
            if (gamepad1.a) { state = "Home"; }
            if (gamepad1.b) { state = "Deploy"; }
            if (gamepad1.x) { state = "Basket"; }
            if (gamepad1.y) { state = "Reach"; }
            if (gamepad1.start) { state = "Manual"; }
            if (gamepad1.dpad_left) { wrist.setPosition(0.3333333); }
            else if(gamepad1.dpad_right) { wrist.setPosition(0); }

            //arm.setPower(.3);
            switch(state) {
                case("Home"):
                    SetSliderSegment(0);
                    SetArmPosition(10);
                    if(arm.getCurrentPosition() < 50) speed = .5;
                    break;
                case("Deploy"):
                    SetSliderSegment(0);
                    SetArmPosition(-60);
                    if(arm.getCurrentPosition() < 50) speed = .5;
                    break;
                case("Basket"):
                    SetSliderSegment(2);
                    speed = 0.25;
                    break;
                case("Reach"):
                    SetSliderSegment(3);
                    speed = 0.125;
                    break;
                case("Manual"):
                    SetSliderSegment(4);
                    speed = 0.125;
                    break;
            }

            if (gamepad1.left_bumper) {
                claw.setPosition(.4);
            }
            else if (gamepad1.right_bumper) {
                claw.setPosition(.5);
            }
            else {
                claw.setPosition(0);
            }

            telemetry.update();
        }
    }

    private double ProximityRunge(int position) {
        double x = (double)position / 4;
        return 1 - 1/(1+x*x);
    }
    private void SetSliderSegment(int segment) {
        //slidearm.setPower(1);
        switch(segment) {
            case(0): SetSliderPosition(0); break;
            case(1): SetSliderPosition(1437); break;
            case(2): SetSliderPosition(2971); break;
            case(3): SetSliderPosition(4498); break;
            case(4): SetSliderPosition(5900); break;
        }
    }
    private void SetSliderPosition(int target) {
        slidearm.setTargetPosition(target);
        slidearm.setPower(.5);
        if(target == 0 && slidearm.getCurrentPosition() < 100) {
            slidearm.setPower(0);
        }
    }
    private void SetArmPosition(int target) {
        int x = arm.getCurrentPosition() - target;
        double speed = 1 / (1 + Math.pow(30, x));

        telemetry.addData("Speed", speed);
        arm.setPower(speed);
    }
}
