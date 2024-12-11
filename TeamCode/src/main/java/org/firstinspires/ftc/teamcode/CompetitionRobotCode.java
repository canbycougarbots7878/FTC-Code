package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
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
        Servo wrist = hardwareMap.get(Servo.class, "wrist");
        Servo claw = hardwareMap.get(Servo.class, "claw");
		// Set each motor's direction
        motorFR.setDirection(DcMotor.Direction.FORWARD);
        motorBR.setDirection(DcMotor.Direction.FORWARD);
        motorBL.setDirection(DcMotor.Direction.REVERSE);
        motorFL.setDirection(DcMotor.Direction.REVERSE);
        //
        slidearm.setTargetPosition(100);
        slidearm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //slidearm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slidearm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        int segment = 0;
        String state = "Manual";
        waitForStart();
        arm.setPower(.5);
        sleep(3000);
        arm.setPower(0);
        while (opModeIsActive()) {

            double axial   = gamepad1.left_stick_y;
            double lateral = gamepad1.left_stick_x;
            double yaw     = -gamepad1.right_stick_x;
            double speed   = .5;

			// Setup quick speed modifications for precision driving
            if (gamepad1.right_bumper) {
                speed = 1;
            } else if (gamepad1.left_bumper) {
                speed = .25;
            }

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
            if (gamepad2.a) { state = "Home"; }
            if (gamepad2.b) { state = "Deploy"; }
            if (gamepad2.x) { state = "Basket"; }
            if (gamepad2.y) { state = "Reach"; }
            if (gamepad2.start) { state = "Manual"; }
            if (gamepad2.dpad_left) { wrist.setPosition(1); }
            else { wrist.setPosition(0); }
            //arm.setPower(.3);
            switch(state) {
                case("Home"):
                    SetSliderSegment(0);
                    SetArmPosition(0);
                    break;
                case("Deploy"):
                    SetSliderSegment(0);
                    SetArmPosition(-70);
                    break;
                case("Basket"):
                    SetSliderSegment(2);
                    break;
                case("Reach"):
                    SetSliderSegment(3);
                    break;
                case("Manual"):
                    SetSliderSegment(4);
                    break;
            }

            if (gamepad2.left_bumper) {
                claw.setPosition(.4);
            }
            else if (gamepad2.right_bumper) {
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
            case(0): SetSliderPosition(100); break;
            case(1): SetSliderPosition(1437); break;
            case(2): SetSliderPosition(2971); break;
            case(3): SetSliderPosition(4498); break;
            case(4): SetSliderPosition(6025); break;
        }
    }
    private void SetSliderPosition(int target) {
        slidearm.setTargetPosition(target);
        slidearm.setPower(.5);
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
}
