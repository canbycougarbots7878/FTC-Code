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
    public void runOpMode() {
		// Init all the motors
        DcMotor motorFR = hardwareMap.get(DcMotor.class, "FrontRight");
        DcMotor motorBR = hardwareMap.get(DcMotor.class, "BackRight");
        DcMotor motorBL = hardwareMap.get(DcMotor.class, "BackLeft");
        DcMotor motorFL = hardwareMap.get(DcMotor.class, "FrontLeft");

        DcMotor slidearm = hardwareMap.get(DcMotor.class, "Slide Arm");
        DcMotor arm = hardwareMap.get(DcMotor.class, "Extending Arm");
        //CRServo clawRotation = hardwareMap.get(CRServo.class, "clawRotation");
        Servo wrist = hardwareMap.get(Servo.class, "wrist");
        CRServo claw = hardwareMap.get(CRServo.class, "claw");

		// Set each motor's direction
        motorFR.setDirection(DcMotor.Direction.FORWARD);
        motorBR.setDirection(DcMotor.Direction.FORWARD);
        motorBL.setDirection(DcMotor.Direction.REVERSE);
        motorFL.setDirection(DcMotor.Direction.REVERSE);

        waitForStart();

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

            double Arm2pos = 1 - gamepad1.right_trigger;
            telemetry.addData("Arm1 Position", slidearm.getCurrentPosition());
            //arm.setPosition(Arm2pos);
            telemetry.addData("Arm2 Position", Arm2pos);
            String state = "Home";

            if (gamepad1.a) { state = "Home"; }
            if (gamepad1.b) { state = "Deploy"; }
            if (gamepad1.x) { state = "Basket"; }
            if (gamepad1.y) { state = "Reach"; }
            if (gamepad1.start) { state = "Manual"; }
            if (gamepad1.dpad_up) { slidearm.setPower(-1); }
            else if (gamepad1.dpad_down) { slidearm.setPower(1); }
            else { slidearm.setPower(0); }
            if (gamepad1.dpad_left) { wrist.setPosition(1); }
            else { wrist.setPosition(0); }
            switch(state) {
                case("Home"):
                    break;
                case("Deploy"):
                    break;
                case("Basket"):
                    break;
                case("Reach"):
                    break;
                case("Manual"):
                    break;
            }
            if (gamepad1.left_bumper) {
                claw.setPower(1);

            }
            else if (gamepad1.right_bumper) {
                claw.setPower(-1);
            }
            else {
                claw.setPower(0);
            }



            telemetry.update();
        }
    }
}
