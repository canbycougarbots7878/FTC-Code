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

        DcMotor arm1 = hardwareMap.get(DcMotor.class, "Arm1");
        Servo arm2 = hardwareMap.get(Servo.class, "Arm2");
        //CRServo clawRotation = hardwareMap.get(CRServo.class, "clawRotation");
        CRServo claw = hardwareMap.get(CRServo.class, "claw");

		// Set each motor's direction
        motorFR.setDirection(DcMotor.Direction.FORWARD);
        motorBR.setDirection(DcMotor.Direction.FORWARD);
        motorBL.setDirection(DcMotor.Direction.REVERSE);
        motorFL.setDirection(DcMotor.Direction.REVERSE);

        arm1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();

        while (opModeIsActive()) {

            double axial   = -gamepad1.left_stick_y;
            double lateral = gamepad1.left_stick_x;
            double yaw     = gamepad1.right_stick_x;
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
            telemetry.addData("Arm1 Position", arm1.getCurrentPosition());
            arm2.setPosition(Arm2pos);
            telemetry.addData("Arm2 Position", Arm2pos);
            String mode = "Home";

            if (gamepad1.a) { mode = "Home"; }
            if (gamepad1.b) { mode = "Deploy"; }
            if (gamepad1.x) { mode = "Basket"; }
            if (gamepad1.y) { mode = "Reach"; }
            if (gamepad1.dpad_up) { mode = "Manual"; }
            if (mode.equals("Home")) {
                arm1.setTargetPosition(0);
                arm1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                arm1.setPower(.1);
                arm2.setPosition(0);
            }
            if (mode.equals("Deploy")) {
                arm1.setTargetPosition(-400);
                arm1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                arm1.setPower(.1);
                arm2.setPosition(0.69);
            }
            if (mode.equals("Basket")) {
                arm1.setTargetPosition(-1092);
                arm1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                arm1.setPower(1);
            }
            if (mode.equals("Reach")) {
                arm1.setTargetPosition(-1092);
                arm1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                arm1.setPower(1);
            }
            if (mode.equals("Manual")) {
                arm1.setTargetPosition(arm1.getTargetPosition() + (int)gamepad1.left_trigger);
                arm2.setPosition(gamepad1.right_trigger);
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
