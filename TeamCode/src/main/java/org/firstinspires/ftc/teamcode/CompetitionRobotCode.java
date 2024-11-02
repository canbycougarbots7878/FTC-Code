package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
//TODO:
// - ARM1 Positions using SelfCheck.java
// - ARM2 Positions
// -
@TeleOp(name = "COMPETITION: Manual Drive", group = "Concept")
public class CompetitionRobotCode extends LinearOpMode {
    private DcMotor motorFL, motorBL, motorFR, motorBR;
    private DcMotor Arm1;
    private Servo Arm2;
    private Servo Grabber;
    public void runOpMode() {
        motorFL = hardwareMap.get(DcMotor.class, "FrontLeft");
        motorBL = hardwareMap.get(DcMotor.class, "BackLeft");
        motorFR = hardwareMap.get(DcMotor.class, "FrontRight");
        motorBR = hardwareMap.get(DcMotor.class, "BackRight");

        Arm1 = hardwareMap.get(DcMotor.class, "Arm1");
        Arm2 = hardwareMap.get(Servo.class, "Arm2");
        Grabber = hardwareMap.get(Servo.class, "Grabber");
        motorFR.setDirection(DcMotor.Direction.FORWARD);
        motorFL.setDirection(DcMotor.Direction.REVERSE);
        motorBR.setDirection(DcMotor.Direction.FORWARD);
        motorBL.setDirection(DcMotor.Direction.REVERSE);

        Arm1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Arm1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();

        while (!gamepad1.start){

        }

        while (opModeIsActive()) {

            double axial = -gamepad1.left_stick_y;
            double lateral = gamepad1.left_stick_x;
            double yaw = gamepad1.right_stick_x;
            double speed = .5;
            if (gamepad1.right_bumper) {
                speed = 1;
            } else if (gamepad1.left_bumper) {
                speed = .25;
            }

            double leftFrontPower = axial + lateral - yaw;
            double rightFrontPower = axial - lateral + yaw;
            double leftBackPower = axial - lateral - yaw;
            double rightBackPower = axial + lateral + yaw;

            motorFR.setPower(speed * leftFrontPower);
            motorFL.setPower(speed * rightFrontPower);
            motorBR.setPower(speed * leftBackPower);
            motorBL.setPower(speed * rightBackPower);
            double Arm2pos = 1 - gamepad1.right_trigger;
            telemetry.addData("Arm1 Position", Arm1.getCurrentPosition());
            Arm2.setPosition(Arm2pos);
            telemetry.addData("Arm2 Position", Arm2pos);
            if (gamepad1.a) {
                // Home |\|#
                Arm1.setTargetPosition(0);
                Arm1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                Arm1.setPower(.1);
                Arm2.setPosition(0);
            }
            if (gamepad1.b) {
                // Collect |\_#
                Arm1.setTargetPosition(-400);
                Arm1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                Arm1.setPower(.1);
                Arm2.setPosition(0.69);
            }
            if (gamepad1.x) {
                //               _
                //              / #
                // Lower basket |
                Arm1.setTargetPosition(-1092);
                Arm1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                Arm1.setPower(1);
            }
            if (gamepad1.y) {
                //                   |#
                //                   |
                //Arm all the way up |
                Arm1.setTargetPosition(-1092);
                Arm1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                Arm1.setPower(1);
            }
            if (gamepad1.left_bumper) {
                Grabber.setPosition(0);

            }
            if (gamepad1.right_bumper) {
                Grabber.setPosition(1);
            }
            telemetry.update();
        }
    }
}
