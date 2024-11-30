package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "motorSpeedTest", group = "Concept")
public class motorSpeedTest extends LinearOpMode {
    DcMotor motorFL = null;
    DcMotor motorBL = null;
    DcMotor motorFR = null;
    DcMotor motorBR = null;
    @Override
    public void runOpMode() {
        motorFR = hardwareMap.get(DcMotor.class, "FrontRight");
        motorBR = hardwareMap.get(DcMotor.class, "BackRight");
        motorBL = hardwareMap.get(DcMotor.class, "BackLeft");
        motorFL = hardwareMap.get(DcMotor.class, "FrontLeft");

        motorFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motorFR.setDirection(DcMotor.Direction.REVERSE);


        motorFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();
        while (opModeIsActive()) {

            TelemetryPosition();

            motorFR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motorBR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motorBL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motorFL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


            motorFR.setPower(1);
            motorBR.setPower(1);
            motorBL.setPower(1);
            motorFL.setPower(1);

            telemetry.update();
        }
    }

    private void TelemetryPosition() {
        telemetry.addData("Front Right", motorFR.getCurrentPosition());
        telemetry.addData("Back Right", motorBR.getCurrentPosition());
        telemetry.addData("Back Left", motorBL.getCurrentPosition());
        telemetry.addData("Front Left", motorFL.getCurrentPosition());
    }

}
