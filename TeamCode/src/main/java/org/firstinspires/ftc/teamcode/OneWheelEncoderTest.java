//January 14, 2025 - Simon Nation

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

//TODO

//Notes
// 1. 90 degrees counterclockwise is 450 mm


@TeleOp(name = "OneWheelEncoderTest", group = "Concept")
public class OneWheelEncoderTest extends LinearOpMode {

    DcMotor motorFR = null;

    public void runOpMode() {
        motorFR = hardwareMap.get(DcMotor.class, "FrontRight");

        motorFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motorFR.setDirection(DcMotor.Direction.FORWARD);

        motorFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();
        telemetry.addData("Start", 11);
        telemetry.update();
        while (opModeIsActive()) {
            telemetry.addData("Front Right", motorFR.getCurrentPosition());
            telemetry.update();
        }
    }
}