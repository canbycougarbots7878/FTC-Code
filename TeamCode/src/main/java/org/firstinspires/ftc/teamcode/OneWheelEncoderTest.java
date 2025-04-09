//January 14, 2025 - Simon Nation

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

//TODO

//Notes
// 1. 90 degrees counterclockwise is 450 mm

@Disabled
@TeleOp(name = "OneWheelEncoderTest", group = "Concept")
public class OneWheelEncoderTest extends LinearOpMode {

    DcMotor motorFL = null;

    public void runOpMode() {
        motorFL = hardwareMap.get(DcMotor.class, "FrontLeft");

        motorFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motorFL.setDirection(DcMotor.Direction.FORWARD);

        motorFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();
        telemetry.addData("Start", 11);
        telemetry.update();
        while (opModeIsActive()) {
            telemetry.addData("Front Left", motorFL.getCurrentPosition());
            telemetry.update();
        }
    }
}