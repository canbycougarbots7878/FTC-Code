package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Autonomous(name="Encoder Check", group="Robot")
public class SelfCheck extends LinearOpMode {
    DcMotor slider = null;
    DcMotor arm = null;
    DcMotor FR = null;
    DcMotor FL = null;
    DcMotor BR = null;
    DcMotor BL = null;
    @Override
    public void runOpMode() {
        slider = hardwareMap.get(DcMotor.class, "Slide Arm");
        arm = hardwareMap.get(DcMotor.class, "Extending Arm");
        FR = hardwareMap.get(DcMotor.class, "FrontRight");
        FL = hardwareMap.get(DcMotor.class, "FrontLeft");
        BR = hardwareMap.get(DcMotor.class, "BackRight");
        BL = hardwareMap.get(DcMotor.class, "BackLeft");
        waitForStart();
        while (opModeIsActive()) {
            telemetry.addData("Slider Position (Motor)", slider.getCurrentPosition());
            telemetry.addData("Arm Position (Motor)", arm.getCurrentPosition());
            telemetry.addData("Front Right Position (Motor)", FR.getCurrentPosition());
            telemetry.addData("Front Left Position (Motor)", FL.getCurrentPosition());
            telemetry.addData("Back Right Position (Motor)", BR.getCurrentPosition());
            telemetry.addData("Back Left Position (Motor)", BL.getCurrentPosition());
            telemetry.update();
        }
    }
}
