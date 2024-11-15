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
    DcMotor motor = null;
    @Override
    public void runOpMode() {
        motor = hardwareMap.get(DcMotor.class, "Arm1");
        waitForStart();
        while (opModeIsActive()) {
            telemetry.addData("Arm 1 Position (Motor)", motor.getCurrentPosition());
            telemetry.update();
        }
    }
}
