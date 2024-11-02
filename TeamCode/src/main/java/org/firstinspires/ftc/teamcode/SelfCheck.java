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
    Servo servo = null;
    @Override
    public void runOpMode() {
        motor = hardwareMap.get(DcMotor.class, "Arm1");
        servo = hardwareMap.get(Servo.class, "Arm2");
        waitForStart();
        while (opModeIsActive()) {
            double Arm2Pos = gamepad1.right_trigger;
            servo.setPosition(Arm2Pos);
            telemetry.addData("Arm 1 Position (Motor)", motor.getCurrentPosition());
            telemetry.addData("Arm 2 Position (Servo)", Arm2Pos);
            telemetry.update();
        }
    }
}
