package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
@TeleOp(name = "Arm Testing", group = "Concept")
public class Testing extends LinearOpMode {
    public void runOpMode() {
        DcMotor slider = hardwareMap.get(DcMotor.class, "Slide Arm");
        DcMotor arm = hardwareMap.get(DcMotor.class, "Extending Arm");
        waitForStart();
        while(opModeIsActive()) {
            slider.setPower(-.3);
            arm.setPower(.5);
        }
    }
}
