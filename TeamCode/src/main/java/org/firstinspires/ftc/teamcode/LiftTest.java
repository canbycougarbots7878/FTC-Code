package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="Lifter Test", group="Linear OpMode")
public class LiftTest extends LinearOpMode{

    private DcMotor Lift_Motor = null;
    @Override
    public void runOpMode() {
        Lift_Motor = hardwareMap.get(DcMotor.class, "Lifter");

        waitForStart();

        while (opModeIsActive()) {
            Lift_Motor.setPower(gamepad2.left_trigger);
        }
    }
}
