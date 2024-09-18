package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
//hi
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
@TeleOp(name="Basic: Servo Tester", group="Linear OpMode")
public class BasicOpMode_Linear extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor test_motor = null;
    private Servo test_sareMap.get(Servo.class, "servo");
        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            test_motor.setPower(gamepad1.left_stick_y);
            float Position = (gamepad1.right_trigger - gamepad1.left_trigger + 1) / 2;
            test_servo.setPosition(Position);
        }
    }
}
