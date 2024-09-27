package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
@TeleOp(name = "Servo Test", group = "Concept")
public class ServoTest extends LinearOpMode{
    private Servo servo = null;
    private ElapsedTime t = null;
    public void runOpMode() {
        servo = hardwareMap.get(Servo.class, "servo");
        waitForStart();
        ElapsedTime t = new ElapsedTime();
        while (opModeIsActive()) {
            servo.setPosition(-1);
        }
    }
}
