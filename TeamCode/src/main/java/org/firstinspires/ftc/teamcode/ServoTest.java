package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import java.util.List;
import java.util.ListIterator;

@TeleOp(name = "Not Test", group = "Concept")
public class ServoTest extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();

    private DcMotor motorFR = null;
    private DcMotor motorFL = null;
    private DcMotor motorBR = null;
    private DcMotor motorBL = null;
    private Servo   Servo0  = null;
    List<Servo> Servos = null;

    @Override
    public void runOpMode() {
        motorFL = hardwareMap.get(DcMotor.class, "FrontLeft");
        motorBL = hardwareMap.get(DcMotor.class, "BackLeft");
        motorFR = hardwareMap.get(DcMotor.class, "FrontRight");
        motorBR = hardwareMap.get(DcMotor.class, "BackRight");
        Servo0  = hardwareMap.get(Servo.class,   "servo");
        Servos  = hardwareMap.getAll(Servo.class);
        ListIterator<Servo> ServoIterator = Servos.listIterator();
        motorFR.setDirection(DcMotor.Direction.FORWARD);
        motorFL.setDirection(DcMotor.Direction.REVERSE);
        motorBR.setDirection(DcMotor.Direction.FORWARD);
        motorBL.setDirection(DcMotor.Direction.REVERSE);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {

            double axial   = -gamepad1.left_stick_y;
            double lateral =  gamepad1.left_stick_x;
            double yaw     =  gamepad1.right_stick_x;
            double speed   =  gamepad1.right_trigger;

            double leftFrontPower  = axial + lateral - yaw;
            double rightFrontPower = axial - lateral + yaw;
            double leftBackPower   = axial - lateral - yaw;
            double rightBackPower  = axial + lateral + yaw;

            motorFR.setPower(leftFrontPower / speed);
            motorFL.setPower(rightFrontPower / speed);
            motorBR.setPower(leftBackPower / speed);
            motorBL.setPower(rightBackPower / speed);
            if (gamepad1.x) {
                Servo0.setPosition(gamepad1.right_trigger);
            }
            if (gamepad1.y) {
                Servos.get(1).setPosition(gamepad1.right_trigger);
            }
            if (gamepad1.a) {
                double position = 0.5 + ((gamepad1.right_trigger) - (gamepad1.left_trigger)) / 2;
                Servos.get(2).setPosition(position);
            }
            if (gamepad1.b) {
                double position = 0.5 + ((gamepad1.right_trigger) - (gamepad1.left_trigger)) / 2;
                Servos.get(3).setPosition(gamepad1.right_trigger);
            }
        }
    }
}