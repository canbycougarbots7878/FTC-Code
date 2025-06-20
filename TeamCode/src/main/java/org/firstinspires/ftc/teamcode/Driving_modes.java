package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

@TeleOp(name = "Driving Modes", group = "Concept")
public class Driving_modes extends LinearOpMode {
    DcMotor Front_Right = null;
    DcMotor Front_Left = null;
    DcMotor Back_Right = null;
    DcMotor Back_Left = null;
    MovementLib.DriveWheels Wheels = null;
    SparkFunOTOS OtosSensor = null;
    MovementLib.OTOSControl Otos = null;
    IMU imu = null;

    public void runOpMode() {
        Front_Right = hardwareMap.get(DcMotor.class, "FrontRight");
        Front_Left = hardwareMap.get(DcMotor.class, "FrontLeft");
        Back_Right = hardwareMap.get(DcMotor.class, "BackRight");
        Back_Left = hardwareMap.get(DcMotor.class, "BackLeft");
        Wheels = new MovementLib.DriveWheels(Front_Right, Front_Left, Back_Right, Back_Left);
        Front_Right.setDirection(DcMotorSimple.Direction.REVERSE);
        Back_Right.setDirection(DcMotorSimple.Direction.REVERSE);
        OtosSensor = hardwareMap.get(SparkFunOTOS.class, "sensor_otos");
        Otos = new MovementLib.OTOSControl(Wheels, OtosSensor, telemetry, hardwareMap);

        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.RIGHT, RevHubOrientationOnRobot.UsbFacingDirection.UP);
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        String driving_mode = "Basic";
        double field_size = 0.3;

        waitForStart();
        while(opModeIsActive()) {
            if(driving_mode.equals("Basic")) {
                double forward = gamepad1.left_stick_y;
                double strafe = - gamepad1.left_stick_x;
                Wheels.Omni_Move(forward, strafe, gamepad1.right_stick_x, 1);
            }
            else if(driving_mode.equals("PRM")) {
                double heading = imu.getRobotYawPitchRollAngles().getYaw();
                double forward = Math.cos(heading) * gamepad1.left_stick_y - Math.sin(heading) * -gamepad1.left_stick_x;
                double strafe = Math.sin(heading) * gamepad1.left_stick_y + Math.cos(heading) * -gamepad1.left_stick_x;
                Wheels.Omni_Move(forward, strafe, gamepad1.right_stick_x, 1);
            }
            if(gamepad1.a) {
                driving_mode = "Basic";
            }
            if(gamepad1.b) {
                driving_mode = "PRM";
            }
            if(gamepad1.x) {
                driving_mode = "Barrier";
            }

            if(gamepad1.start) {
                Otos.calibrate();
            }
            telemetry.addData("Mode", driving_mode);
            telemetry.update();
        }
    }
}
