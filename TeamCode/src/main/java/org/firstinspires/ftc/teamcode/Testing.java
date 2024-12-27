package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.MovementLib.*;
@TeleOp(name = "Arm Testing", group = "Concept")
public class Testing extends LinearOpMode {
    public IMU imu;
    public DcMotor Front_Right;
    public DcMotor Front_Left;
    public DcMotor Back_Right;
    public DcMotor Back_Left;
    public DriveWheels Drive_Wheels;
    public void runOpMode() {
        Front_Right = hardwareMap.get(DcMotor.class, "Front Right");
        Front_Left = hardwareMap.get(DcMotor.class, "Front Left");
        Back_Right = hardwareMap.get(DcMotor.class, "Back Right");
        Back_Left = hardwareMap.get(DcMotor.class, "Back Left");
        Drive_Wheels = new DriveWheels(Front_Right, Front_Left, Back_Right, Back_Left);
        Drive_Wheels.Reverse_these_wheels(true, true, false, true);
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(
          new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD))
        );
        waitForStart();
        while (opModeIsActive()) {
            YawPitchRollAngles angles = imu.getRobotYawPitchRollAngles();
            double yaw = angles.getYaw() + 123;
            double forward = Math.cos(Math.toRadians(yaw)) * gamepad1.left_stick_y;
            double right = - Math.sin(Math.toRadians(yaw)) * gamepad1.left_stick_y;
            telemetry.addData("Angles", angles);
            telemetry.update();
            Drive_Wheels.Omni_Move(forward, right, gamepad1.right_stick_x, 1);
        }
    }
}
