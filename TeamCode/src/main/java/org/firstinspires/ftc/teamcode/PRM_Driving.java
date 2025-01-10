package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.MovementLib.*;
@TeleOp(name = "Driving Modes", group = "Concept")
public class PRM_Driving extends LinearOpMode {
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
        double PRM_OFFSET = imu.getRobotYawPitchRollAngles().getYaw();
        int mode = 0;
        while (opModeIsActive()) {
            if(gamepad1.a) {
                mode = 1;
                PRM_OFFSET = imu.getRobotYawPitchRollAngles().getYaw();
            }
            else if(gamepad1.b) {
                mode = 0;
            }
            if(mode == 0) { // LM mode
                telemetry.addLine("Local Movement Mode");
                Drive_Wheels.Omni_Move(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x, 1);
            }
            else { // GM mode
                telemetry.addLine("Global Movement Mode");
                YawPitchRollAngles angles = imu.getRobotYawPitchRollAngles();
                double yaw = angles.getYaw() - PRM_OFFSET;
                double forward = Math.cos(Math.toRadians(yaw)) * gamepad1.left_stick_y + Math.sin(Math.toRadians(yaw)) * gamepad1.left_stick_x;
                double right = -Math.sin(Math.toRadians(yaw)) * gamepad1.left_stick_y + Math.cos(Math.toRadians(yaw)) * gamepad1.left_stick_x;
                telemetry.addData("Angles", angles);
                Drive_Wheels.Omni_Move(forward, right, gamepad1.right_stick_x, 1);
                if (gamepad1.start) {
                    PRM_OFFSET = imu.getRobotYawPitchRollAngles().getYaw();
                }
            }
            telemetry.update();
        }
    }
}
