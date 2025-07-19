package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "movementGamepad", group = "movement")
public class movementGamepad extends LinearOpMode {

    private SparkFunOTOS myOtos;

    public void runOpMode() throws InterruptedException {
        // Initialize motors
        DcMotor frontRight = hardwareMap.get(DcMotor.class, "FrontRight");
        DcMotor frontLeft = hardwareMap.get(DcMotor.class, "FrontLeft");
        DcMotor backRight = hardwareMap.get(DcMotor.class, "BackRight");
        DcMotor backLeft = hardwareMap.get(DcMotor.class, "BackLeft");

        // Reverse left motors
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        // Wrap motors and OTOS
        MovementLib.DriveWheels wheels = new MovementLib.DriveWheels(frontRight, frontLeft, backRight, backLeft);
        myOtos = hardwareMap.get(SparkFunOTOS.class, "sensor_otos");
        MovementLib.OTOSControl oc = new MovementLib.OTOSControl(wheels, myOtos);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            double f = gamepad1.left_stick_y;
            double s = -gamepad1.left_stick_x;
            double turn = -gamepad1.right_stick_x;

            // Optional field-centric drive
            SparkFunOTOS.Pose2D pose = myOtos.getPosition();
            double headingRad = Math.toRadians(pose.h);
            double fField = f * Math.cos(headingRad) - s * Math.sin(headingRad);
            double sField = f * Math.sin(headingRad) + s * Math.cos(headingRad);

            oc.dw.Omni_Move(fField, sField, turn, 0.5);

            telemetry.addData("X", pose.x);
            telemetry.addData("Y", pose.y);
            telemetry.addData("Heading", pose.h);
            telemetry.update();
        }

        oc.dw.Stop_Wheels(); // Safety stop
    }
}
