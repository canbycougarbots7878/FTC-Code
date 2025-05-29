/*
    SPDX-License-Identifier: MIT

    Copyright (c) 2024 SparkFun Electronics
*/
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "spiral of Archimedes", group = "OTOS")
public class spiral_of_Archimedes extends LinearOpMode {
    // Create an instance of the sensor
    SparkFunOTOS myOtos;

    @Override
    public void runOpMode() throws InterruptedException {
        // Get a reference to the sensor
        DcMotor Front_Right = hardwareMap.get(DcMotor.class, "FrontRight");
        DcMotor Front_Left = hardwareMap.get(DcMotor.class, "FrontLeft");
        DcMotor Back_Right = hardwareMap.get(DcMotor.class, "BackRight");
        DcMotor Back_Left = hardwareMap.get(DcMotor.class, "BackLeft");
        MovementLib.DriveWheels Wheels = new MovementLib.DriveWheels(Front_Right, Front_Left, Back_Right, Back_Left);
        Front_Left.setDirection(DcMotorSimple.Direction.REVERSE);
        Back_Left.setDirection(DcMotorSimple.Direction.REVERSE);
        myOtos = hardwareMap.get(SparkFunOTOS.class, "sensor_otos");
        MovementLib.OTOSControl OC = new MovementLib.OTOSControl(Wheels, myOtos);
        double theta = 0;
        double delta_theta = 1;
        int number_of_rotations = 4;

        waitForStart();
        while (opModeIsActive()) {
            if(gamepad1.start) {
                OC.calibrate();
            }
            double x = theta * Math.cos(Math.toRadians(theta));
            double y = theta * Math.sin(Math.toRadians(theta));
            OC.OTOS_Move( x/100.0f, y/100.0f, theta, 0.2);
            SparkFunOTOS.Pose2D pos = OC.otos.getPosition();
            telemetry.addData("X:", pos.x);
            telemetry.addData("Y:", pos.y);
            telemetry.addData("H:", pos.h);
            telemetry.update();
            if (theta < (360 * number_of_rotations)){
                theta += delta_theta;
            }
        }
    }
}