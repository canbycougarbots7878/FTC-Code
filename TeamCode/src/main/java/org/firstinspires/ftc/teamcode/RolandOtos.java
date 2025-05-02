/*
    SPDX-License-Identifier: MIT

    Copyright (c) 2024 SparkFun Electronics
*/
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "Roland: OTOS Testing", group = "Sensor")
public class RolandOtos extends LinearOpMode {
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
        Back_Right.setDirection(DcMotorSimple.Direction.REVERSE);
        myOtos = hardwareMap.get(SparkFunOTOS.class, "sensor_otos");
        MovementLib.OTOSControl OC = new MovementLib.OTOSControl(Wheels, myOtos);

        waitForStart();
        while (opModeIsActive()) {
            if(gamepad1.start) {
                OC.calibrate();
            }
            OC.OTOS_Move(1);
        }
    }
}