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

@TeleOp(name = "OTOS: Dance Routine", group = "OTOS")
public class DanceRoutineOTOS extends LinearOpMode {
    // Create an instance of the sensor
    SparkFunOTOS myOtos;
    public DcMotor Front_Right;
    public DcMotor Front_Left;
    public DcMotor Back_Right;
    public DcMotor Back_Left;
    public MovementLib.DriveWheels Wheels;
    public MovementLib.OTOSControl OC;
    @Override
    public void runOpMode() throws InterruptedException {
        // Get a reference to the sensor
        Front_Right = hardwareMap.get(DcMotor.class, "FrontRight");
        Front_Left = hardwareMap.get(DcMotor.class, "FrontLeft");
        Back_Right = hardwareMap.get(DcMotor.class, "BackRight");
        Back_Left = hardwareMap.get(DcMotor.class, "BackLeft");
        Wheels = new MovementLib.DriveWheels(Front_Right, Front_Left, Back_Right, Back_Left);
        Front_Left.setDirection(DcMotorSimple.Direction.REVERSE);
        Back_Left.setDirection(DcMotorSimple.Direction.REVERSE);
        myOtos = hardwareMap.get(SparkFunOTOS.class, "sensor_otos");
        OC = new MovementLib.OTOSControl(Wheels, myOtos);
        int corner = 0;
        int ticks = 0;
        waitForStart();
        while (opModeIsActive()) {
            if(gamepad1.start) {
                OC.calibrate();
            }
            OC.OTOS_Move(1,0,0,.5);
            SparkFunOTOS.Pose2D pos = OC.otos.getPosition();
            telemetry.addData("X:", pos.x);
            telemetry.addData("Y:", pos.y);
            telemetry.addData("H:", pos.h);
            telemetry.update();
        }
    }
}