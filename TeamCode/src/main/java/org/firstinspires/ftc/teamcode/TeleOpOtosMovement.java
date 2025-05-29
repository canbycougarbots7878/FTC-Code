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

@Disabled
@TeleOp(name = "TeleOp OTOS Movement", group = "OTOS")
public class TeleOpOtosMovement extends LinearOpMode {
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


        waitForStart();
        while (opModeIsActive()) {
            if(gamepad1.start) {
                OC.calibrate();
            }
            if(gamepad1.back) {
                OC.OTOS_Move(0,0,0,1);
            }
            SparkFunOTOS.Pose2D pos = myOtos.getPosition();
            Wheels.Omni_Move(-gamepad1.left_stick_y + -gamepad2.left_stick_y, gamepad1.left_stick_x + gamepad2.left_stick_x, -gamepad1.right_stick_x + -gamepad2.right_stick_x, 0.5);
            telemetry.addData("X:", pos.x);
            telemetry.addData("Y:", pos.y);
            telemetry.addData("H:", pos.h);
            telemetry.update();
        }
    }
}