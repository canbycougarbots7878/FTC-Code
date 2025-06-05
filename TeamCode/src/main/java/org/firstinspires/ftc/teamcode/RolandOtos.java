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
@TeleOp(name = "OTOS Drive in circles", group = "OTOS")
public class RolandOtos extends LinearOpMode {
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
        OC = new MovementLib.OTOSControl(Wheels, myOtos, telemetry);
        int corner = 0;
        int ticks = 0;
        waitForStart();
        while (opModeIsActive()) {
            if(gamepad1.start) {
                OC.calibrate();
            }

            double seconds = getRuntime() / 3;
            OC.OTOS_Move(Math.sin(seconds)/2,Math.cos(seconds)/2,Math.toDegrees(seconds),.5);

            SparkFunOTOS.Pose2D pos = OC.otos.getPosition();
            telemetry.addData("X:", pos.x);
            telemetry.addData("Y:", pos.y);
            telemetry.addData("H:", pos.h);
            telemetry.addData("Distance:", Math.sqrt(pos.x*pos.x+pos.y*pos.y));
            telemetry.update();
        }
    }
    private double distance(double x1, double y1, double x2, double y2) {
        double x = x2 - x1;
        double y = y2 - y1;
        return Math.sqrt(x*x+y*y);
    }
    private boolean Corner(int number) {
        switch(number) {
            case 0: return OC.OTOS_Move(0,0,0,.5);
            case 1: return OC.OTOS_Move(1,1,0,.5);
            case 2: return OC.OTOS_Move(1,-1,0,.5);
            case 3: return OC.OTOS_Move(-1,-1,0,.5);
            case 4: return OC.OTOS_Move(-1,1,0,.5);
        }
        return false;
    }
}