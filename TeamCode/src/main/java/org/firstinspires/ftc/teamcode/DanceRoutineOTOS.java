package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "OTOS: Dance Routine", group = "OTOS")
public class DanceRoutineOTOS extends LinearOpMode {
    SparkFunOTOS myOtos;
    DcMotor Front_Right, Front_Left, Back_Right, Back_Left;
    MovementLib.DriveWheels Wheels;
    MovementLib.OTOSControl OC;

    @Override
    public void runOpMode() throws InterruptedException {
        Front_Right = hardwareMap.get(DcMotor.class, "FrontRight");
        Front_Left  = hardwareMap.get(DcMotor.class, "FrontLeft");
        Back_Right  = hardwareMap.get(DcMotor.class, "BackRight");
        Back_Left   = hardwareMap.get(DcMotor.class, "BackLeft");

        Front_Right.setDirection(DcMotorSimple.Direction.REVERSE);
        Back_Right.setDirection(DcMotorSimple.Direction.REVERSE);

        Wheels = new MovementLib.DriveWheels(Front_Right, Front_Left, Back_Right, Back_Left);
        myOtos = hardwareMap.get(SparkFunOTOS.class, "sensor_otos");
        OC = new MovementLib.OTOSControl(Wheels, myOtos, telemetry, hardwareMap);

        waitForStart();

        boolean moveComplete = false;

        while (opModeIsActive()) {
            if (gamepad1.start) {
                OC.calibrate();
            }

            if (!moveComplete) {
                // Keep moving until it's done
                moveComplete = !OC.OTOS_Move(1.0, 0.0, 0, 1, true);
            } else {
                telemetry.addLine("Move complete. Press A to restart.");
                if (gamepad1.a) {
                    moveComplete = false; // Restart movement when A is pressed
                }
            }

            telemetry.update();
        }
    }
}
