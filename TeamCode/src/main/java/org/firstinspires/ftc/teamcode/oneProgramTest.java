// Date: 2025-06-04

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class oneProgramTest {

    public static class Position {

        private final MovementLib.OTOSControl OC;
        private final Telemetry telemetry;
        private final LinearOpMode opMode;

        public Position(HardwareMap hardwareMap, Telemetry telemetry, LinearOpMode opMode) {
            this.telemetry = telemetry;
            this.opMode = opMode;

            // Initialize motors
            DcMotor frontRight = hardwareMap.get(DcMotor.class, "FrontRight");
            DcMotor frontLeft = hardwareMap.get(DcMotor.class, "FrontLeft");
            DcMotor backRight = hardwareMap.get(DcMotor.class, "BackRight");
            DcMotor backLeft = hardwareMap.get(DcMotor.class, "BackLeft");

            // Motor directions
            frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
            frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
            backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
            backRight.setDirection(DcMotorSimple.Direction.FORWARD);

            // Optional: Reset motor encoders if needed
            frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            // Get OTOS sensor
            SparkFunOTOS myOtos = hardwareMap.get(SparkFunOTOS.class, "sensor_otos");

            // Create DriveWheels instance
            MovementLib.DriveWheels wheels = new MovementLib.DriveWheels(frontRight, frontLeft, backRight, backLeft);

            // Create OTOSControl instance
            this.OC = new MovementLib.OTOSControl(wheels, myOtos, telemetry);
        }

        public void goToPosition(int position) {
            if (position == 1) {
                double destinationX = 1.0;
                double destinationY = 0.0;
                double destinationH = 0.0;
                double speed = 0.3;


                while (OC.OTOS_Move(destinationX, destinationY, destinationH, speed) && opMode.opModeIsActive()) {
                    // OTOS_Move handles all motion & telemetry updates
                }

                OC.dw.Stop_Wheels();
                telemetry.addLine("Target reached or timed out.");
                telemetry.update();

                opMode.sleep(250); // Safe sleep inside opMode
            }
        }
    }
}
