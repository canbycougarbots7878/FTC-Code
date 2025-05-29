// Date: 2025-05-28

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

            // Get references to the motors
            DcMotor frontRight = hardwareMap.get(DcMotor.class, "FrontRight");
            DcMotor frontLeft = hardwareMap.get(DcMotor.class, "FrontLeft");
            DcMotor backRight = hardwareMap.get(DcMotor.class, "BackRight");
            DcMotor backLeft = hardwareMap.get(DcMotor.class, "BackLeft");

            // Set directions as needed (check if reversed on left/right sides)
            frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
            backRight.setDirection(DcMotorSimple.Direction.REVERSE);

            // Get OTOS sensor
            SparkFunOTOS myOtos = hardwareMap.get(SparkFunOTOS.class, "sensor_otos");

            // Create helper classes
            MovementLib.DriveWheels wheels = new MovementLib.DriveWheels(frontRight, frontLeft, backRight, backLeft);
            this.OC = new MovementLib.OTOSControl(wheels, myOtos);
        }

        public void goToPosition(int position) {
            if (position == 1) {
                double destinationX = 1;
                double destinationY = 0;
                double destinationH = 0;
                double speed = 0.2;

                long startTime = System.currentTimeMillis();
                long timeout = 5000; // 5 seconds timeout max

                // Loop while OTOS_Move returns true (still moving), op mode active, and within timeout
                while (OC.OTOS_Move(destinationX, destinationY, destinationH, speed)
                        && opMode.opModeIsActive()
                        && System.currentTimeMillis() - startTime < timeout) {

                    SparkFunOTOS.Pose2D pos = OC.otos.getPosition();
                    telemetry.addData("X:", pos.x);
                    telemetry.addData("Y:", pos.y);
                    telemetry.addData("H:", pos.h);
                    telemetry.update();
                }

                // Stop wheels explicitly after movement completes or timeout reached
                OC.dw.Stop_Wheels();

                telemetry.addLine("Target reached or timeout.");
                telemetry.update();

                try {
                    Thread.sleep(250); // Brief pause to let robot stabilize
                } catch (InterruptedException e) {
                    Thread.currentThread().interrupt();
                }
            }
        }
    }
}
