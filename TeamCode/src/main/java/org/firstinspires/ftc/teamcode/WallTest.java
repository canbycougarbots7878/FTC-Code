package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import java.util.Arrays;
import java.util.List;

@TeleOp(name = "Wall Test", group = "Test")
public class WallTest extends LinearOpMode {

    private SparkFunOTOS myOtos;

    public void runOpMode() throws InterruptedException {
        // Initialize motors
        DcMotor frontRight = hardwareMap.get(DcMotor.class, "FrontRight");
        DcMotor frontLeft = hardwareMap.get(DcMotor.class, "FrontLeft");
        DcMotor backRight = hardwareMap.get(DcMotor.class, "BackRight");
        DcMotor backLeft = hardwareMap.get(DcMotor.class, "BackLeft");

        // Reverse left motors for correct mecanum behavior
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        // Initialize movement library and OTOS
        MovementLib.DriveWheels wheels = new MovementLib.DriveWheels(frontRight, frontLeft, backRight, backLeft);
        myOtos = hardwareMap.get(SparkFunOTOS.class, "sensor_otos");
        MovementLib.OTOSControl oc = new MovementLib.OTOSControl(wheels, myOtos);

        // Define 4 virtual walls (3 feet = 0.9144 meters from center)
        List<Wall> walls = Arrays.asList(
                new Wall(Wall.Axis.Y,  0.9144, -0.9144, 0.9144), // Top wall
                new Wall(Wall.Axis.Y, -0.9144, -0.9144, 0.9144), // Bottom wall
                new Wall(Wall.Axis.X, -0.9144, -0.9144, 0.9144), // Left wall
                new Wall(Wall.Axis.X,  0.9144, -0.9144, 0.9144)  // Right wall
        );

        telemetry.addLine("Initialized and ready.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            double f = gamepad1.left_stick_y;
            double s = -gamepad1.left_stick_x;
            double turn = -gamepad1.right_stick_x;

            SparkFunOTOS.Pose2D pose = myOtos.getPosition();
            double headingRad = Math.toRadians(pose.h);

            // Field-centric transformation
            double fField = f * Math.cos(headingRad) - s * Math.sin(headingRad);
            double sField = f * Math.sin(headingRad) + s * Math.cos(headingRad);

            double threshold = 0.1; // meters (~4 inches)

            for (Wall wall : walls) {
                if (wall.isNear(pose.x, pose.y, threshold)) {
                    telemetry.addLine("Warning: Near wall at " + wall.axis + "=" + wall.position);

                    // Direction-aware stop
                    if (wall.axis == Wall.Axis.Y) {
                        if ((pose.y > wall.position && fField > 0) || (pose.y < wall.position && fField < 0)) {
                            fField = 0;
                        }
                    } else if (wall.axis == Wall.Axis.X) {
                        if ((pose.x > wall.position && sField > 0) || (pose.x < wall.position && sField < 0)) {
                            sField = 0;
                        }
                    }
                }
            }

            oc.dw.Omni_Move(fField, sField, turn, 0.5);

            telemetry.addData("X", pose.x);
            telemetry.addData("Y", pose.y);
            telemetry.addData("Heading", pose.h);
            telemetry.update();
        }

        oc.dw.Stop_Wheels(); // Safety stop on end
    }

    // --- Wall Class Definition ---
    public static class Wall {
        public enum Axis { X, Y }

        public Axis axis;
        public double position;  // Where the wall is located on the axis
        public double minRange;  // The allowed range on the other axis
        public double maxRange;

        public Wall(Axis axis, double position, double minRange, double maxRange) {
            this.axis = axis;
            this.position = position;
            this.minRange = minRange;
            this.maxRange = maxRange;
        }

        // Returns true if robot is near the wall
        public boolean isNear(double x, double y, double threshold) {
            if (axis == Axis.X) {
                if (y < minRange || y > maxRange) return false;
                return Math.abs(x - position) < threshold;
            } else {
                if (x < minRange || x > maxRange) return false;
                return Math.abs(y - position) < threshold;
            }
        }
    }
}
