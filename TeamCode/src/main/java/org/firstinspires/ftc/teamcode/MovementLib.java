package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;

public class MovementLib {

    public static class DriveWheels {
        public DcMotor Front_Right;
        public DcMotor Front_Left;
        public DcMotor Back_Right;
        public DcMotor Back_Left;

        public DriveWheels(DcMotor Front_Right, DcMotor Front_Left, DcMotor Back_Right, DcMotor Back_Left) {
            this.Front_Right = Front_Right;
            this.Front_Left = Front_Left;
            this.Back_Right = Back_Right;
            this.Back_Left = Back_Left;
        }

        public void Set_Wheels(double Front_Right_Power, double Front_Left_Power, double Back_Right_Power, double Back_Left_Power) {
            this.Front_Right.setPower(Front_Right_Power);
            this.Front_Left.setPower(Front_Left_Power);
            this.Back_Right.setPower(Back_Right_Power);
            this.Back_Left.setPower(Back_Left_Power);
        }

        public void Omni_Move(double Forward, double Right, double Rotate, double speed) {
            double fl = Forward + Right + Rotate;
            double fr = Forward - Right - Rotate;
            double br = Forward + Right - Rotate;
            double bl = Forward - Right + Rotate;

            double max = Math.max(1.0, Math.max(Math.abs(fl),
                    Math.max(Math.abs(fr), Math.max(Math.abs(br), Math.abs(bl)))));

            fl /= max;
            fr /= max;
            br /= max;
            bl /= max;

            this.Set_Wheels(fr * speed, fl * speed, br * speed, bl * speed);
        }

        public void Stop_Wheels() {
            this.Set_Wheels(0, 0, 0, 0);
        }
    }

    public static class OTOSControl {
        public DriveWheels dw;
        public SparkFunOTOS otos;
        public double heading = 0;
        private Telemetry telemetry;
        private VoltageSensor batterySensor;

        private Double startX = null;
        private Double startY = null;

        private final double DONE_DISTANCE_THRESHOLD = 0.07; // 7 cm tolerance
        private final double SLOWDOWN_START_DISTANCE = 0.2;
        private final double MIN_POWER = 0.5;

        private Long startTime = null;
        private final double ACCEL_TIME = 0.75; // seconds to reach full speed

        public OTOSControl(DriveWheels driveWheels, SparkFunOTOS OTOS_attach, Telemetry telemetry, HardwareMap hardwareMap) {
            this.dw = driveWheels;
            this.otos = OTOS_attach;
            this.telemetry = telemetry;
            calibrate();
            this.batterySensor = hardwareMap.voltageSensor.iterator().next();
        }

        public boolean OTOS_Move(double x, double y, double h, double power) {
            return OTOS_Move(x, y, h, power, false);
        }

        public boolean OTOS_Move(double destination_x, double destination_y, double destination_h, double maxPower, boolean stopIfDone) {
            SparkFunOTOS.Pose2D pos = this.otos.getPosition();
            double x = pos.x;
            double y = pos.y;
            this.heading = pos.h;

            if (startX == null || startY == null) {
                startX = x;
                startY = y;
                startTime = System.nanoTime(); // reset time on new move
            }

            double dx = destination_x - x;
            double dy = destination_y - y;
            double distance = Math.hypot(dx, dy);

            // Rotate field-relative vector (dx, dy) into robot-centric movement commands (f, s)
            double headingRad = Math.toRadians(heading);  // Use +heading
            double cosH = Math.cos(headingRad);
            double sinH = Math.sin(headingRad);

            double f = dx * cosH - dy * sinH;
            double s = dx * sinH + dy * cosH;

            double turnError = normalizeAngle(destination_h - heading);
            double gain = Math.abs(turnError) > 45 ? 0.08 : 0.05;

            // Flip sign to fix turn direction (90° = CCW)
            double turn = -turnError * gain;
            turn = Math.max(-0.4, Math.min(0.4, turn));

            // Suppress forward and strafe while turning sharply
            if (Math.abs(turnError) > 15) {
                f = 0;
                s = 0;
            }

            double movePower;
            double decelT;
            if (distance > SLOWDOWN_START_DISTANCE) {
                decelT = 1.0;
            } else if (distance < DONE_DISTANCE_THRESHOLD) {
                decelT = 0.0;
            } else {
                decelT = (distance - DONE_DISTANCE_THRESHOLD) / (SLOWDOWN_START_DISTANCE - DONE_DISTANCE_THRESHOLD);
                decelT = Math.max(0, Math.min(1, decelT));
                decelT = Math.pow(decelT, 2); // Quadratic easing
            }

            // Calculate acceleration progress
            double accelT = 1.0;
            if (startTime != null) {
                double elapsedSec = (System.nanoTime() - startTime) / 1e9;
                accelT = Math.max(0.0, Math.min(1.0, elapsedSec / ACCEL_TIME));
            }

            movePower = MIN_POWER + (maxPower - MIN_POWER) * Math.min(accelT, decelT);

            double voltageFactor = 1.0; // Disabled voltage compensation for testing
            movePower *= voltageFactor;

            this.dw.Omni_Move(f, s, turn, movePower);

            // Progress-based completion logic
            double dirX = destination_x - startX;
            double dirY = destination_y - startY;
            double pathLengthSquared = dirX * dirX + dirY * dirY;
            double progress = (pathLengthSquared == 0) ? 0 : ((dirX * (x - startX) + dirY * (y - startY)) / pathLengthSquared);
            progress = Math.max(0, Math.min(1, progress));

            boolean alignedHeading = Math.abs(turnError) < 6;
            boolean done = (distance < DONE_DISTANCE_THRESHOLD && alignedHeading);

            if (done) {
                startX = null;
                startY = null;
                startTime = null;
                if (stopIfDone) this.dw.Stop_Wheels();
            }

            if (telemetry != null) {
                telemetry.addData("X", x);
                telemetry.addData("Y", y);
                telemetry.addData("Heading (h)", heading);
                telemetry.addData("Turn Error", turnError);
                telemetry.addData("Turn Power", turn);
                telemetry.addData("Forward", f);
                telemetry.addData("Strafe", s);
                telemetry.addData("Move Power", movePower);
                telemetry.addData("Distance to Target (m)", distance);
                telemetry.addData("Battery Voltage", batterySensor.getVoltage());
                telemetry.addData("Progress", progress);
                telemetry.addData("Aligned Heading", alignedHeading);
                telemetry.addData("Done?", done);
                telemetry.addData("FR Power", dw.Front_Right.getPower());
                telemetry.addData("FL Power", dw.Front_Left.getPower());
                telemetry.addData("BR Power", dw.Back_Right.getPower());
                telemetry.addData("BL Power", dw.Back_Left.getPower());
                telemetry.update();
            }

            return !done;
        }

        public void calibrate() {
            this.otos.setLinearUnit(DistanceUnit.METER);
            this.otos.setAngularUnit(AngleUnit.DEGREES);
            this.otos.setOffset(new SparkFunOTOS.Pose2D(0, 0, 0));
            this.otos.setLinearScalar(1.0);
            this.otos.setAngularScalar(1.0);
            this.otos.calibrateImu();
            this.otos.resetTracking();
            this.otos.setPosition(new SparkFunOTOS.Pose2D(0, 0, 0));
        }

        private double normalizeAngle(double angle) {
            while (angle > 180) angle -= 360;
            while (angle < -180) angle += 360;
            return angle;
        }
    }
}
