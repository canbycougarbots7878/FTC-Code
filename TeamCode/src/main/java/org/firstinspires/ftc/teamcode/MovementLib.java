package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

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
            this.Set_Wheels(0,0,0,0);
        }
    }
    public static class OTOSControl {
        public DriveWheels dw;
        public SparkFunOTOS otos;
        public double heading = 0;

        private double last_heading = 0;
        private double dh = 0;
        private Telemetry telemetry;

        public OTOSControl(DriveWheels DriveWheels_attach, SparkFunOTOS OTOS_attach, Telemetry telemetry) {
            this.dw = DriveWheels_attach;
            this.otos = OTOS_attach;
            this.telemetry = telemetry;
            calibrate();
        }

        public boolean OTOS_Move(double destination_x, double destination_y, double destination_h, double maxPower) {
            SparkFunOTOS.Pose2D pos = this.otos.getPosition();
            double x = pos.x;
            double y = pos.y;
            this.heading = pos.h;

            double dx = destination_x - x;
            double dy = destination_y - y;

            double distance = Math.hypot(dx, dy);

            // Dynamic power scaling — never drop below minPower
            double minPower = 0.12;
            double movePower = Math.max(minPower, Math.min(maxPower, distance * 1.5));

            // No normalization — keep full directional intent
            double f = -dx;
            double s = dy;

            // Heading correction
            double turnError = normalizeAngle(destination_h - pos.h);
            double gain = Math.abs(turnError) > 45 ? 0.08 : 0.05;
            double turn = turnError * gain;
            turn = Math.max(-0.4, Math.min(0.4, turn));

            // Drive
            this.dw.Omni_Move(f, s, turn, movePower);

            if (telemetry != null) {
                telemetry.addData("X", x);
                telemetry.addData("Y", y);
                telemetry.addData("Heading (h)", heading);
                telemetry.addData("Turn Error", turnError);
                telemetry.addData("Turn Power", turn);
                telemetry.addData("Forward", f);
                telemetry.addData("Strafe", s);
                telemetry.addData("Move Power", movePower);
                telemetry.update();
            }

            boolean reachedPosition = distance < 0.03;
            boolean alignedHeading = Math.abs(turnError) < 3;
            return !(reachedPosition && alignedHeading);
        }

        private double normalizeAngle(double angle) {
            while (angle > 180) angle -= 360;
            while (angle < -180) angle += 360;
            return angle;
        }

        public void OTOS_Forward(double destination_x) {
            SparkFunOTOS.Pose2D pos = this.otos.getPosition();
            double x = pos.x;
            double y_offset = pos.y;
            double turn_offset = -pos.h;
            if (Math.abs(destination_x - x) > 0.05) {
                this.dw.Omni_Move(destination_x - x, y_offset, turn_offset / 90.0f, 1);
            } else {
                this.dw.Stop_Wheels();
            }
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
    }
}