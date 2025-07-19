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
        public final DriveWheels dw;
        public final SparkFunOTOS otos;
        public double heading = 0;

        public OTOSControl(DriveWheels driveWheels, SparkFunOTOS OTOS_attach) {
            this.dw = driveWheels;
            this.otos = OTOS_attach;
            initialize();  // Only once, in constructor
        }

        private void initialize() {
            this.otos.setLinearUnit(DistanceUnit.METER);
            this.otos.setAngularUnit(AngleUnit.DEGREES);
            this.otos.setOffset(new SparkFunOTOS.Pose2D(0, 0, 0));
            this.otos.setLinearScalar(1.0);   // Adjust if your robot isn't tracking distance correctly
            this.otos.setAngularScalar(1.0);  // Adjust if heading is off
            this.otos.calibrateImu();
            this.otos.resetTracking();
            this.otos.setPosition(new SparkFunOTOS.Pose2D(0, 0, 0));
        }

        public boolean OTOS_Move(double destination_x, double destination_y, double destination_h, double maxPower) {
            SparkFunOTOS.Pose2D pos = this.otos.getPosition();
            double x = pos.x;
            double y = pos.y;
            this.heading = pos.h;

            double dx = destination_x - x;
            double dy = destination_y - y;

            double distance = Math.hypot(dx, dy);
            double headingError = normalizeAngle(destination_h - heading);

            // Stop if close enough
            if (distance < 0.02 && Math.abs(headingError) < 2.0) {
                this.dw.Stop_Wheels();
                return true;
            }

            double turn = headingError / 30.0; // Tune if needed

            double headingRad = Math.toRadians(heading);
            double cosH = Math.cos(headingRad);
            double sinH = Math.sin(headingRad);

            double f = dx * cosH - dy * sinH;
            double s = dx * sinH + dy * cosH;

            this.dw.Omni_Move(f, s, turn, maxPower);
            return false;
        }

        private double normalizeAngle(double angle) {
            while (angle > 180) angle -= 360;
            while (angle < -180) angle += 360;
            return angle;
        }
    }
}
