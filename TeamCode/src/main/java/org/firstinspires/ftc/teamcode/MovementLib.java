package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

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
            //pneumonoultramicroscopicsilicovolcanoconiosis
            this.Back_Left = Back_Left;
        }
        public void Set_Wheels(double Front_Right_Power, double Front_Left_Power, double Back_Right_Power, double Back_Left_Power) {
            this.Front_Right.setPower(Front_Right_Power);
            this.Front_Left.setPower(Front_Left_Power);
            this.Back_Right.setPower(Back_Right_Power);
            this.Back_Left.setPower(Back_Left_Power);
        }
        public void Omni_Move(double Forward, double Right, double Rotate, double speed) {
            double Front_Left_Power = Math.min(Math.max(- Forward - Right + Rotate, -1), 1);
            double Front_Right_Power = Math.min(Math.max(- Forward + Right - Rotate, -1), 1);
            double Back_Right_Power = Math.min(Math.max(- Forward - Right - Rotate, -1), 1);
            double Back_Left_Power = Math.min(Math.max(- Forward + Right + Rotate, -1), 1);
            this.Set_Wheels(speed * Front_Right_Power, speed * Front_Left_Power, speed * Back_Right_Power, speed * Back_Left_Power);
        }

        public void Stop_Wheels() {
            this.Set_Wheels(0,0,0,0);
        }
    }
    public static class OTOSControl {
        public DriveWheels dw;
        public SparkFunOTOS otos;

        public OTOSControl(DriveWheels DriveWheels_attach, SparkFunOTOS OTOS_attach) {
            this.dw = DriveWheels_attach;
            this.otos = OTOS_attach;
            calibrate();
        }
        public boolean OTOS_Move(double destination_y, double destination_x, double destination_h, double power) {
            SparkFunOTOS.Pose2D pos = this.otos.getPosition();
            double x = pos.x;
            double y = pos.y;
            double h = pos.h;
            if(Math.abs(destination_y - y) > 0.0015 || Math.abs(destination_x - x) > 0.0015 || Math.abs(destination_h - h) > 0.0015) {
                double forward = destination_y - y;
                double strafe = destination_x - x;
                double turn = (destination_h - h) * 0.25;
                this.dw.Omni_Move(10*forward,10*strafe,turn, power); // times ten so it doesn't slow down so fast, the value gets clamped anyway
            }
            else {
                this.dw.Stop_Wheels();
            }
            double xx = destination_x - x;
            double yy = destination_y - y;
            if(Math.sqrt(xx*xx+yy*yy) < 0.003) {
                return false;
            }
            else {
                return true;
            }
        }
        public void calibrate() {
            // Set the desired units for linear and angular measurements. Can be either
            // meters or inches for linear, and radians or degrees for angular. If not
            // set, the default is inches and degrees. Note that this setting is not
            // persisted in the sensor, so you need to set at the start of all your
            // OpModes if using the non-default value.
            this.otos.setLinearUnit(DistanceUnit.METER);
            this.otos.setAngularUnit(AngleUnit.DEGREES);

            // Assuming you've mounted your sensor to a robot and it's not centered,
            // you can specify the offset for the sensor relative to the center of the
            // robot. The units default to inches and degrees, but if you want to use
            // different units, specify them before setting the offset! Note that as of
            // firmware version 1.0, these values will be lost after a power cycle, so
            // you will need to set them each time you power up the sensor. For example, if
            // the sensor is mounted 5 inches to the left (negative X) and 10 inches
            // forward (positive Y) of the center of the robot, and mounted 90 degrees
            // clockwise (negative rotation) from the robot's orientation, the offset
            // would be {-5, 10, -90}. These can be any value, even the angle can be
            // tweaked slightly to compensate for imperfect mounting (eg. 1.3 degrees).
            SparkFunOTOS.Pose2D offset = new SparkFunOTOS.Pose2D(0, 0, 0);
            this.otos.setOffset(offset);

            // Here we can set the linear and angular scalars, which can compensate for
            // scaling issues with the sensor measurements. Note that as of firmware
            // version 1.0, these values will be lost after a power cycle, so you will
            // need to set them each time you power up the sensor. They can be any value
            // from 0.872 to 1.127 in increments of 0.001 (0.1%). It is recommended to
            // first set both scalars to 1.0, then calibrate the angular scalar, then
            // the linear scalar. To calibrate the angular scalar, spin the robot by
            // multiple rotations (eg. 10) to get a precise error, then set the scalar
            // to the inverse of the error. Remember that the angle wraps from -180 to
            // 180 degrees, so for example, if after 10 rotations counterclockwise
            // (positive rotation), the sensor reports -15 degrees, the required scalar
            // would be 3600/3585 = 1.004. To calibrate the linear scalar, move the
            // robot a known distance and measure the error; do this multiple times at
            // multiple speeds to get an average, then set the linear scalar to the
            // inverse of the error. For example, if you move the robot 100 inches and
            // the sensor reports 103 inches, set the linear scalar to 100/103 = 0.971
            this.otos.setLinearScalar(1.0);
            this.otos.setAngularScalar(1.0);

            // The IMU on the OTOS includes a gyroscope and accelerometer, which could
            // have an offset. Note that as of firmware version 1.0, the calibration
            // will be lost after a power cycle; the OTOS performs a quick calibration
            // when it powers up, but it is recommended to perform a more thorough
            // calibration at the start of all your OpModes. Note that the sensor must
            // be completely stationary and flat during calibration! When calling
            // calibrateImu(), you can specify the number of samples to take and whether
            // to wait until the calibration is complete. If no parameters are provided,
            // it will take 255 samples and wait until done; each sample takes about
            // 2.4ms, so about 612ms total
            this.otos.calibrateImu();

            // Reset the tracking algorithm - this resets the position to the origin,
            // but can also be used to recover from some rare tracking errors
            this.otos.resetTracking();

            // After resetting the tracking, the OTOS will report that the robot is at
            // the origin. If your robot does not start at the origin, or you have
            // another source of location information (eg. vision odometry), you can set
            // the OTOS location to match and it will continue to track from there.
            SparkFunOTOS.Pose2D currentPosition = new SparkFunOTOS.Pose2D(0, 0, 0);
            this.otos.setPosition(currentPosition);

            // Get the hardware and firmware version
            SparkFunOTOS.Version hwVersion = new SparkFunOTOS.Version();
            SparkFunOTOS.Version fwVersion = new SparkFunOTOS.Version();
            this.otos.getVersionInfo(hwVersion, fwVersion);
        }
    }
}