package org.firstinspires.ftc.teamcode;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

public class MovementLib {
    static class DriveWheels {
        public DcMotor Front_Right;
        public DcMotor Front_Left;
        public DcMotor Back_Right;
        public DcMotor Back_Left;

        public DriveWheels() {
        }

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
            double Front_Left_Power = - Forward + Right + Rotate;
            double Front_Right_Power = - Forward - Right - Rotate;
            double Back_Right_Power = - Forward + Right - Rotate;
            double Back_Left_Power = - Forward - Right + Rotate;
            this.Set_Wheels(speed * Front_Right_Power, speed * Front_Left_Power, speed * Back_Right_Power, speed * Back_Left_Power);
        }
        public void Reverse_these_wheels(boolean fr, boolean fl, boolean br, boolean bl) {
            if(fr) { this.Front_Right.setDirection(DcMotorSimple.Direction.REVERSE); } else { this.Front_Right.setDirection(DcMotorSimple.Direction.FORWARD); };
            if(fl) { this.Front_Left.setDirection(DcMotorSimple.Direction.REVERSE); } else { this.Front_Left.setDirection(DcMotorSimple.Direction.FORWARD); };
            if(br) { this.Back_Right.setDirection(DcMotorSimple.Direction.REVERSE); } else { this.Back_Right.setDirection(DcMotorSimple.Direction.FORWARD); };
            if(bl) { this.Back_Left.setDirection(DcMotorSimple.Direction.REVERSE); } else { this.Back_Left.setDirection(DcMotorSimple.Direction.FORWARD); };
        }
        public void Turn_Robot(double speed) {
            this.Set_Wheels(-speed, speed, -speed, speed);
        }
        public void Stop_Wheels() {
            this.Set_Wheels(0,0,0,0);
        }
        public void Turn_to(YawPitchRollAngles angles, double target_yaw) {
            while (angles.getYaw() > target_yaw + 10 || angles.getYaw() < target_yaw - 10) {
                this.Turn_Robot(-.3);
            }
        }
    }
    static class Slider {
        public DcMotor Slider_Motor;
        public int Max_Height;
        public int pos1;
        public int pos2;
        public int pos3;
        public Slider() {}

        public Slider(DcMotor attachedMotor) {
            this.Slider_Motor = attachedMotor;
        }
        public Slider(DcMotor attachedMotor, int Max_Height) {
            this.Slider_Motor = attachedMotor;
            this.Max_Height = Max_Height;
        }

        public void init() {
            this.Slider_Motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            this.Slider_Motor.setTargetPosition(0);
            this.Slider_Motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            this.Slider_Motor.setPower(0);
        }

        public void goPercentage(int percentage) {
            float decimal = percentage / 100;
        }

        public void goToPosition(int pos) {
            if (pos < 0 || pos > Max_Height) return;
            this.Slider_Motor.setTargetPosition(pos);
        }

        public void
    }
}