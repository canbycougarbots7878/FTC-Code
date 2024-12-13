package org.firstinspires.ftc.teamcode;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

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
            Front_Right.setDirection(DcMotor.Direction.REVERSE);
            Front_Left.setDirection(DcMotor.Direction.REVERSE);
            Back_Left.setDirection(DcMotor.Direction.REVERSE);
        }
        public void Omni_Move(double Forward, double Right, double Rotate, double speed) {
            double Front_Left_Power = - Forward - Right - Rotate;
            double Front_Right_Power = - Forward + Right + Rotate;
            double Back_Right_Power = - Forward - Right + Rotate;
            double Back_Left_Power = - Forward + Right - Rotate;
            this.Front_Right.setPower(Front_Right_Power);
            this.Front_Left.setPower(Front_Left_Power);
            this.Back_Right.setPower(Back_Right_Power);
            this.Back_Left.setPower(Back_Left_Power);
        }
    }
    public static final int REVERSE_FRONT_RIGHT = 1;
    public static final int REVERSE_FRONT_LEFT = 2;
    public static final int REVERSE_BACK_RIGHT = 4;
    public static final int REVERSE_BACK_LEFT = 8;
}