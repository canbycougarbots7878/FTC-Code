package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.hardware.DcMotor;

public class MovementLib {
    private void Omni_Move(double Forward, double Right, double Rotate, double speed) {
        double leftFrontPower = - Forward - Right - Rotate;
        double rightFrontPower = - Forward + Right + Rotate;
        double leftBackPower = - Forward + Right - Rotate;
        double rightBackPower = - Forward - Right + Rotate;

    }
}
