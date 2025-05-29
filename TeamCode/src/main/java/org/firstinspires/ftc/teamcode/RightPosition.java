package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Right Position", group = "Positions")
public class RightPosition extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        oneProgramTest.Position posUtil = new oneProgramTest.Position(hardwareMap, telemetry, this);

        waitForStart();

        posUtil.goToPosition(1);
    }
}
