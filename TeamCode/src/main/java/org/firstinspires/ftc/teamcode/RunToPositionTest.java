package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;


@Autonomous(name="Robot: Auto Drive By Encoder Test", group="Robot")
public class RunToPositionTest extends LinearOpMode{
    DcMotor FrontRight = null;
    DcMotor BackRight = null;
    DcMotor FrontLeft = null;
    DcMotor BackLeft = null;

    @Override
    public void runOpMode() {
        FrontRight = hardwareMap.dcMotor.get("FrontRight");
        BackRight = hardwareMap.dcMotor.get("BackRight");
        FrontLeft = hardwareMap.dcMotor.get("FrontLeft");
        BackLeft = hardwareMap.dcMotor.get("BackLeft");

        FrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
        FrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
        BackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
        BackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);

        waitForStart();

        DriveForwardDistance(0.5, 10);
        TurnRightDistance(0.5, 90);
        DriveForwardDistance(0.5, 10);
        TurnRightDistance(0.5, 90);
        DriveForwardDistance(0.5, 10);

    }


    public boolean EncodersAreNegative(){
        return ((FrontRight.getCurrentPosition() < 0) && (FrontLeft.getCurrentPosition() < 0) && (BackRight.getCurrentPosition() < 0) && (BackLeft.getCurrentPosition() < 0));
    }

    public  void DriveForward(double power){
        FrontRight.setPower(power);
        FrontLeft.setPower(power);
        BackLeft.setPower(power);
        BackRight.setPower(power);
    }

    public void StopDriving(){
        DriveForward(0);
    }

    public void TurnLeft(double power){
        FrontRight.setPower(power);
        FrontLeft.setPower(-power);
        BackLeft.setPower(-power);
        BackRight.setPower(power);

    }

    public void TurnRight(double power){
        TurnLeft(-power);

    }

    public void DriveForwardDistance(double power, int distance){
        FrontRight.setMode(DcMotor.RunMode.RESET_ENCODERS);
        FrontLeft.setMode(DcMotor.RunMode.RESET_ENCODERS);
        BackRight.setMode(DcMotor.RunMode.RESET_ENCODERS);
        BackLeft.setMode(DcMotor.RunMode.RESET_ENCODERS);

        FrontRight.setTargetPosition(distance);
        FrontLeft.setTargetPosition(distance);
        BackRight.setTargetPosition(distance);
        BackLeft.setTargetPosition(distance);

        FrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        DriveForward(power);

        while (FrontLeft.isBusy() && FrontRight.isBusy() && BackLeft.isBusy() && BackRight.isBusy()){
            //waiting until stop
        }

        StopDriving();


        FrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
        FrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
        BackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
        BackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);

        sleep(500);

    }
    public void TurnLeftDistance(double power, int distance){
        FrontRight.setMode(DcMotor.RunMode.RESET_ENCODERS);
        FrontLeft.setMode(DcMotor.RunMode.RESET_ENCODERS);
        BackRight.setMode(DcMotor.RunMode.RESET_ENCODERS);
        BackLeft.setMode(DcMotor.RunMode.RESET_ENCODERS);

        FrontRight.setTargetPosition(distance);
        FrontLeft.setTargetPosition(-distance);
        BackRight.setTargetPosition(distance);
        BackLeft.setTargetPosition(-distance);

        FrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        TurnLeft(power);

        while (FrontLeft.isBusy() && FrontRight.isBusy() && BackLeft.isBusy() && BackRight.isBusy()){
            //waiting until stop
        }

        StopDriving();

        FrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
        FrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
        BackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
        BackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);

        sleep(500);

    }
    public void TurnRightDistance(double power, int distance){
        FrontRight.setMode(DcMotor.RunMode.RESET_ENCODERS);
        FrontLeft.setMode(DcMotor.RunMode.RESET_ENCODERS);
        BackRight.setMode(DcMotor.RunMode.RESET_ENCODERS);
        BackLeft.setMode(DcMotor.RunMode.RESET_ENCODERS);

        FrontRight.setTargetPosition(-distance);
        FrontLeft.setTargetPosition(distance);
        BackRight.setTargetPosition(-distance);
        BackLeft.setTargetPosition(distance);

        FrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        TurnRight(power);

        while (FrontLeft.isBusy() && FrontRight.isBusy() && BackLeft.isBusy() && BackRight.isBusy()){
            //waiting until stop
        }

        StopDriving();

        FrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
        FrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
        BackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
        BackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);

        sleep(500);

    }
}
