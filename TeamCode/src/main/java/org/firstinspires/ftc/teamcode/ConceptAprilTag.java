package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;

@TeleOp(name = "Concept: AprilTag", group = "Concept")

public class ConceptAprilTag extends LinearOpMode {

    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    private DcMotor motorFR      = null;
    private DcMotor motorFL      = null;
    private DcMotor motorBR      = null;
    private DcMotor motorBL      = null;

    final double SPEED_GAIN      = 0.02;
    final double STRAFE_GAIN     = 0.015;
    final double TURN_GAIN       = 0.01;
    final double MAX_AUTO_SPEED  = 0.5;
    final double MAX_AUTO_STRAFE = 0.5;
    final double MAX_AUTO_TURN   = 0.3;
    final double DESIRED_DISTANCE = 12.0;

    @Override

    public void runOpMode() {
        initAprilTag();
        initMotors();
        setManualExposure();

        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch Play to start OpMode");
        telemetry.update();

        double drive;
        double strafe;
        double turn;

        waitForStart();

        if (opModeIsActive()) {

            while (opModeIsActive()) {

                List<AprilTagDetection> currentDetections = aprilTag.getDetections();

                if (!currentDetections.isEmpty()) {

                    AprilTagDetection desiredTag = currentDetections.get(0);
                    //telemetryAprilTag(currentDetections);

                    if (desiredTag.ftcPose.range > DESIRED_DISTANCE){

                        driveRobot(desiredTag.ftcPose.range - DESIRED_DISTANCE);

                    }
                    else if (desiredTag.ftcPose.range < DESIRED_DISTANCE) {

                        driveRobot(desiredTag.ftcPose.range - DESIRED_DISTANCE);

                    }
                    else

                        stopRobot();

                    double  rangeError      = (desiredTag.ftcPose.range - DESIRED_DISTANCE);
                    double  headingError    = desiredTag.ftcPose.bearing;
                    double  yawError        = desiredTag.ftcPose.yaw;

                    sleep(10);
                }
                else {
                    stopRobot();
                }
            }
        }
        visionPortal.close();
    }
    private void initAprilTag() {

        aprilTag = new AprilTagProcessor.Builder().build();
        aprilTag.setDecimation(2);

        VisionPortal.Builder builder = new VisionPortal.Builder();
        builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        builder.addProcessor(aprilTag);

        visionPortal = builder.build();
    }
    private void initMotors() {
        motorFL = hardwareMap.get(DcMotor.class, "FrontRight");
        motorBL = hardwareMap.get(DcMotor.class, "BackLeft");
        motorFR = hardwareMap.get(DcMotor.class, "FrontLeft");
        motorBR = hardwareMap.get(DcMotor.class, "BackRight");

        motorFR.setDirection(DcMotor.Direction.FORWARD);
        motorFL.setDirection(DcMotor.Direction.REVERSE);
        motorBR.setDirection(DcMotor.Direction.FORWARD);
        motorBL.setDirection(DcMotor.Direction.REVERSE);
    }

    public void setMotors(double rightFrontPower, double leftFrontPower, double rightBackPower, double leftBackPower) {

        motorFL.setPower(leftFrontPower);
        motorFR.setPower(rightFrontPower);
        motorBL.setPower(-leftBackPower);
        motorBR.setPower(-rightBackPower);

    }
    public void driveRobot(double x) {

        double leftFrontPower    =  x;
        double rightFrontPower   =  x;
        double leftBackPower     =  x;
        double rightBackPower    =  x;
        double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));

        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {

            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;

        }

        setMotors(rightFrontPower,leftFrontPower,rightBackPower,leftBackPower);

    }
    private void setManualExposure() {

        if (visionPortal == null) {

            return;
        }
        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            telemetry.addData("Camera", "Waiting");
            telemetry.update();

            while (!isStopRequested() && (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {

                sleep(20);

            }

            telemetry.addData("Camera", "Ready");
            telemetry.update();

        }
        if (!isStopRequested()) {

            ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);

            if (exposureControl.getMode() != ExposureControl.Mode.Manual) {

                exposureControl.setMode(ExposureControl.Mode.Manual);
                sleep(50);

            }

            exposureControl.setExposure(6, TimeUnit.MILLISECONDS);
            sleep(20);

            GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
            gainControl.setGain(250);
            sleep(20);

        }
    }

    private void stopRobot() {
        setMotors(0,0,0,0);
    }

}
