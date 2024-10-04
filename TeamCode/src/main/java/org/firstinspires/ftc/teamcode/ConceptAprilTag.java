package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.checkerframework.checker.units.qual.Length;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
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
        initMotors("FrontRight","FrontLeft","BackRight", "BackLeft");
        setManualExposure(6, 250);
        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch Play to start OpMode");
        telemetry.update();
        double drive  = 0;
        double strafe = 0;
        double turn   = 0;
        waitForStart();
        if (opModeIsActive()) {
            while (opModeIsActive()) {
                List<AprilTagDetection> currentDetections = aprilTag.getDetections();
                if (!currentDetections.isEmpty()) {
                    AprilTagDetection desiredTag = currentDetections.get(0);
                    //telemetryAprilTag(currentDetections);
                    double  rangeError      = (desiredTag.ftcPose.range - DESIRED_DISTANCE);
                    double  headingError    = desiredTag.ftcPose.bearing;
                    double  yawError        = desiredTag.ftcPose.yaw;
                    drive  = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
                    turn   = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN) ;
                    strafe = Range.clip(-yawError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);
                    moveRobot(drive, strafe, turn);
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
    private void initMotors(String FR, String FL, String BR, String BL) {
        motorFL = hardwareMap.get(DcMotor.class, FR);
        motorBL = hardwareMap.get(DcMotor.class, BL);
        motorFR = hardwareMap.get(DcMotor.class, FL);
        motorBR = hardwareMap.get(DcMotor.class, BR);
        motorFR.setDirection(DcMotor.Direction.FORWARD);
        motorFL.setDirection(DcMotor.Direction.REVERSE);
        motorBR.setDirection(DcMotor.Direction.FORWARD);
        motorBL.setDirection(DcMotor.Direction.REVERSE);
    }
    private void telemetryAprilTag(List<AprilTagDetection> currentDetections) {
        telemetry.addData("# AprilTags Detected", currentDetections.size());
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
            } else {
                telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
            }
        }
        telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
        telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
        telemetry.addLine("RBE = Range, Bearing & Elevation");
    }
    public void setMotors(double rightFrontPower, double leftFrontPower, double rightBackPower, double leftBackPower) {
        motorFL.setPower(leftFrontPower);
        motorFR.setPower(rightFrontPower);
        motorBL.setPower(leftBackPower);
        motorBR.setPower(rightBackPower);
    }
    public void moveRobot(double x, double y, double yaw) {
        double leftFrontPower    =  x -y -yaw;
        double rightFrontPower   =  x +y +yaw;
        double leftBackPower     =  x +y -yaw;
        double rightBackPower    =  x -y +yaw;
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
    private void setManualExposure(int exposureMS, int gain) {
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
            exposureControl.setExposure((long)exposureMS, TimeUnit.MILLISECONDS);
            sleep(20);
            GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
            gainControl.setGain(gain);
            sleep(20);
        }
    }
    private void stopRobot() {
        setMotors(0,0,0,0);
    }
}
