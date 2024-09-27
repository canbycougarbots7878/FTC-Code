package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.checkerframework.checker.units.qual.Length;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
@TeleOp(name = "Concept: AprilTag", group = "Concept")
public class ConceptAprilTag extends LinearOpMode {
    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;
    private DcMotor motorFR = null;
    private DcMotor motorFL = null;
    private DcMotor motorBR = null;
    private DcMotor motorBL = null;
    @Override
    public void runOpMode() {
        initAprilTag();
        initMotors("FrontRight","FrontLeft","BackRight", "BackLeft");
        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch Play to start OpMode");
        telemetry.update();
        waitForStart();
        if (opModeIsActive()) {
            while (opModeIsActive()) {
                List<AprilTagDetection> currentDetections = aprilTag.getDetections();
                telemetryAprilTag(currentDetections);
                telemetry.update();

                if(!currentDetections.isEmpty()) {
                    AprilTagDetection detection = currentDetections.get(0);
                    if (detection.metadata != null) {
                        setRobot(8 - detection.ftcPose.y / 4, - detection.ftcPose.x / 2);
                    }
                }
                else {
                    stopRobot();
                }
                sleep(20);
            }
        }
        visionPortal.close();
    }
    private void initAprilTag() {
        aprilTag = new AprilTagProcessor.Builder().build();
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
    private void setMotors(double FR, double FL, double BR, double BL) {
        motorFR.setPower(FR);
        motorFL.setPower(FL);
        motorBR.setPower(BR);
        motorBL.setPower(BL);
    }
    private void moveRobot(double Forward, double Right) {
        double leftFrontPower  = - Forward + Right;
        double rightFrontPower = - Forward - Right;
        double leftBackPower   =  Forward - Right;
        double rightBackPower  =  Forward + Right;
        setMotors(rightFrontPower,leftFrontPower,rightBackPower,leftBackPower);
    }
    private void stopRobot() {
        setMotors(0,0,0,0);
    }
    private void turnRobot(double amount) {
        setMotors(-amount,amount,-amount,amount);
    }
    private void setRobot(double Forward, double Right, double turn) {
        double leftFrontPower  = - Forward + Right + turn;
        double rightFrontPower = - Forward - Right - turn;
        double leftBackPower   =  Forward - Right + turn;
        double rightBackPower  =  Forward + Right - turn;
        setMotors(rightFrontPower,leftFrontPower,rightBackPower,leftBackPower);
    }
}
