// Date: 2025-02-08
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.MovementLib.*;

@TeleOp(name = "Competition TeleOp", group = "Competition")
public class CompetitionTeleopV2 extends LinearOpMode {

    // Hardware components
    private Servo Arm, Arm_Lock, Claw, Wrist;
    private DcMotor Slider;

    // Constants for servo positions
    private static final double ARM_DOWN_POSITION = 0.55;
    private static final double ARM_UP_POSITION = 0.21;
    private static final double CLAW_OPEN_POSITION = 0.65;
    private static final double CLAW_CLOSED_POSITION = 1;
    private static final double WRIST_VERTICAL_POSITION = 0;
    private static final double WRIST_HORIZONTAL_POSITION = 0.3;

    // Robot control variables
    private int slider_position = 0;
    private boolean claw_open = false;
    private double robot_speed = 0.8;
    private double currentSpeed = 0.8; // Smooth speed transition

    // Telemetry tracking
    private int lastSliderPosition = -1;
    private double lastArmPosition = -1;

    @Override
    public void runOpMode() {
        // Initialize Motors
        DcMotor Front_Right = hardwareMap.get(DcMotor.class, "FrontRight");
        DcMotor Front_Left = hardwareMap.get(DcMotor.class, "FrontLeft");
        DcMotor Back_Right = hardwareMap.get(DcMotor.class, "BackRight");
        DcMotor Back_Left = hardwareMap.get(DcMotor.class, "BackLeft");

        // Set motors to run without encoders for teleop movement
        Front_Right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Front_Left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Back_Right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Back_Left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Create drive system
        DriveWheels Wheels = new DriveWheels(Front_Right, Front_Left, Back_Right, Back_Left);
        Front_Right.setDirection(DcMotorSimple.Direction.REVERSE);
        Back_Right.setDirection(DcMotorSimple.Direction.REVERSE);

        // Initialize Slider
        Slider = hardwareMap.get(DcMotor.class, "Slide Arm");
        Slider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Slider.setTargetPosition(0);
        Slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Slider.setPower(0);

        // Initialize Servos
        Arm = hardwareMap.get(Servo.class, "Arm");
        Arm_Lock = hardwareMap.get(Servo.class, "Arm Lock");
        Claw = hardwareMap.get(Servo.class, "Claw");
        Wrist = hardwareMap.get(Servo.class, "Wrist");

        waitForStart();
        Unlock_Arm();

        while (opModeIsActive()) {
            // Drive control with dead zone and smooth acceleration
            double moveY = applyDeadZone(gamepad1.left_stick_y, 0.1);
            double moveX = applyDeadZone(gamepad1.left_stick_x, 0.1);
            double rotate = applyDeadZone(gamepad1.right_stick_x, 0.1);
            updateRobotSpeed();
            Wheels.Omni_Move(moveY, moveX, rotate, currentSpeed);

            // Slider control
            updateSliderPosition();

            // Arm control
            controlArm();

            // Claw control
            if (gamepad2.a) claw_open = false;
            if (gamepad2.b) claw_open = true;
            if (claw_open) Open_Claw();
            else Close_Claw();

            // Wrist control
            if (gamepad2.x) Wrist_Vertical();
            else if (gamepad2.y) Wrist_Horizontal();

            // Emergency stop
            if (gamepad1.start && gamepad1.back) stopAllMotors();

            // Arm locking control
            if (gamepad2.back) Lock_Arm();

            // Optimized telemetry update
            updateTelemetry();
        }
    }

    /** Applies a dead zone to joystick inputs to prevent unintended movement. */
    private double applyDeadZone(double value, double threshold) {
        return (Math.abs(value) > threshold) ? value : 0;
    }

    /** Updates the robot's speed gradually for smooth acceleration and deceleration. */
    private void updateRobotSpeed() {
        double speedIncrement = 0.02;

        if (gamepad1.left_bumper) robot_speed = 1;
        else if (gamepad2.right_bumper) robot_speed = 0.25;
        else if (Slider.getCurrentPosition() < 100 && !gamepad1.left_bumper) robot_speed = 0.5;
        else robot_speed = 0.8;

        if (currentSpeed < robot_speed) {
            currentSpeed += speedIncrement;
            if (currentSpeed > robot_speed) currentSpeed = robot_speed;
        } else if (currentSpeed > robot_speed) {
            currentSpeed -= speedIncrement;
            if (currentSpeed < robot_speed) currentSpeed = robot_speed;
        }
    }

    /** Updates the slider position dynamically and adjusts power based on distance. */
    private void updateSliderPosition() {
        if (gamepad2.dpad_down) setSliderPosition(0);
        if (gamepad2.dpad_left) setSliderPosition(2201);
        if (gamepad2.dpad_up) setSliderPosition(5000);
    }

    /** Sets the slider position with dynamic power adjustment. */
    private void setSliderPosition(int position) {
        int error = Math.abs(position - Slider.getCurrentPosition());
        double power = (error > 500) ? 1.0 : (error > 200) ? 0.5 : 0.25;
        Slider.setTargetPosition(position);
        Slider.setPower(power);
    }

    /** Controls the arm movement using proportional adjustment. */
    private void controlArm() {
        if (gamepad2.right_bumper) {
            Arm.setPosition(ARM_UP_POSITION - gamepad2.right_trigger / 5.0);
        } else {
            Arm.setPosition(ARM_DOWN_POSITION);
        }
    }

    /** Stops all motors immediately for an emergency stop. */
    private void stopAllMotors() {
        Slider.setPower(0);
        Claw.setPosition(CLAW_CLOSED_POSITION);
        Arm.setPosition(ARM_DOWN_POSITION);
        Wrist.setPosition(WRIST_VERTICAL_POSITION);
        robot_speed = 0;
    }

    /** Locks the arm in place. */
    private void Lock_Arm() {
        Arm_Lock.setPosition(0);
    }

    /** Unlocks the arm. */
    private void Unlock_Arm() {
        Arm_Lock.setPosition(0.5);
    }

    /** Opens the claw. */
    private void Open_Claw() {
        Claw.setPosition(CLAW_OPEN_POSITION);
    }

    /** Closes the claw. */
    private void Close_Claw() {
        Claw.setPosition(CLAW_CLOSED_POSITION);
    }

    /** Positions the wrist vertically. */
    private void Wrist_Vertical() {
        Wrist.setPosition(WRIST_VERTICAL_POSITION);
    }

    /** Positions the wrist horizontally. */
    private void Wrist_Horizontal() {
        Wrist.setPosition(WRIST_HORIZONTAL_POSITION);
    }

    /** Updates telemetry only when values change significantly. */
    private void updateTelemetry() {
        int sliderPos = Slider.getCurrentPosition();
        double armPos = Arm.getPosition();

        if (sliderPos != lastSliderPosition || armPos != lastArmPosition) {
            telemetry.addData("Slider Position", sliderPos);
            telemetry.addData("Arm Position", armPos);
            telemetry.addData("Robot Speed", currentSpeed);
            telemetry.update();
            lastSliderPosition = sliderPos;
            lastArmPosition = armPos;
        }
    }
}