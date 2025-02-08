package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.MovementLib.*;

@TeleOp(name = "Competition TeleOp V ChatGPT", group = "Competition")
public class CompetitionRobotCode extends LinearOpMode {
    // Motors
    DcMotor Front_Right, Front_Left, Back_Right, Back_Left, Slider;

    // Servos
    Servo Arm, Arm_Lock, Claw, Wrist;

    // Control Variables
    private double robot_speed = 0.8;   // Target speed
    private double currentSpeed = 0.0;  // Smoothed speed
    private double joystickDeadzone = 0.1; // Deadzone for joystick smoothing
    private boolean claw_open = false;
    private int slider_position = 0;

    @Override
    public void runOpMode() {
        initHardware();
        waitForStart();
        Unlock_Arm();

        while (opModeIsActive()) {
            controlMovement();
            controlSlider();
            controlArm();
            toggleClaw();
            controlWrist();
            adjustSpeed();

            // Telemetry Data
            telemetry.addData("Slider Position", Slider.getCurrentPosition());
            telemetry.addData("Target Speed", robot_speed);
            telemetry.addData("Current Speed", currentSpeed);
            telemetry.update();
        }
    }

    private void initHardware() {
        // Initialize Motors
        Front_Right = hardwareMap.get(DcMotor.class, "FrontRight");
        Front_Left = hardwareMap.get(DcMotor.class, "FrontLeft");
        Back_Right = hardwareMap.get(DcMotor.class, "BackRight");
        Back_Left = hardwareMap.get(DcMotor.class, "BackLeft");

        Front_Right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Front_Left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Back_Right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Back_Left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        
        // Reverse motor direction if needed
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
    }

    private void controlMovement() {
        double speedDifference = robot_speed - currentSpeed;
        if (Math.abs(speedDifference) > 0.01) { // If speed needs adjusting
            if (speedDifference > 0) {  
                // Accelerating (increase speed smoothly)
                currentSpeed += 0.07 * speedDifference;  
            } else {  
                // Decelerating (reduce speed faster)
                currentSpeed += 0.15 * speedDifference;
            }
        }

        // Dynamic movement with deadzone
        Wheels.Omni_Move(
            applyDeadzone(gamepad1.left_stick_y + gamepad2.left_stick_y), 
            applyDeadzone(gamepad1.left_stick_x + gamepad2.left_stick_x), 
            applyDeadzone(gamepad1.right_stick_x + gamepad2.right_stick_x), 
            currentSpeed
        );
    }

    private void controlSlider() {
        if (gamepad2.dpad_down) slider_position = 0;
        if (gamepad2.dpad_left) slider_position = 1;
        if (gamepad2.dpad_up) slider_position = 2;

        switch (slider_position) {
            case 0:
                Slider.setTargetPosition(0);
                Slider.setPower(Slider.getCurrentPosition() < 100 ? 0 : 1);
                break;
            case 1:
                Slider.setTargetPosition(2201);
                Slider.setPower(1);
                break;
            case 2:
                Slider.setTargetPosition(5000);
                Slider.setPower(1);
                break;
        }
    }

    private void controlArm() {
        if (gamepad2.right_bumper) {
            robot_speed = 0.25;
            Arm.setPosition(0.31 - gamepad2.right_trigger / 5.0f);
        } else {
            Arm.setPosition(0.55);
        }
    }

    private void toggleClaw() {
        if (gamepad2.b) {
            claw_open = !claw_open;
        }
        if (claw_open) Open_Claw();
        else Close_Claw();
    }

    private void controlWrist() {
        if (gamepad2.x) Wrist_Vertical();
        else if (gamepad2.y) Wrist_Horizontal();
    }

    private void adjustSpeed() {
        if (!gamepad2.right_bumper && Slider.getCurrentPosition() < 100 && !gamepad1.left_bumper) {
            robot_speed = 0.5;
        }
        if (gamepad1.left_bumper) {
            robot_speed = 1;
        }
    }

    private void Lock_Arm() {
        Arm_Lock.setPosition(0);
    }

    private void Unlock_Arm() {
        Arm_Lock.setPosition(0.5);
    }

    private void Open_Claw() {
        Claw.setPosition(0.65);
    }

    private void Close_Claw() {
        Claw.setPosition(1);
    }

    private void Wrist_Vertical() {
        Wrist.setPosition(0);
    }

    private void Wrist_Horizontal() {
        Wrist.setPosition(0.3);
    }

    private double applyDeadzone(double value) {
        if (Math.abs(value) < joystickDeadzone) {
            return 0;
        }
        return value;
    }
}