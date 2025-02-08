package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.MovementLib.*;

@TeleOp(name = "Competition TeleOp V ChatGPT Combined", group = "Competition")
public class CompetitionRobotCodeCombined extends LinearOpMode {
    Servo Arm = null;
    DcMotor Slider = null;
    Servo Arm_Lock = null;
    Servo Claw = null;
    Servo Wrist = null;

    private double robot_speed = 0.8;   // Target speed
    private double currentSpeed = 0.0;  // Smoothed speed
    private double accelerationFactor = 0.07;  // Increase rate (adjustable)
    private double decelerationFactor = 0.15;  // Faster stopping (adjustable)
    private double joystickDeadzone = 0.1; // Joystick deadzone for smoother control
    private double slideFactor = 1.0; // Slider position scaling factor
    private int slider_position = 0; // Track the slider position

    @Override
    public void runOpMode() {
        // Initialize Motors
        DcMotor Front_Right = hardwareMap.get(DcMotor.class, "FrontRight");
        DcMotor Front_Left = hardwareMap.get(DcMotor.class, "FrontLeft");
        DcMotor Back_Right = hardwareMap.get(DcMotor.class, "BackRight");
        DcMotor Back_Left = hardwareMap.get(DcMotor.class, "BackLeft");

        Front_Right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Front_Left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Back_Right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Back_Left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

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

        boolean claw_open = false;
        boolean previousBState = false;

        waitForStart();
        Unlock_Arm();

        while(opModeIsActive()) {
            // **Refined Acceleration and Deceleration Control**
            double speedDifference = robot_speed - currentSpeed;
            if (Math.abs(speedDifference) > 0.01) { // If speed needs adjusting
                if (speedDifference > 0) {  
                    // Accelerating (increase speed smoothly)
                    currentSpeed += accelerationFactor * speedDifference;  
                } else {  
                    // Decelerating (reduce speed faster)
                    currentSpeed += decelerationFactor * speedDifference;
                }
            }

            // **Dynamic Speed Based on Joystick Input**
            double joystickMagnitude = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);
            double dynamicAcceleration = joystickMagnitude * 0.1; // Scale acceleration based on joystick input
            double dynamicDeceleration = joystickMagnitude * 0.2; // Faster deceleration for better control

            // Apply smoothed speed to movement
            Wheels.Omni_Move(
                applyDeadzone(gamepad1.left_stick_y + gamepad2.left_stick_y), 
                applyDeadzone(gamepad1.left_stick_x + gamepad2.left_stick_x), 
                applyDeadzone(gamepad1.right_stick_x + gamepad2.right_stick_x), 
                currentSpeed
            );

            // **Slider Position Scaling**
            slideFactor = 1.0 - (Slider.getCurrentPosition() / 5000.0); // Scale between 1 and 0 based on slider position
            robot_speed = 0.5 + (0.5 * slideFactor); // Min speed of 0.5 when fully extended

            // **Adjust Slider Position Based on D-Pad Inputs**
            if (gamepad2.dpad_down) slider_position = 0;
            if (gamepad2.dpad_left) slider_position = 1;
            if (gamepad2.dpad_up) slider_position = 2;

            // Control slider movement with gradual speed changes
            if (slider_position == 0) {
                Slider.setTargetPosition(0);
                if (Slider.getCurrentPosition() < 100) {
                    Slider.setPower(0);
                    if (Slider.getCurrentPosition() < 30) {
                        Slider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        Slider.setTargetPosition(0);
                        Slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    }
                } else {
                    Slider.setPower(1);
                }
            } else if (slider_position == 1) {
                Slider.setTargetPosition(2201);
                Slider.setPower(1);
            } else {
                Slider.setTargetPosition(5000);
                Slider.setPower(1);
            }

            // **Arm Movement Control**
            if(gamepad2.right_bumper) {
                robot_speed = 0.25;
                Arm.setPosition(0.31 - gamepad2.right_trigger / 5.0f);
            } else {
                Arm.setPosition(0.55);
            }

            // **Claw Toggle Mechanism**
            if (gamepad2.b && !previousBState) {
                claw_open = !claw_open;
            }
            previousBState = gamepad2.b;

            // **Wrist Control**
            if (gamepad2.x) Wrist_Vertical();
            else if (gamepad2.y) Wrist_Horizontal();

            // Open or close claw based on state
            if (claw_open) Open_Claw();
            else Close_Claw();

            // **Reset Slider Encoders if Needed**
            if (gamepad2.start) {
                Slider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                Slider.setTargetPosition(0);
                Slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
            if (gamepad1.back && gamepad2.back) {
                Slider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                Slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }

            // **Adjust Robot Speed Limits Based on Inputs**
            if (!gamepad2.right_bumper && Slider.getCurrentPosition() < 100 && !gamepad1.left_bumper) {
                robot_speed = 0.5;
            }
            if (gamepad1.left_bumper) {
                robot_speed = 1;
            }

            // **Arm Lock**
            if (gamepad2.back) Lock_Arm();

            // Telemetry Data
            telemetry.addData("Slider Position", Slider.getCurrentPosition());
            telemetry.addData("Target Speed", robot_speed);
            telemetry.addData("Current Speed", currentSpeed);
            telemetry.update();
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

    // **Apply Deadzone to Joystick Inputs**
    private double applyDeadzone(double value) {
        if (Math.abs(value) < joystickDeadzone) {
            return 0;
        } else {
            return value;
        }
    }
}