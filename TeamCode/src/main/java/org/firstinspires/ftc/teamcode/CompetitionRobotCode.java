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

    // Stop sequence tracking
    private String controller1Sequence = "";
    private String controller2Sequence = "";

    private final String STOP_SEQUENCE_1 = "ABXY";  // Controller 1 button sequence (A, B, X, Y)
    private final String STOP_SEQUENCE_2 = "UDLR";  // Controller 2 Dpad sequence (Up, Down, Left, Right)

    @Override
    public void runOpMode() {
        initHardware();
        waitForStart();
        Unlock_Arm();

        while (opModeIsActive()) {
            // Check for the stop sequence press
            if (checkStopSequence()) {
                stopProgram();
                return; // Exit the loop and stop the program
            }

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

    // Checks if both controllers have pressed the correct sequence
    private boolean checkStopSequence() {
        // Update sequences for each controller
        updateController1Sequence();
        updateController2Sequence();

        // If both sequences match the stop sequence, return true
        if (controller1Sequence.equals(STOP_SEQUENCE_1) && controller2Sequence.equals(STOP_SEQUENCE_2)) {
            return true;
        }
        return false;
    }

    // Update the sequence for Controller 1 (gamepad1)
    private void updateController1Sequence() {
        if (gamepad1.a) controller1Sequence += "A";
        if (gamepad1.b) controller1Sequence += "B";
        if (gamepad1.x) controller1Sequence += "X";
        if (gamepad1.y) controller1Sequence += "Y";

        // Limit the sequence to the length of 4 characters
        if (controller1Sequence.length() > 4) controller1Sequence = controller1Sequence.substring(controller1Sequence.length() - 4);
    }

    // Update the sequence for Controller 2 (gamepad2)
    private void updateController2Sequence() {
        if (gamepad2.dpad_up) controller2Sequence += "U";
        if (gamepad2.dpad_down) controller2Sequence += "D";
        if (gamepad2.dpad_left) controller2Sequence += "L";
        if (gamepad2.dpad_right) controller2Sequence += "R";

        // Limit the sequence to the length of 4 characters
        if (controller2Sequence.length() > 4) controller2Sequence = controller2Sequence.substring(controller2Sequence.length() - 4);
    }

    // Stops the robot and the entire program
    private void stopProgram() {
        // Stop the motors
        Front_Right.setPower(0);
        Front_Left.setPower(0);
        Back_Right.setPower(0);
        Back_Left.setPower(0);
        Slider.setPower(0);
    
        // Provide telemetry feedback that the stop sequence was activated
        telemetry.addData("Program Stopped", "Stop Sequence Activated");
        telemetry.update();
    
        // Optionally add a brief delay for feedback visibility
        sleep(500);  // Sleep for 500 milliseconds to allow feedback display
    
        // Clear the sequences to avoid triggering the stop on restart
        controller1Sequence = "";
        controller2Sequence = "";
    
        // End the OpMode (this stops the program)
        requestOpModeStop();
    }
}