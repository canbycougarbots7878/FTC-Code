package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Speed Test", group = "Concept")
public class speedTest extends LinearOpMode {

    DcMotor motorFR, motorBR, motorBL, motorFL;
    SparkFunOTOS myOtos;
    public MovementLib.DriveWheels Wheels;
    public MovementLib.OTOSControl OC;

    @Override
    public void runOpMode() {
        // Hardware mapping
        motorFR = hardwareMap.get(DcMotor.class, "FrontRight");
        motorBR = hardwareMap.get(DcMotor.class, "BackRight");
        motorBL = hardwareMap.get(DcMotor.class, "BackLeft");
        motorFL = hardwareMap.get(DcMotor.class, "FrontLeft");

        // ✅ 1. Correct motor order for DriveWheels
        Wheels = new MovementLib.DriveWheels(motorFR, motorFL, motorBR, motorBL);

        // Motor behavior
        DcMotor[] motors = {motorFR, motorBR, motorBL, motorFL};
        for (DcMotor m : motors) {
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        motorFR.setDirection(DcMotor.Direction.FORWARD);
        motorBR.setDirection(DcMotor.Direction.FORWARD);
        motorBL.setDirection(DcMotor.Direction.REVERSE);
        motorFL.setDirection(DcMotor.Direction.REVERSE);

        // OTOS setup
        myOtos = hardwareMap.get(SparkFunOTOS.class, "sensor_otos");
        OC = new MovementLib.OTOSControl(Wheels, myOtos, telemetry, hardwareMap);

        waitForStart();

        // Initial movement setup
        double power = 1.0;
        double powerStep = 0.1;
        double x = 0.1;
        double dx = 0.1;

        while (opModeIsActive()) {
            SparkFunOTOS.Pose2D pos = myOtos.getPosition();

            // Move using OTOS
            OC.OTOS_Move(x, 0, 0, power, true);

            // ✅ 5. Improved telemetry formatting
            telemetry.addLine(String.format("Target X: %.2f | Power: %.2f", x, power));
            telemetry.addLine(String.format("Position => X: %.3f, Y: %.3f, H: %.1f°", pos.x, pos.y, pos.h));
            telemetry.update();

            x += dx;
            power -= powerStep;

            // ✅ 2. Safe floating-point comparison
            if (Math.abs(power - powerStep) < 1e-6) {
                powerStep /= 10;
            }

            // ✅ 3. Prevent tight loop flooding motors and telemetry
            sleep(5000);
        }
    }
}
