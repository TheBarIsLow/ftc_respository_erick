package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="2025 TeleOp - Fixed Drivetrai hopefully", group="Linear Opmode")
public class HornStrafeCode extends LinearOpMode {

    private DcMotor frontLeft, backLeft, frontRight, backRight;
    private DcMotor Intake, flyWheel;
    private Servo flyWheelServo;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // --- Hardware Map ---
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        Intake = hardwareMap.get(DcMotor.class, "Intake");
        flyWheel = hardwareMap.get(DcMotor.class, "flyWheel");
        flyWheelServo = hardwareMap.get(Servo.class, "flyWheelServo");

        // --- Motor Directions ---
        frontLeft.setDirection(DcMotor.Direction.REVERSE);   // chain driven
        frontRight.setDirection(DcMotor.Direction.FORWARD);  // chain driven
        backLeft.setDirection(DcMotor.Direction.FORWARD);    // direct drive
        backRight.setDirection(DcMotor.Direction.REVERSE);   // direct drive

        Intake.setDirection(DcMotor.Direction.FORWARD);
        flyWheel.setDirection(DcMotor.Direction.REVERSE);

        // --- Run without encoders ---
        DcMotor[] motors = {frontLeft, backLeft, frontRight, backRight, Intake, flyWheel};
        for (DcMotor m : motors) {
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        flyWheelServo.setPosition(0.49);
        waitForStart();

        while (opModeIsActive()) {

            // ===== DRIVE CONTROL =====
            double y = -gamepad1.left_stick_y;   // forward/back
            double x = gamepad1.left_stick_x;    // turning
            double strafe = gamepad1.right_stick_x; // strafing

            // Apply deadzone to prevent drift
            double deadzone = 0.1;
            if (Math.abs(y) < deadzone) y = 0;
            if (Math.abs(x) < deadzone) x = 0;
            if (Math.abs(strafe) < deadzone) strafe = 0;

            // Calculate individual motor powers (mecanum drive formula)
            double frontLeftPower = y + x + strafe;
            double frontRightPower = y - x - strafe;
            double backLeftPower = y - x + strafe;
            double backRightPower = y + x - strafe;

            // Normalize powers to ensure no value exceeds ±1.0
            double maxPower = Math.max(Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower)), 
                                      Math.max(Math.abs(backLeftPower), Math.abs(backRightPower)));
            if (maxPower > 1.0) {
                frontLeftPower /= maxPower;
                frontRightPower /= maxPower;
                backLeftPower /= maxPower;
                backRightPower /= maxPower;
            }

            // Set motor powers
            frontLeft.setPower(frontLeftPower);
            frontRight.setPower(frontRightPower);
            backLeft.setPower(backLeftPower);
            backRight.setPower(backRightPower);

            // ===== INTAKE ALWAYS ON =====
            Intake.setPower(1.0);

            // ===== FLYWHEEL CONTROL =====
            if (gamepad1.left_bumper) {
                flyWheel.setPower(0.75);
            } else if (gamepad1.left_trigger > 0.1) {
                flyWheel.setPower(0.9);
            } else {
                flyWheel.setPower(0);
            }

            // ===== SERVO FLICK =====
            if (gamepad1.right_bumper) {
                flyWheelServo.setPosition(0.27);
                sleep(250);
                flyWheelServo.setPosition(0.49);
                sleep(500);
            }

            // Telemetry
            telemetry.addData("Status", "Running");
            telemetry.addData("Front Left Power", "%.2f", frontLeftPower);
            telemetry.addData("Front Right Power", "%.2f", frontRightPower);
            telemetry.addData("Back Left Power", "%.2f", backLeftPower);
            telemetry.addData("Back Right Power", "%.2f", backRightPower);
            telemetry.addData("Stick Values", "Y: %.2f, X: %.2f, Strafe: %.2f", y, x, strafe);
            telemetry.update();
        }
    }
}
