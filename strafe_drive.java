package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name="Erick Strafe Drive test", group="Linear Opmode")
public class BasicTeleOp extends LinearOpMode {

    // Declare OpMode members.
    private DcMotor frontLeftDrive = null;
    private DcMotor frontRightDrive = null;
    private DcMotor backLeftDrive = null;
    private DcMotor backRightDrive = null;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        double frontL=0;
        double backL=0;
        double frontR=0;
        double backR=0;
        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during robot configuration.
        frontLeftDrive  = hardwareMap.get(DcMotor.class, "front_Left_drive");
        frontRightDrive = hardwareMap.get(DcMotor.class, "front_Right_drive");
        backLeftDrive  = hardwareMap.get(DcMotor.class, "back_Left_drive");
        backRightDrive = hardwareMap.get(DcMotor.class, "back_Right_drive");

        // Most robots need the motors to be reversed or set to FORWARD.
        // You will need to determine which direction is "forward" for your robot.
        frontLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        frontRightDrive.setDirection(DcMotor.Direction.REVERSE); // Reverse one motor for tank drive
        backLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotor.Direction.REVERSE); // Reverse one motor for tank drive

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        telemetry.addData("Status", "Running");
        telemetry.update();

        // Run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            //get the data from the gamepad
            frontL=gamepad1.left_stick_y-((gamepad1.left_stick_x+gamepad1.right_stick_x)/2);
            frontR=gamepad1.right_stick_y-((gamepad1.left_stick_x+gamepad1.right_stick_x)/2);
            backR=gamepad1.right_stick_y-((gamepad1.left_stick_x+gamepad1.right_stick_x)/2);
            backL=gamepad1.left_stick_y-((gamepad1.left_stick_x+gamepad1.right_stick_x)/2);


            // Get joystick values from gamepad1 for forward movement
            double frontLeftPower = frontL;
            double frontRightPower = frontR;
            double backLeftPower = backL;
            double backRightPower = backR;


            // Set motor power
            frontLeftDrive.setPower(frontLeftPower);
            frontRightDrive.setPower(frontRightPower);
            backLeftDrive.setPower(backLeftPower);
            backRightDrive.setPower(backRightPower);

            // Display current motor power on Driver Station
            telemetry.addData("Front Left Motor Power", frontLeftPower);
            telemetry.addData("Front Right Motor Power", frontRightPower);
            telemetry.addData("Back Left Motor Power", backLeftPower);
            telemetry.addData("Back Right Motor Power", backRightPower);
            telemetry.update();
        }
    }
}