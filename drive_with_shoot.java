package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name="Erick_Strafe_Drive_test", group="Linear Opmode")
public class Erick_Strafe_Drive_test extends LinearOpMode {

    // Declare OpMode members.
    private DcMotor frontLeftDrive = null;
    private DcMotor frontRightDrive = null;
    private DcMotor backLeftDrive = null;
    private DcMotor backRightDrive = null;
    private DcMotor shootMotor = null;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        double frontL=0;
        double backL=0;
        double frontR=0;
        double backR=0;
        int shootM=0;


        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during robot configuration.
        frontLeftDrive  = hardwareMap.get(DcMotor.class, "front_Left_drive");
        frontRightDrive = hardwareMap.get(DcMotor.class, "front_Right_drive");
        backLeftDrive  = hardwareMap.get(DcMotor.class, "back_Left_drive");
        backRightDrive = hardwareMap.get(DcMotor.class, "back_Right_drive");
        shootMotor = hardwareMap.get(DcMotor.class,"shooting_motor");


        frontLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        frontRightDrive.setDirection(DcMotor.Direction.REVERSE); // Reverse one motor for tank drive
        backLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotor.Direction.REVERSE); // Reverse one motor for tank drive
        shootMotor.setDirection(DcMotor.Direction.FORWARD);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        telemetry.addData("Status", "Running");
        telemetry.update();

        // Run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            //get the data from the gamepad
            frontL=gamepad1.left_stick_y+((gamepad1.left_stick_x+gamepad1.right_stick_x)/2);
            frontR=gamepad1.right_stick_y-((gamepad1.left_stick_x+gamepad1.right_stick_x)/2);
            backR=gamepad1.right_stick_y+((gamepad1.left_stick_x+gamepad1.right_stick_x)/2);
            backL=gamepad1.left_stick_y-((gamepad1.left_stick_x+gamepad1.right_stick_x)/2);
            //makes the trigger input into an integer for shootMotor
            if(gamepad1_a){
                shootM=1;
            }else{
                shootM=0;
            }
/*
            // Get joystick values from gamepad1 for forward movement
            double  frontLeftPower = frontL;
            double frontRightPower = frontR;
            double backLeftPower = backL;
            double backRightPower = backR;
*/

            // Set motor power
            frontLeftDrive.setPower(frontL);
            frontRightDrive.setPower(frontR);
            backLeftDrive.setPower(backL);
            backRightDrive.setPower(backR);
            shootMotor.setPower(shootM);

            // Display current motor power on Driver Station
            telemetry.addData("Front Left Motor Power", frontL);
            telemetry.addData("Front Right Motor Power", frontR);
            telemetry.addData("Back Left Motor Power", backL);
            telemetry.addData("Back Right Motor Power", backR);
            telemetry.addData("Shoot Motor Power", shootM);
            telemetry.update();
        }
    }
}