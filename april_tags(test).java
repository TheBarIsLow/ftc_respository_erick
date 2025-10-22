package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name="april-tags-erick", group="Linear Opmode")
public class april-tags-erick extends LinearOpMode {

    // Declare OpMode members.
    private DcMotor frontLeftDrive = null;


    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during robot configuration.
        frontLeftDrive  = hardwareMap.get(DcMotor.class, "front_Left_drive");

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        telemetry.addData("Status", "Running");
        telemetry.update();

        // Run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Display current motor power on Driver Station
            telemetry.addData("Front Left Motor Power", frontLeftPower);
            telemetry.update();
        }
    }
}