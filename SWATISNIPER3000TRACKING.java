//April Tag Detection and Tracking
//WORKING CODE

//NEED TO CHANGE DESIRED DISTANCE 

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;

@TeleOp(name = "HornSWATISNIPER3000TRACKINGTest", group = "Testing")
public class HornSWATISNIPER3000TRACKINGTest extends LinearOpMode {

    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;
    
    // Drive motors
    private DcMotor frontLeft, backLeft, frontRight, backRight;
    
    // Tracking parameters
    private double DESIRED_DISTANCE = 12.0; // inches
    private double SPEED_GAIN = 0.02;
    private double STRAFE_GAIN = 0.015;
    private double TURN_GAIN = 0.01;
    
    // PID constants for smooth tracking
    private double RANGE_TOLERANCE = 1.0; // inches
    private double BEARING_TOLERANCE = 2.0; // degrees

    @Override
    public void runOpMode() {
        // Initialize drive motors
        initDriveMotors();
        
        // Initialize AprilTag Processor with enhanced visualization
        aprilTag = new AprilTagProcessor.Builder()
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .setTagLibrary(AprilTagGameDatabase.getCurrentGameTagLibrary())
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setDrawCubeProjection(true)
                .setDrawAxes(true)
                .build();

        // Create the vision portal with camera
        VisionPortal.Builder builder = new VisionPortal.Builder();
        builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);
        builder.enableLiveView(true);
        builder.setAutoStopLiveView(true);
        builder.addProcessor(aprilTag);
        
        visionPortal = builder.build();

        telemetry.addData("->", "SWATISNIPER3000 TRACKING MODE READY");
        telemetry.addData("->", "Desired Distance: %.1f inches", DESIRED_DISTANCE);
        telemetry.addData("->", "Press Play to begin tracking");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            boolean targetFound = false;
            double drive = 0, strafe = 0, turn = 0;

            // Check all detected tags
            if (!aprilTag.getDetections().isEmpty()) {
                AprilTagDetection detection = aprilTag.getDetections().get(0); // Track first detected tag
                targetFound = true;

                if (detection.metadata != null) {
                    // Calculate tracking corrections
                    double rangeError = detection.ftcPose.range - DESIRED_DISTANCE;
                    double bearingError = detection.ftcPose.bearing;
                    double yawError = detection.ftcPose.yaw;

                    // Calculate motor powers using PID-like control
                    drive = -rangeError * SPEED_GAIN;
                    strafe = -bearingError * STRAFE_GAIN;
                    turn = -yawError * TURN_GAIN;

                    // Apply motor powers for tracking
                    moveRobot(drive, strafe, turn);

                    // Display tracking info
                    telemetry.addLine("\n=== SWATISNIPER3000 TRACKING ACTIVE ===");
                    telemetry.addData("TARGET", "#%d (%s)", detection.id, detection.metadata.name);
                    telemetry.addData("RANGE", "Current: %.1f\" | Target: %.1f\" | Error: %.1f\"", 
                                     detection.ftcPose.range, DESIRED_DISTANCE, rangeError);
                    telemetry.addData("BEARING", "%.1f deg | Correction: %.3f", bearingError, strafe);
                    telemetry.addData("YAW", "%.1f deg | Correction: %.3f", yawError, turn);
                    
                    // Check if target is locked
                    boolean rangeLocked = Math.abs(rangeError) < RANGE_TOLERANCE;
                    boolean bearingLocked = Math.abs(bearingError) < BEARING_TOLERANCE;
                    boolean yawLocked = Math.abs(yawError) < BEARING_TOLERANCE;
                    
                    if (rangeLocked && bearingLocked && yawLocked) {
                        telemetry.addLine("*** TARGET LOCKED ***");
                    } else {
                        telemetry.addLine(">>> TRACKING... <<<");
                    }
                    
                    telemetry.addData("DRIVE POWER", "%.3f", drive);
                    telemetry.addData("STRAFE POWER", "%.3f", strafe);
                    telemetry.addData("TURN POWER", "%.3f", turn);
                }
                telemetry.addData("TAGS DETECTED", aprilTag.getDetections().size());
            } else {
                // No target found - stop motors
                moveRobot(0, 0, 0);
                telemetry.addLine("SWATISNIPER3000: No targets in sight");
                telemetry.addData("STATUS", "Scanning... Stop motors");
            }

            // System status
            telemetry.addLine("\n=== SYSTEM STATUS ===");
            telemetry.addData("Camera", visionPortal.getCameraState());
            telemetry.addData("FPS", "%.1f", visionPortal.getFps());
            telemetry.addData("Target Found", targetFound);
            
            telemetry.update();
            sleep(20);
        }
        
        // Clean up - stop motors
        moveRobot(0, 0, 0);
        visionPortal.close();
    }

    private void initDriveMotors() {
        // Initialize drive motors - update these names to match your robot configuration
        frontLeft = hardwareMap.get(DcMotor.class, "front_left");
        backLeft = hardwareMap.get(DcMotor.class, "back_left");
        frontRight = hardwareMap.get(DcMotor.class, "front_right");
        backRight = hardwareMap.get(DcMotor.class, "back_right");

        // Set motor directions (adjust based on your robot)
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.FORWARD);

        // Set motors to brake when power is zero
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    private void moveRobot(double drive, double strafe, double turn) {
        // Calculate motor powers for mecanum drive
        double frontLeftPower = drive + strafe + turn;
        double backLeftPower = drive - strafe + turn;
        double frontRightPower = drive - strafe - turn;
        double backRightPower = drive + strafe - turn;

        // Normalize powers to maintain proper proportions
        double maxPower = Math.max(Math.max(Math.abs(frontLeftPower), Math.abs(backLeftPower)),
                                  Math.max(Math.abs(frontRightPower), Math.abs(backRightPower)));
        if (maxPower > 1.0) {
            frontLeftPower /= maxPower;
            backLeftPower /= maxPower;
            frontRightPower /= maxPower;
            backRightPower /= maxPower;
        }

        // Set motor powers
        frontLeft.setPower(frontLeftPower);
        backLeft.setPower(backLeftPower);
        frontRight.setPower(frontRightPower);
        backRight.setPower(backRightPower);
    }
}
