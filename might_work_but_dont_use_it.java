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

@TeleOp(name = "SWATISNIPER3000", group = "Concept");
public class Swatisniper3000 extends LinearOpMode {

    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;
    private DcMotor frontLeft = null;
    private DcMotor frontRight = null;
    private DcMotor backLeft = null;
    private DcMotor backRight = null;

    @Override
    public void runOpMode() {

        frontLeft  = hardwareMap.get(DcMotor.class, "front_Left_drive");
        frontRight = hardwareMap.get(DcMotor.class, "front_Right_drive");
        backLeft  = hardwareMap.get(DcMotor.class, "back_Left_drive");
        backRight = hardwareMap.get(DcMotor.class, "back_Right_drive");

        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.REVERSE); // Reverse one motor for tank drive
        backLeft.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.REVERSE); // Reverse one motor for tank drive


        // Initialize AprilTag Processor with enhanced visualization
        aprilTag = new AprilTagProcessor.Builder();
                setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11);
                setTagLibrary(AprilTagGameDatabase.getCurrentGameTagLibrary());
                setDrawTagID(true);
                setDrawTagOutline(true);
                setDrawCubeProjection(true);    // 3D cube projection
                setDrawAxes(true);              // 3D coordinate axes
                build();

        // Create the vision portal with camera
        VisionPortal.Builder builder = new VisionPortal.Builder();
        builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);
        builder.enableLiveView(true);  // Enable camera preview on DS
        builder.setAutoStopLiveView(true);
        builder.addProcessor(aprilTag);

        visionPortal = builder.build();
        double frontL;
        double frontR;
        double backL;
        double backR;

        // Optional: Set camera resolution for better detection
        visionPortal.setProcessorEnabled(aprilTag, true);

        telemetry.addData("->", "SWATISNIPER3000 READY");
        telemetry.addData("->", "Press Play to begin scanning");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Check all detected tags
            if (!aprilTag.getDetections().isEmpty()) {
                for (AprilTagDetection detection : aprilTag.getDetections()) {
                    if (detection.metadata != null) {
                        telemetry.addLine("\n=== SWATISNIPER3000 TARGET ACQUIRED ===");
                        telemetry.addData("TAG ID", "#%d (%s)", detection.id, detection.metadata.name);
                        telemetry.addData("RANGE", "%.1f inches", detection.ftcPose.range);
                        telemetry.addData("BEARING", "%.1f deg", detection.ftcPose.bearing);
                        telemetry.addData("YAW", "%.1f deg", detection.ftcPose.yaw);
                        telemetry.addLine("--- 3D POSE DATA ---");
                        telemetry.addData("X", "%.1f inches", detection.ftcPose.x);
                        telemetry.addData("Y", "%.1f inches", detection.ftcPose.y);
                        telemetry.addData("Z", "%.1f inches", detection.ftcPose.z);
                        telemetry.addData("ROLL", "%.1f deg", detection.ftcPose.roll);
                        telemetry.addData("PITCH", "%.1f deg", detection.ftcPose.pitch);
                        telemetry.addLine("-------------------");
                    } else {
                        telemetry.addLine("\n=== UNKNOWN TAG DETECTED ===");
                        telemetry.addData("TAG ID", "#%d", detection.id);
                        telemetry.addData("RANGE", "%.1f inches", detection.ftcPose.range);
                        telemetry.addData("BEARING", "%.1f deg", detection.ftcPose.bearing);
                    }
                }
                telemetry.addData("TAGS DETECTED", aprilTag.getDetections().size());
            } else {
                telemetry.addLine("SWATISNIPER3000: No targets in sight");
                telemetry.addData("STATUS", "Scanning...");
            }

            // Vision portal status
            telemetry.addLine("\n=== SYSTEM STATUS ===");
            telemetry.addData("Camera", visionPortal.getCameraState());
            telemetry.addData("Vision", "AprilTag Processor");
            telemetry.addData("FPS", "%.1f", visionPortal.getFps());

            telemetry.update();
            //sleep(20);
            if(detection.metadata != null && detection.ftcPose.z<50){
                frontL=frontL+((detection.ftcPose.z-20)/100);
                frontR=frontR+((detection.ftcPose.z-20)/100);
                backL=backL+((detection.ftcPose.z-20)/100);
                backR=backR+((detection.ftcPose.z-20)/100);
                frontR=frontR+(detection.ftcPose.bearing/34);
                backR=backR+(detection.ftcPose.bearing/34);
                backL=backL+(detection.ftcPose.bearing/34);
                frontL=frontL+(detection.ftcPose.bearing/34);
            }
        }


        // Clean up
        visionPortal.close();
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }
}