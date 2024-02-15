/* Copyright (c) 2023 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.autonomus;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.ArrayList;
import java.util.List;

/*
 * This OpMode illustrates the basics of AprilTag recognition and pose estimation, using
 * the easy way.
 *
 * For an introduction to AprilTags, see the FTC-DOCS link below:
 * https://ftc-docs.firstinspires.org/en/latest/apriltag/vision_portal/apriltag_intro/apriltag-intro.html
 *
 * In this sample, any visible tag ID will be detected and displayed, but only tags that are included in the default
 * "TagLibrary" will have their position and orientation information displayed.  This default TagLibrary contains
 * the current Season's AprilTags and a small set of "test Tags" in the high number range.
 *
 * When an AprilTag in the TagLibrary is detected, the SDK provides location and orientation of the tag, relative to the camera.
 * This information is provided in the "ftcPose" member of the returned "detection", and is explained in the ftc-docs page linked below.
 * https://ftc-docs.firstinspires.org/apriltag-detection-values
 *
 * To experiment with using AprilTags to navigate, try out these two driving samples:
 * RobotAutoDriveToAprilTagOmni and RobotAutoDriveToAprilTagTank
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list.
 */
@Autonomous(name = "Red Close", group = "A")
//@Disabled
public class RedCloseRR extends LinearOpMode {

    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera
    private static final String TFOD_MODEL_FILE = "/sdcard/FIRST/tflitemodels/RedPropp.tflite";
    private static final String[] LABELS = {
            "Pixel",
    };
    /**
     * The variable to store our instance of the AprilTag processor.
     */
    private AprilTagProcessor aprilTag;

    private TfodProcessor tfod;

    /**
     * The variable to store our instance of the vision portal.
     */
    private VisionPortal visionPortal;



    int randomization = 2;

    Servo lever;

    @Override
    public void runOpMode() {

        initAprilTag();
        lever = hardwareMap.get(Servo.class, "lever");


        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d startPosition = new Pose2d(13, -63, Math.toRadians(90) );
        drive.setPoseEstimate(startPosition);
        TrajectorySequence left =     drive.trajectorySequenceBuilder(new Pose2d(13, -63, Math.toRadians(90) ) )
                .splineTo(new Vector2d(6, -38), Math.toRadians(135))
                .back(12)
                .turn(Math.toRadians(-135))
                .forward(30)
                .strafeLeft(6)
                //.lineTo(new Vector2d( 57, -34))
                .build();

        TrajectorySequence center =    drive.trajectorySequenceBuilder(new Pose2d(13, -63, Math.toRadians(90) ) )
                .forward(32)
                .back(2)
                .lineTo(new Vector2d(37, -35))
                .turn(Math.toRadians(-90))
                //.lineTo(new Vector2d( 57, -38))
                .build();

        TrajectorySequence right =     drive.trajectorySequenceBuilder(new Pose2d(13, -63, Math.toRadians(90) ) )
                .splineTo(new Vector2d(19, -36), Math.toRadians(45))
                .back(10)
                .turn(Math.toRadians(-45))
                .forward(32)
                .strafeLeft(6)
                //.lineTo(new Vector2d( 57, -46))
                .build();

        //telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        //telemetry.addData(">", "Touch Play to start OpMode");
        telemetry.update();

        while(opModeInInit()){
            //telemetry.addData("pos", getLocation());
            //telemetry.update();
            telemetryTfod();

        }

        waitForStart();

        if (opModeIsActive()) {
            while (opModeIsActive()) {

                switch (randomization) {
                    case 0:
                        drive.followTrajectorySequence(left);
                        break;
                    case 1:
                        drive.followTrajectorySequence(center);
                        break;
                    case 2:
                        drive.followTrajectorySequence(right);
                        break;
                    default:
                        break;
                }

                Pose2d location = getLocation();
                drive.setPoseEstimate(location);
                drive.followTrajectory(drive.trajectoryBuilder(location)
                        .lineToLinearHeading(getDropoff(false,randomization,false))
                        .build()
                );
                lever.setPosition(0.7);
                slowServo(lever, 0.3);
                sleep(30000);
                //telemetry.addData("location", getLocation());
                //telemetry.update();
            }
        }

        // Save more CPU resources when camera is no longer needed.
        visionPortal.close();

    }   // end method runOpMode()

    /**
     * Initialize the AprilTag processor.
     */
    private void initAprilTag() {

        // Create the AprilTag processor the easy way.
        aprilTag = AprilTagProcessor.easyCreateWithDefaults();
        tfod = new TfodProcessor.Builder().setModelFileName(TFOD_MODEL_FILE).build();

        // Create the vision portal the easy way.

        visionPortal = VisionPortal.easyCreateWithDefaults(
                hardwareMap.get(WebcamName.class, "Webcam 1"), aprilTag);

    }   // end method initAprilTag()
    private void telemetryTfod() {

        List<Recognition> currentRecognitions = tfod.getRecognitions();
        telemetry.addData("# Objects Detected", currentRecognitions.size());

        // Step through the list of recognitions and display info for each one.
        for (Recognition recognition : currentRecognitions) {
            double x = (recognition.getLeft() + recognition.getRight()) / 2 ;
            double y = (recognition.getTop()  + recognition.getBottom()) / 2 ;


            randomization = posFind(x);

            telemetry.addData(""," ");
            telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
            telemetry.addData("- Position", "%.0f / %.0f", x, y);
            telemetry.addData("- Size", "%.0f x %.0f", recognition.getWidth(), recognition.getHeight());
            telemetry.addData("randomization",randomization);
            telemetry.update();


        }   // end for() loop

    }
    private int posFind(double x){
        // int pos = 0;

        if(x <= 120){//TODO: make correct
            return 0;
        }
        else if (x <= 640){
            return 1;
        }

        return 2;
    }
    private Pose2d getLocation() {
        Pose2d location = new Pose2d(0,0,0);
        // run at 30 fps, or 33.3hz
        // this should be the max allowed by the decimation
        for(int i = 0; i < 33; i ++){
            List<AprilTagDetection> currentTagDetections = aprilTag.getDetections();
            telemetry.addData("AprilTags", currentTagDetections.size());

            //ArrayList<Pose2d> locations = new ArrayList<>();

            // Step through the list of detections and display info for each one.
            if (currentTagDetections.size() > 0) {
                AprilTagDetection detection = currentTagDetections.get(0);
                if (detection.metadata != null) {
                    telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                    telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                    telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                    telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));

                    Pose2d tag = DriveConstants.AprilTags.tags[detection.id];

                    location = new Pose2d(
                            tag.getX() - detection.ftcPose.y-DriveConstants.camera.x,
                            tag.getY() + detection.ftcPose.x-DriveConstants.camera.y,
                            Math.toRadians(tag.getHeading() - detection.ftcPose.yaw - DriveConstants.camera.yaw));

                    //telemetry.addData("error", detection.ftcPose.yaw * 3.14159 / 180 - location.getHeading());
                    return location;
                } else {
                    telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                    telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
                }
            }


            // Add "key" information to telemetry
            telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
            telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
            telemetry.addLine("RBE = Range, Bearing & Elevation");

            sleep(1000 / 30);

        }

        return location;

    }
    private  Pose2d getDropoff(boolean blue, int randomization, boolean left){
        double y;
        if(blue){
            y =   72 - ( 31 + 6 * randomization + (left?0:3) );
        }
        else {
            y = -72 + (24.5 + 6 * (2 - randomization) + (left ? 3 : 0));
        }

        return new Pose2d(48.5, y, 0);
    }
    private void slowServo(Servo servo, double end){
        int count = 50;
        double speed = 0.1; //units are servo range degrees / second
        double start = servo.getPosition();

        for(double i = 0; i < 1; i += 1 / count){
            servo.setPosition(start + (end - start) *  i);
            sleep((long) ((speed / count ) / Math.abs(start - end) * 1000));
        }
    }


}   // end class
