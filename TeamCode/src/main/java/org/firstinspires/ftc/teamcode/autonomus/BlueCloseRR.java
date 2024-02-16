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
import com.qualcomm.robotcore.hardware.DcMotor;
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
@Autonomous(name = "Blue Close", group = "A")
//@Disabled
public class BlueCloseRR extends LinearOpMode {

    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera
    private static final String TFOD_MODEL_FILE = "/sdcard/FIRST/tflitemodels/BluePropp.tflite";
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



    int randomization = 0;

    static int rand = 0;

    DcMotor leftHang;
    DcMotor rightHang;
    Servo lever;

    @Override
    public void runOpMode() {

        initAprilTag();
        lever = hardwareMap.get(Servo.class, "lever");

        rightHang  = hardwareMap.get(DcMotor.class, "rightHang");
        leftHang  = hardwareMap.get(DcMotor.class, "leftHang");

        leftHang.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightHang.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);



        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d startPosition = new Pose2d(13, -63, Math.toRadians(90) );
        drive.setPoseEstimate(startPosition);
        TrajectorySequence right =     drive.trajectorySequenceBuilder(new Pose2d(13, -63, Math.toRadians(90) ) )
                .splineTo(new Vector2d(20, -36), Math.toRadians(45))
                .back(10)
                .turn(Math.toRadians(135))
                .forward(32)
                .strafeRight(8)
                //.lineTo(new Vector2d( 57, -46))
                .build();

        TrajectorySequence center =    drive.trajectorySequenceBuilder(new Pose2d(13, -63, Math.toRadians(90) ) )
                .forward(30)
                .back(2)
                .strafeLeft(30)
                .turn(Math.toRadians(90))
                //.lineTo(new Vector2d( 57, -38))
                //.forward(6)
                .build();

        TrajectorySequence left =     drive.trajectorySequenceBuilder(new Pose2d(13, -63, Math.toRadians(90) ) )
                .splineTo(new Vector2d(6, -38), Math.toRadians(135))
                .back(12)
                .turn(Math.toRadians(45))
                .forward(30)
                .strafeRight(8)
                //.lineTo(new Vector2d( 57, -34))
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
                }

                Pose2d location = getLocation();
                /*if(!location.equals(new Pose2d(0,0,0))) {
                    drive.setPoseEstimate(location);

                    telemetry.addLine("found");
                }
                else{*/
                    location = drive.getPoseEstimate();
                    telemetry.addLine("not found");
                //}
                telemetry.addData("getLocation",location);
                telemetry.update();

                drive.followTrajectory(drive.trajectoryBuilder(location)
                        .lineToLinearHeading(getDropoff(true,randomization,false))
                        .build()
                );

                lever.setPosition(1);
                slowServo(lever, 0.5);
                drive.followTrajectorySequence(drive.trajectorySequenceBuilder(getDropoff(true,randomization,false))
                        .back(12)
                        .strafeLeft(24)
                        .forward(20)
                        .build()
                );

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

        tfod.setMinResultConfidence(0.5f);
        // Create the vision portal the easy way.

        visionPortal = VisionPortal.easyCreateWithDefaults(
                hardwareMap.get(WebcamName.class, "Webcam 1"), aprilTag, tfod);



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

        if(x <= 230){//TODO: make correct
            return 1;
        }
        else if (x <= 640){
            return 2;
        }

        return 0;
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
            //telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
            //telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
            //telemetry.addLine("RBE = Range, Bearing & Elevation");

            sleep(1000 / 30);

        }

        return location;

    }
    private  Pose2d getDropoff(boolean isBlue, int randomization, boolean isLeft){
        double y;
        if(isBlue){
            y =   -72 + ( 39 + 6 * randomization + (isLeft?0:3) );
        }
        else {
            y = -72 + (24.5 + 6 * (2- randomization) + (isLeft ? 3 : 0));
        }
        telemetry.addData("y", y);
        telemetry.addData("r", randomization);
        telemetry.addData("B", isBlue);
        telemetry.addData("L", isLeft);
        telemetry.update();
        return new Pose2d(54, y, 0);
    }
    private void slowServo(Servo servo, double end){
        int count = 100;
        double seconds = 5;
        double start = servo.getPosition();

        for(double i = 0; i <= 1; i += 1.0 / count){
            servo.setPosition(start + (end - start) *  i);
            sleep((long) seconds * 1000 / count);

            telemetry.addData("", i);
            telemetry.update();
        }
    }


}   // end class
