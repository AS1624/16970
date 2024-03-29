package org.firstinspires.ftc.teamcode.autonomus;

import android.media.MediaRecorder;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import org.firstinspires.ftc.teamcode.drive.DriveConstants.camera;
import org.firstinspires.ftc.teamcode.drive.DriveConstants.AprilTags;

import java.util.ArrayList;
import java.util.List;

@Autonomous(name="RedFarRR", group="Robot")

public class RedAgro extends LinearOpMode {

    private static final boolean USE_WEBCAM = true;
    private static final String TFOD_MODEL_FILE = "/sdcard/FIRST/tflitemodels/RedPropp.tflite";
    private static final String[] LABELS = {
            "Pixel",
    };
    TfodProcessor tfod;
    VisionPortal visionPortal;

    int randomization = 2;
    AprilTagProcessor  aprilTag;

    Servo lever;

    @Override
    public void runOpMode() {

        // Wait for the game to start (driver presses PLAY)
        initVision();
        telemetryTfod();

        lever = hardwareMap.get(Servo.class, "lever");
        aprilTag = AprilTagProcessor.easyCreateWithDefaults();

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d startPosition = new Pose2d(13, -63, Math.toRadians(90) );
        drive.setPoseEstimate(startPosition);



        TrajectorySequence left = drive.trajectorySequenceBuilder(new Pose2d(-35, -63, Math.toRadians(90) ) )
                .splineTo(new Vector2d(-42, -36), Math.toRadians(135))
                .back(10)
                .turn(Math.toRadians(-45))
                .forward(32)
                .turn(Math.toRadians(-90))
                .lineTo(new Vector2d(48, -11))
                .strafeRight(18)
                .lineTo(new Vector2d( 57, -34))
                .addDisplacementMarker( () -> {
                    lever.setPosition(0.68);
                    sleep(1000);
                })
                .back(4)
                .addDisplacementMarker( () -> {
                    lever.setPosition(0.6);
                    sleep(1000);
                })
                .build();


        TrajectorySequence center = drive.trajectorySequenceBuilder(new Pose2d(-35, -63, Math.toRadians(90) ) )
                .forward(32)
                .back(2)
                .strafeLeft(18)
                .forward(22)
                .turn(Math.toRadians(-90))
                .lineTo(new Vector2d(48, -11))
                .strafeRight(24)
                .lineTo(new Vector2d( 57, -38))
                .addDisplacementMarker( () -> {
                    lever.setPosition(0.68);
                    sleep(1000);
                })
                .back(4)
                .addDisplacementMarker( () -> {
                    lever.setPosition(0.6);
                    sleep(1000);
                })
                .build();


        TrajectorySequence right = drive.trajectorySequenceBuilder(new Pose2d(-35, -63, Math.toRadians(90)))
                .splineTo(new Vector2d(-29, -36), Math.toRadians(45))
                .back(10)
                .turn(Math.toRadians(45))
                .forward(31)
                .turn(Math.toRadians(-90))
                .forward(84)
                .strafeRight(24)
                .lineTo(new Vector2d(57, -38))
                .addDisplacementMarker(() -> {
                    lever.setPosition(0.68);
                    sleep(1000);
                })
                .back(4)
                .addDisplacementMarker(() -> {
                    lever.setPosition(0.6);
                    sleep(1000);
                })
                .build();

        telemetry.addData("Status", "Initialized");
        while(opModeInInit()){
            //telemetry.addData("pos", getLocation());
            telemetry.update();
            telemetryTfod();

        }


        waitForStart();
        //runtime.reset();

        boolean triggerDown = false;
        //double doorPosition = DOWN;


        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            if(randomization == 0) {
                drive.followTrajectorySequence(left);
            }
            else if(randomization == 1) {
                drive.followTrajectorySequence(center);
            }
            else if(randomization == 2) {
                drive.followTrajectorySequence(right);
            }
            sleep(30000);
        }

        //visionPortal.close();
    }


    private void initVision() {
        // Create the TensorFlow processor by using a builder.
        tfod = new TfodProcessor.Builder().setModelFileName(TFOD_MODEL_FILE).build();

        // Create the vision portal the easy way.
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessors(tfod)
                .build();

        tfod.setMinResultConfidence(0.68f);

    }
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


    /*private Pose2d getLocation() {


        List<AprilTagDetection> currentTagDetections = aprilTag.getDetections();
        telemetry.addData("AprilTags", currentTagDetections.size());

        ArrayList<Pose2d> locations = new ArrayList<>();
        Pose2d location = new Pose2d();

        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentTagDetections) {
            if (detection.metadata != null) {
                telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));

                Pose2d tag = AprilTags.tags[detection.id];
                locations.add( new Pose2d(tag.getX() - detection.ftcPose.x + camera.x,
                                          tag.getY() - detection.ftcPose.y + camera.y,
                                     tag.getHeading() - detection.ftcPose.yaw + camera.yaw) );
            } else {
                telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
            }
        }   // end for() loop

        /*for(int i = 0; i < locations.size(); i ++){
            location = new Pose2d(location.getX() + locations.get(i).getX(),
                                  location.getY() + locations.get(i).getY(),
                                  location.getHeading() + locations.get(i).getHeading());
        }

        location = new Pose2d(location.getX() / locations.size(),
                              location.getY() / locations.size(),
                              location.getHeading() / locations.size());

        // Add "key" information to telemetry
        telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
        telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
        telemetry.addLine("RBE = Range, Bearing & Elevation");

        //Pose2d location = new Pose2d(0,0,0);
        return location;

    }*/
    public  void setServoPosition(Servo servo, double position){
        double curPosition = servo.getPosition();
        double moveTo = position-curPosition;

        while(curPosition != position){
            curPosition+= moveTo/10;
            servo.setPosition(curPosition);
            sleep(100);
        }
    }
}
