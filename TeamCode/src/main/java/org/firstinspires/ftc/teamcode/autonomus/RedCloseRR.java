package org.firstinspires.ftc.teamcode.autonomus;

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

import org.firstinspires.ftc.teamcode.util.camera;
import org.firstinspires.ftc.teamcode.util.AprilTags;

import java.util.ArrayList;
import java.util.List;

@Autonomous(name="RedCloseRR", group="Robot")

public class RedCloseRR extends LinearOpMode {

    private static final boolean USE_WEBCAM = true;
    private static final String TFOD_MODEL_FILE = "/sdcard/FIRST/tflitemodels/RedPropp.tflite";
    private static final String[] LABELS = {
            "Pixel",
    };

    int randomization = 0;


    @Override
    public void runOpMode() {

        // Wait for the game to start (driver presses PLAY)
        initTfod();
        telemetryTfod();

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        TrajectorySequence left = drive.trajectorySequenceBuilder(new Pose2d(-35, -63, Math.toRadians(90) ) )
                .splineTo(new Vector2d(-41, -36), Math.toRadians(135))
                .strafeRight(10)
                .turn(Math.toRadians(-45))
                .forward(17)
                .turn(Math.toRadians(-90))
                .lineTo(new Vector2d(48, -12))
                .strafeRight(24)
                .build();

        TrajectorySequence center = drive.trajectorySequenceBuilder(new Pose2d(-35, -63, Math.toRadians(90) ) )
                .forward(28)
                .strafeLeft(18)
                .forward(23)
                .turn(Math.toRadians(-90))
                .lineTo(new Vector2d(48, -12))
                .strafeRight(24)
                .build();

        TrajectorySequence right = drive.trajectorySequenceBuilder(new Pose2d(-35, -63, Math.toRadians(90) ) )
                .splineTo(new Vector2d(-29, -36), Math.toRadians(45))
                .strafeLeft(10)
                .turn(Math.toRadians(45))
                .forward(17)
                .turn(Math.toRadians(-90))
                .lineTo(new Vector2d(48, -12))
                .strafeRight(24)
                .build();

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        while(opModeInInit()){
            telemetryTfod();
        }


        waitForStart();
        //runtime.reset();

        boolean triggerDown = false;
        double doorPosition = DOWN;


        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            if(randomization == 0) {
                drive.followTrajectorySequence(left);

              Pose2d location = getLocation();
                telemetry.addData("computed location: %2.1f, %2.1f, heading %2.1f" , location.getX(), location.getY(), location.getHeading());
                telemetry.update();

//                drive.followTrajectorySequence( drive.trajectorySequenceBuilder(getLocation())
//                        .lineTo(new Vector2d( /* TODO: add x y of robot when ready to place yellow pixel */))
//                        .addDisplacementMarker( () -> {
//                            /* TODO: add code to place yellow pixel */
//                        })
//                        .build());

            }
            else if(randomization == 1) {
                drive.followTrajectorySequence(center);
//                drive.followTrajectorySequence( drive.trajectorySequenceBuilder(getLocation())
//                        .lineTo(new Vector2d( /* TODO: add x y of robot when ready to place yellow pixel */))
//                        .addDisplacementMarker( () -> {
//                            /* TODO: add code to place yellow pixel */
//                        })
//                        .build());
            }
            else if(randomization == 2) {
                drive.followTrajectorySequence(right);
//                drive.followTrajectorySequence( drive.trajectorySequenceBuilder(getLocation())
//                        .lineTo(new Vector2d( /* TODO: add x y of robot when ready to place yellow pixel */))
//                        .addDisplacementMarker( () -> {
//                            /* TODO: add code to place yellow pixel */
//                        })
//                        .build());
            }
        }

        visionPortal.close();
    }


    private void initTfod() {

        // Create the TensorFlow processor by using a builder.
        tfod = new TfodProcessor.Builder().setModelFileName(TFOD_MODEL_FILE).build();

        VisionPortal.Builder builder = new VisionPortal.Builder();
        if (USE_WEBCAM) {
            builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        } else {
            builder.setCamera(BuiltinCameraDirection.BACK);
        }
        builder.addProcessor(tfod);
        visionPortal = builder.build();
        tfod.setMinResultConfidence(0.7f);

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
            telemetry.addData("",randomization);
            telemetry.update();


        }   // end for() loop

    }
    private int posFind(double x){
        // int pos = 0;

        if(x >= 490){
            return 2;
        }
        else if (x >= 1){
            return 1;
        }

        return 0;
    }

    private Pose2d getLocation() {

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());

        ArrayList<Pose2d> locations = new ArrayList<>();
        Pose2d location = new Pose2d();

        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));

                Pose2d tag = AprilTags.tags[detection.id];
                locations.add( new Pose2d(tag.getX() - detection.ftcpose.x + camera.x,
                                          tag.getY() - detection.ftcPose.y + camera.y,
                                     tag.getHeading() - detection.ftcPose.yaw + camera.yaw) );
            } else {
                telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
            }
        }   // end for() loop

        for(int i = 0; i < locations.size(); i ++){
            location = new Pose2d(location.getX() + locations.get(i).getX(),
                                  location.getY() + locations.get(i).getY(),
                                  location.getHeading() + locations.get(i).getHeading());
        }

        location = new Pose2d(location.getX() / locations.size(),
                              location.getY() / locations.size(),
                              location.getHeading() / locations.size())

        // Add "key" information to telemetry
        telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
        telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
        telemetry.addLine("RBE = Range, Bearing & Elevation");

        return location;

    }
}
