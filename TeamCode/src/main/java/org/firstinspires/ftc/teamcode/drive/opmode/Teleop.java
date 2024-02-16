package org.firstinspires.ftc.teamcode.drive.opmode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name="Teleop", group="Linear OpMode")

public class Teleop extends LinearOpMode {

    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime      = new ElapsedTime();
  
    private DcMotor leftFrontDrive  = null;
    private DcMotor leftBackDrive   = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive  = null;
    private DcMotor rightHang  = null;
    private DcMotor leftHang  = null;

    private Servo flipDoor          = null;
    private Servo linkDoor          = null;
    private CRServo belt            = null;
    private Servo launcher          = null;
    private Servo lever             = null;

    private IMU imu;
  
    private static final double UP   = 0.3;
    private static final double DOWN = 0;

    private static final double  pr = 0.1;
  
    private static final int RobotCentric = 0;
    private static final int FeildCentric = 1;
    
    private boolean launch           = false;
    
    private int DriveMode = FeildCentric;

    double leftFrontPower;
    double rightFrontPower;
    double rightBackPower;
    double leftBackPower;
    @Override
    public void runOpMode() {

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "motor0");
        leftBackDrive   = hardwareMap.get(DcMotor.class, "motor3");
        rightBackDrive = hardwareMap.get(DcMotor.class, "motor2");
        rightFrontDrive  = hardwareMap.get(DcMotor.class, "motor1");

        rightHang  = hardwareMap.get(DcMotor.class, "rightHang");
        leftHang  = hardwareMap.get(DcMotor.class, "leftHang");

        flipDoor   = hardwareMap.get(Servo.class, "flip");
        linkDoor  = hardwareMap.get(Servo.class, "link");
        belt       = hardwareMap.get(CRServo.class, "belt");
        launcher   = hardwareMap.get(Servo.class, "launcher");
        lever      = hardwareMap.get(Servo.class, "lever");

        imu = hardwareMap.get(IMU.class, "imu");

      IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
        RevHubOrientationOnRobot.LogoFacingDirection.UP,
        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
      // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
      imu.initialize(parameters);
      
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        leftHang.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightHang.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();
        
        boolean linkTrigger = false;
        boolean flipTrigger = false;
        double linkPosition = DOWN;
        double flipPosition = UP; // up and down are flipped
        
        
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
          double max;
          
          boolean flipSet         = gamepad1.right_bumper;
          boolean linkSet         = gamepad1.left_bumper;
          boolean launcherTrigger = gamepad2.options;
          boolean runBelt         = gamepad2.a;


          boolean linkUp          = gamepad1.left_bumper;
          boolean linkDown        = gamepad1.left_trigger > 0.7;

          boolean flipUp          = gamepad1.right_bumper;
          boolean flipDown        = gamepad1.right_trigger > 0.7;

          double leftHangSpeed = gamepad2.left_trigger + (gamepad2.left_bumper?-2:0);
          double rightHangSpeed = gamepad2.right_trigger + (gamepad2.right_bumper?-2:0);

          
          if (gamepad1.options) {
              imu.resetYaw();
          }

          if(DriveMode == RobotCentric){
            double  drive    =   gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
            double  strafe   =   gamepad1.left_stick_x;
            double  turn     = - gamepad1.right_stick_x;
            //boolean slow     =   gamepad1.right_bumper;
            
            leftFrontPower  = drive + strafe - turn;
            rightFrontPower = drive - strafe + turn;
            leftBackPower   = drive - strafe - turn;
            rightBackPower  = drive + strafe + turn;

            max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));

            if (max > (1.0) ) {
                leftFrontPower  /= max;
                rightFrontPower /= max;
                leftBackPower   /= max;
                rightBackPower  /= max;
            }
          }
          else{
            double x = - gamepad1.left_stick_x;
            double y = gamepad1.left_stick_y;
            double rx = gamepad1.right_stick_x;
              
            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            // Rotate the movement direction counter to the bot's rotation
            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);
  //
            rotX = rotX * 1;  // Counteract imperfect strafing

                // Denominator is the largest motor power (absolute value) or 1
                // This ensures all the powers maintain the same ratio,
                // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            leftFrontPower = (rotY + rotX + rx) / denominator;
            leftBackPower = (rotY - rotX + rx) / denominator;
            rightFrontPower = (rotY - rotX - rx) / denominator;
            rightBackPower = (rotY + rotX - rx) / denominator;
              
          }

          leftFrontDrive.setPower(leftFrontPower);
          rightFrontDrive.setPower(rightFrontPower);
          leftBackDrive.setPower(leftBackPower);
          rightBackDrive.setPower(rightBackPower);


          rightHang.setPower(leftHangSpeed);
          leftHang.setPower(rightHangSpeed);

          if(launcherTrigger) {
              launcher.setPosition(0.8);
          }
          else{
              launcher.setPosition(0);
          }


          belt.setPower(runBelt?1:0);
          lever.setPosition(0.4); // 0.1 = 10 degrees ish

            if(linkUp){
                linkDoor.setPosition(UP);
            }
            if(linkDown){
                linkDoor.setPosition(DOWN);
            }

            if(flipUp){
                flipDoor.setPosition(DOWN);
            }
            if(flipDown){
                flipDoor.setPosition(UP);
            }
          

          // Show the elapsed game time and wheel power.
          telemetry.addData("Status", "Run Time: " + runtime.toString());
          telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
          telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
          telemetry.addData("left bumper", "%1b", linkSet);
          telemetry.update();


        }
    }
}
