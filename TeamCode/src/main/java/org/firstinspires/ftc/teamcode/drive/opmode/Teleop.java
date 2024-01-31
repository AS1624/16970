package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Teleop", group="Linear OpMode")

public class Teleop extends LinearOpMode {

    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime      = new ElapsedTime();
  
    private DcMotor leftFrontDrive   = null;
    private DcMotor leftBackDrive    = null;
    private DcMotor rightFrontDrive  = null;
    private DcMotor rightBackDrive   = null;

    private Servo flipDoor           = null;
    private Servo slideDoor          = null;
    private CRServo belt             = null;
    private Servo launcher           = null;
    private Servo lever              = null;
    
    private static final double UP   = 0.3;
    private static final double DOWN = 0;
  
    private static final int RobotCentric = 0;
    private static final int FeildCentric = 1;
    
    private boolean launch           = false;
    
    private int DriveMode = RobotCentric;

    @Override
    public void runOpMode() {

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "motor0");
        leftBackDrive   = hardwareMap.get(DcMotor.class, "motor3");
        rightBackDrive = hardwareMap.get(DcMotor.class, "motor2");
        rightFrontDrive  = hardwareMap.get(DcMotor.class, "motor1");
        
        flipDoor   = hardwareMap.get(Servo.class, "servo5");
        slideDoor  = hardwareMap.get(Servo.class, "servo4");
        belt       = hardwareMap.get(CRServo.class, "servo2");
        door2Right = hardwareMap.get(Servo.class, "servo3");
        launcher   = hardwareMap.get(Servo.class, "servo1");
        lever      = hardwareMap.get(Servo.class, "servo0");

        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
        
        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();
        
        boolean linkTrigger = false;
        boolean flipTrigger = false;
        double linkPosition = DOWN;
        double flipPosition = DOWN;
        
        
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            double max;
            
            boolean flipSet         = gamepad1.left_bumper;
            boolean linkSet         = gamepad1.right_bumper;
            boolean launcherTrigger = gamepad1.y;
            boolean runBelt         = gamepad2.a;

            //arm winch
            if(linkSet && !linkTrigger){
                //trigger1Down  = true;
                linkPosition = (linkPosition == UP?DOWN:UP);
            }
            linkTrigger = linkSet;
            
            if(door2Set && !flipTrigger){
                //trigger1Down  = true;
                flipPosition = (flipPosition == UP?DOWN:UP);
            }
            flipTrigger = flipSet;
            
            if(launch != launcherTrigger){
                launch = ! launch;
                launcherTrigger = ! launcherTrigger;
            }
            

            if(DriveMode == RobotCentric){
              double  drive    = - gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
              double  strafe   =   gamepad1.left_stick_x;
              double  turn     =   gamepad1.right_stick_x;
              boolean slow     =   gamepad1.right_bumper;
              
              double leftFrontPower  = drive + strafe - turn;
              double rightFrontPower = drive - strafe + turn;
              double leftBackPower   = drive - strafe - turn;
              double rightBackPower  = drive + strafe + turn;

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
              double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

              // Rotate the movement direction counter to the bot's rotation
              double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
              double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

              rotX = rotX * 1;  // Counteract imperfect strafing

              // Denominator is the largest motor power (absolute value) or 1
              // This ensures all the powers maintain the same ratio,
              // but only if at least one is out of the range [-1, 1]
              double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
              double leftFrontPower = (rotY + rotX + rx) / denominator;
              double leftBackPower = (rotY - rotX + rx) / denominator;
              double rightFrontPower = (rotY - rotX - rx) / denominator;
              double rightBackPower = (rotY + rotX - rx) / denominator;
            }
          
            leftFrontDrive.setPower(leftFrontPower);
            rightFrontDrive.setPower(rightFrontPower);
            leftBackDrive.setPower(leftBackPower);
            rightBackDrive.setPower(rightBackPower);
            
            slideDoor.setPosition(slidePosition);
            belt.setPower(runBelt?1:0);
            launcher.setPosition(launch?0:0.5);
            lever.setPosition(0.9);
            

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
            telemetry.addData("left bumper", "%1b", linkSet);
            telemetry.update();
        }
    }}
