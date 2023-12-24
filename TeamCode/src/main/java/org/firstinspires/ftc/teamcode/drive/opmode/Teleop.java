/* Copyright (c) 2021 FIRST. All rights reserved.
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

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

/*
 * This file contains an example of a Linear "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When a selection is made from the menu, the corresponding OpMode is executed.
 *
 * This particular OpMode illustrates driving a 4-motor Omni-Directional (or Holonomic) robot.
 * This code will work with either a Mecanum-Drive or an X-Drive train.
 * Both of these drives are illustrated at https://gm0.org/en/latest/docs/robot-design/drivetrains/holonomic.html
 * Note that a Mecanum drive must display an X roller-pattern when viewed from above.
 *
 * Also note that it is critical to set the correct rotation direction for each motor.  See details below.
 *
 * Holonomic drives provide the ability for the robot to move in three axes (directions) simultaneously.
 * Each motion axis is controlled by one Joystick axis.
 *
 * 1) drive:    Driving forward and backward               Left-joystick Forward/Backward
 * 2) strafe:  Strafing right and left                     Left-joystick Right and Left
 * 3) turn:      Rotating Clockwise and counter clockwise    Right-joystick Right and Left
 *
 * This code is written assuming that the right-side motors need to be reversed for the robot to drive forward.
 * When you first test your robot, if it moves backward when you push the left stick forward, then you must flip
 * the direction of all 4 motors (see code below).
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@TeleOp(name="Teleop", group="Linear OpMode")

public class Teleop extends LinearOpMode {

    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime      = new ElapsedTime();
    private DcMotor leftFrontDrive   = null;
    private DcMotor leftBackDrive    = null;
    private DcMotor rightFrontDrive  = null;
    private DcMotor rightBackDrive   = null;

    private Servo door1Left          = null;
    private Servo door1Right         = null;
    private Servo door2Left          = null;
    private Servo door2Right         = null;
    private Servo launcher           = null;
    private Servo lever              = null;
    
    private static final double UP   = 0.3;
    private static final double DOWN = 0;
    
    
    private boolean launch            = false;
    
    

    @Override
    public void runOpMode() {

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "motor0");
        leftBackDrive   = hardwareMap.get(DcMotor.class, "motor1");
        rightBackDrive = hardwareMap.get(DcMotor.class, "motor2");
        rightFrontDrive  = hardwareMap.get(DcMotor.class, "motor3");
        
        door1Left       = hardwareMap.get(Servo.class, "servo5");
        door1Right      = hardwareMap.get(Servo.class, "servo4");
        door2Left       = hardwareMap.get(Servo.class, "servo2");
        door2Right      = hardwareMap.get(Servo.class, "servo3");
        launcher        = hardwareMap.get(Servo.class, "servo1");
        lever           = hardwareMap.get(Servo.class, "servo0");

        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
        
        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();
        
        boolean trigger1Down = false;
        boolean trigger2Down = false;
        double door1Position = DOWN;
        double door2Position = DOWN;
        
        
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            double max;

            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            double  drive    =   gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
            double  strafe   =   -gamepad1.left_stick_x;
            double  turn     =   gamepad1.right_stick_x;
            boolean slow     =   gamepad1.right_bumper;
            
            boolean door2Set =   gamepad1.left_bumper;
            boolean door1Set =   gamepad1.right_bumper;
            //boolean doorUp   =   gamepad1.y;
            //boolean doorDown =   gamepad1.x;
            boolean launcherTrigger = gamepad1.y;

            //arm winch
            if(door1Set && !trigger1Down){
                //trigger1Down  = true;
                door1Position = (door1Position == UP?DOWN:UP);
            }
            trigger1Down = door1Set;
            
            if(door2Set && !trigger2Down){
                //trigger1Down  = true;
                door2Position = (door2Position == UP?DOWN:UP);
            }
            trigger2Down = door2Set;
            
            if(launch != launcherTrigger){
                launch = ! launch;
                launcherTrigger = ! launcherTrigger;
            }
            
            
            /*if(doorDown && !doorUp){
                door1Position -= 0.03;
            }
            
            if(!doorDown && doorUp){
                door1Position += 0.03;
            }*/
            

            // Combine the joystick requests for each axis-motion to determine each wheel's power.
            // Set up a variable for each drive wheel to save the power level for telemetry.
            double leftFrontPower  = drive + strafe - turn;
            double rightFrontPower = drive - strafe - turn;
            double leftBackPower   = drive - strafe + turn;
            double rightBackPower  = drive + strafe + turn;

            // Normalize the values so no wheel power exceeds 100%
            // This ensures that the robot maintains the desired motion.
            max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));
            
            //Another way
            //max = Math.max(Math.max(Math.abs(leftBackPower), Math.abs(rightBackPower)), Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower)));
            
            if (max > (1.0) ) {
                leftFrontPower  /= max;
                rightFrontPower /= max;
                leftBackPower   /= max;
                rightBackPower  /= max;
            }
            
            if(door1Position > UP){
                door1Position = UP;
            }
            
            if(door1Position < DOWN){
                door1Position = DOWN;
            }
            
            
            if(door2Position > UP){
                door2Position = UP;
            }
            
            if(door2Position < DOWN){
                door2Position = DOWN;
            }

            // Send calculated power to wheels
            leftFrontDrive.setPower(leftFrontPower);
            rightFrontDrive.setPower(rightFrontPower);
            leftBackDrive.setPower(leftBackPower);
            rightBackDrive.setPower(rightBackPower);
            
            door1Right.setPosition(door1Position);
            door1Left.setPosition(- door1Position + UP);
            door2Right.setPosition(door2Position);
            door2Left.setPosition(- door2Position + UP);
            launcher.setPosition(launch?0:0.5);
            lever.setPosition(0.9);
            

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
            telemetry.addData("left bumper", "%1b", door1Set);
            telemetry.update();
        }
    }}
