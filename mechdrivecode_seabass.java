/*   MIT License
 *   Copyright (c) [2024] [Base 10 Assets, LLC]
 *
 *   Permission is hereby granted, free of charge, to any person obtaining a copy
 *   of this software and associated documentation files (the "Software"), to deal
 *   in the Software without restriction, including without limitation the rights
 *   to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 *   copies of the Software, and to permit persons to whom the Software is
 *   furnished to do so, subject to the following conditions:

 *   The above copyright notice and this permission notice shall be included in all
 *   copies or substantial portions of the Software.

 *   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *   IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *   FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *   AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *   LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 *   OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 *   SOFTWARE.
 */

 package org.firstinspires.ftc.teamcode;

 import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
 import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
 import com.qualcomm.robotcore.hardware.CRServo;
 import com.qualcomm.robotcore.hardware.DcMotor;
 import com.qualcomm.robotcore.hardware.DcMotorEx;
 import com.qualcomm.robotcore.hardware.Servo;
 
 import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
 
 /* Rolling Robots update - mech drive, arm p controller
  * This OpMode is an example driver-controlled (TeleOp) mode for the goBILDA 2024-2025 FTC
  * Into The Deep Starter Robot
  * The code is structured as a LinearOpMode
  *
  * This robot has a two-motor differential-steered (sometimes called tank or skid steer) drivetrain.
  * With a left and right drive motor.
  * The drive on this robot is controlled in an "Arcade" style, with the left stick Y axis
  * controlling the forward movement and the right stick X axis controlling rotation.
  * This allows easy transition to a standard "First Person" control of a
  * mecanum or omnidirectional chassis.
  *
  * The drive wheels are 96mm diameter traction (Rhino) or omni wheels.
  * They are driven by 2x 5203-2402-0019 312RPM Yellow Jacket Planetary Gearmotors.
  *
  * This robot's main scoring mechanism includes an arm powered by a motor, a "wrist" driven
  * by a servo, and an intake driven by a continuous rotation servo.
  *
  * The arm is powered by a 5203-2402-0051 (50.9:1 Yellow Jacket Planetary Gearmotor) with an
  * external 5:1 reduction. This creates a total ~254.47:1 reduction.
  * This OpMode uses the motor's encoder and the RunToPosition method to drive the arm to
  * specific setpoints. These are defined as a number of degrees of rotation away from the arm's
  * starting position.
  *
  * Make super sure that the arm is reset into the robot, and the wrist is folded in before
  * you run start the OpMode. The motor's encoder is "relative" and will move the number of degrees
  * you request it to based on the starting position. So if it starts too high, all the motor
  * setpoints will be wrong.
  *
  * The wrist is powered by a goBILDA Torque Servo (2000-0025-0002).
  *
  * The intake wheels are powered by a goBILDA Speed Servo (2000-0025-0003) in Continuous Rotation mode.
  */
 
 
 @TeleOp(name="mechdrivecode_seabass", group="Robot")
 //@Disabled
 public class mechdrivecode_seabass extends LinearOpMode {
 
     /* Declare OpMode members. */
     public DcMotor  leftDrive   = null; //the left drivetrain motor
     public DcMotor  rightDrive  = null; //the right drivetrain motor
     public DcMotor  leftDriveBack   = null; //the left drivetrain motor
     public DcMotor  rightDriveBack  = null; //the right drivetrain motor
     public DcMotor  armMotor    = null; //the arm motor
     public CRServo  intake      = null; //the active intake servo
     public Servo    wrist       = null; //the wrist servo
 
     public double ArmTarget = 0.0;
     /* This constant is the number of encoder ticks for each degree of rotation of the arm.
     To find this, we first need to consider the total gear reduction powering our arm.
     First, we have an external 20t:100t (5:1) reduction created by two spur gears.
     But we also have an internal gear reduction in our motor.
     The motor we use for this arm is a 117RPM Yellow Jacket. Which has an internal gear
     reduction of ~50.9:1. (more precisely it is 250047/4913:1)
     We can multiply these two ratios together to get our final reduction of ~254.47:1.
     The motor's encoder counts 28 times per rotation. So in total you should see about 7125.16
     counts per rotation of the arm. We divide that by 360 to get the counts per degree. */
     final double ARM_TICKS_PER_DEGREE =
             28 // number of encoder ticks per rotation of the bare motor
                     * 250047.0 / 4913.0 // This is the exact gear ratio of the 50.9:1 Yellow Jacket gearbox
                     * 100.0 / 20.0 // This is the external gear reduction, a 20T pinion gear that drives a 100T hub-mount gear
                     * 1/360.0; // we want ticks per degree, not per rotation
 
 
     /* These constants hold the position that the arm is commanded to run to.
     These are relative to where the arm was located when you start the OpMode. So make sure the
     arm is reset to collapsed inside the robot before you start the program.
 
     In these variables you'll see a number in degrees, multiplied by the ticks per degree of the arm.
     This results in the number of encoder ticks the arm needs to move in order to achieve the ideal
     set position of the arm. For example, the ARM_SCORE_SAMPLE_IN_LOW is set to
     160 * ARM_TICKS_PER_DEGREE. This asks the arm to move 160° from the starting position.
     If you'd like it to move further, increase that number. If you'd like it to not move
     as far from the starting position, decrease it. */
 
     final double ARM_COLLAPSED_INTO_ROBOT  = 0;
     final double ARM_COLLECT               = 230 * ARM_TICKS_PER_DEGREE;
     final double ARM_CLEAR_BARRIER         = 230 * ARM_TICKS_PER_DEGREE;
     final double ARM_SCORE_SPECIMEN        = 160 * ARM_TICKS_PER_DEGREE;
     final double ARM_SCORE_SAMPLE_IN_LOW   = 160 * ARM_TICKS_PER_DEGREE;
     final double ARM_ATTACH_HANGING_HOOK   = 120 * ARM_TICKS_PER_DEGREE;
     final double ARM_WINCH_ROBOT           = 15  * ARM_TICKS_PER_DEGREE;
 
     /* Variables to store the speed the intake servo should be set at to intake, and deposit game elements. */
     final double INTAKE_COLLECT    = -1.0;
     final double INTAKE_OFF        =  0.0;
     final double INTAKE_DEPOSIT    =  0.5;
 
     /* Variables to store the positions that the wrist should be set to when folding in, or folding out. */
     final double WRIST_FOLDED_IN   = 0.3;
     final double WRIST_FOLDED_OUT  = 0.75;
 
     /* A number in degrees that the triggers can adjust the arm position by */
     final double FUDGE_FACTOR = 15 * ARM_TICKS_PER_DEGREE;
 
     /* Variables that are used to set the arm to a specific position */
     double armPosition = (int)ARM_COLLAPSED_INTO_ROBOT;
     double armPositionFudgeFactor;
 
 
     @Override
     public void runOpMode() {
         /*
         These variables are private to the OpMode, and are used to control the drivetrain.
          */
         double left;
         double right;
         double forward;
         double rotate;
         double max;
         double left_y;
         double left_x;
         double right_y;
         double right_x;
 
 
         /* Define and Initialize Motors */
         leftDrive  = hardwareMap.get(DcMotor.class, "left_drive");
         rightDrive = hardwareMap.get(DcMotor.class, "right_drive");
         leftDriveBack  = hardwareMap.get(DcMotor.class, "left_drive_back");
         rightDriveBack = hardwareMap.get(DcMotor.class, "right_drive_back");
         armMotor    = hardwareMap.get(DcMotor.class, "the_arm");
 
 
         /* Most skid-steer/differential drive robots require reversing one motor to drive forward.
         for this robot, we reverse the right motor.*/
         leftDrive.setDirection(DcMotor.Direction.REVERSE);
         rightDrive.setDirection(DcMotor.Direction.FORWARD);
         leftDriveBack.setDirection(DcMotor.Direction.FORWARD);
         rightDriveBack.setDirection(DcMotor.Direction.REVERSE);
 
 
         /* Setting zeroPowerBehavior to BRAKE enables a "brake mode". This causes the motor to slow down
         much faster when it is coasting. This creates a much more controllable drivetrain. As the robot
         stops much quicker. */
         leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
         rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
         leftDriveBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
         rightDriveBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
         armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
 
         /*This sets the maximum current that the control hub will apply to the arm before throwing a flag */
         ((DcMotorEx) armMotor).setCurrentAlert(5,CurrentUnit.AMPS);
 
 
         /* Before starting the armMotor. We'll make sure the TargetPosition is set to 0.
         Then we'll set the RunMode to RUN_TO_POSITION. And we'll ask it to stop and reset encoder.
         If you do not have the encoder plugged into this motor, it will not run in this code. */
         /* armMotor.setTargetPosition(0);
         armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
         armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); */
         armMotor.setTargetPosition(0);
         armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
         armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODERS);
 
 
         /* Define and initialize servos.*/
         intake = hardwareMap.get(CRServo.class, "intake");
         wrist  = hardwareMap.get(Servo.class, "wrist");
 
         /* Make sure that the intake is off, and the wrist is folded in. */
         intake.setPower(INTAKE_OFF);
         wrist.setPosition(WRIST_FOLDED_IN);
 
         /* Send telemetry message to signify robot waiting */
         telemetry.addLine("Robot Ready.");
         telemetry.update();
 
         //armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
         /* Wait for the game driver to press play */
         waitForStart();
 
         /* Run until the driver presses stop */
         while (opModeIsActive()) {
 
             /* Set the drive and turn variables to follow the joysticks on the gamepad.
             the joysticks decrease as you push them up. So reverse the Y axis. */
             left_y = -gamepad1.left_stick_y;
             left_x = gamepad1.left_stick_x;
             right_x  = gamepad1.right_stick_x;
 
 
             
 
             /* Set the motor power to the variables we've mixed and normalized call drive function*/
             driveMechanum(left_y,left_x,right_x);
 
 
 
             /* Here we handle the three buttons that have direct control of the intake speed.
             These control the continuous rotation servo that pulls elements into the robot,
             If the user presses A, it sets the intake power to the final variable that
             holds the speed we want to collect at.
             If the user presses X, it sets the servo to Off.
             And if the user presses B it reveres the servo to spit out the element.*/
 
             /* TECH TIP: If Else statements:
             We're using an else if statement on "gamepad1.x" and "gamepad1.b" just in case
             multiple buttons are pressed at the same time. If the driver presses both "a" and "x"
             at the same time. "a" will win over and the intake will turn on. If we just had
             three if statements, then it will set the intake servo's power to multiple speeds in
             one cycle. Which can cause strange behavior. */
 
             if (gamepad1.a) {
                 intake.setPower(INTAKE_COLLECT);
             }
             else if (gamepad1.x) {
                 intake.setPower(INTAKE_OFF);
             }
             else if (gamepad1.b) {
                 intake.setPower(INTAKE_DEPOSIT);
             }
             
             if (gamepad1.left_bumper) {
                ArmTarget = ArmTarget+0.5;
             }
             else if (gamepad1.left_trigger>=0.5) {
                 ArmTarget = ArmTarget-0.5;
             }
             
             if (gamepad1.dpad_left) {
                /* This turns off the intake, folds in the wrist, and moves the arm
                back to folded inside the robot. This is also the starting configuration */
                armPosition = ARM_COLLAPSED_INTO_ROBOT;
                intake.setPower(INTAKE_OFF);
                wrist.setPosition(WRIST_FOLDED_IN);
            }

            else if (gamepad1.dpad_right){
                /* This is the correct height to score SPECIMEN on the HIGH CHAMBER */
                armPosition = ARM_SCORE_SPECIMEN;
                wrist.setPosition(WRIST_FOLDED_IN);
            }

            else if (gamepad1.dpad_up){
                /* This sets the arm to vertical to hook onto the LOW RUNG for hanging */
                armPosition = ARM_ATTACH_HANGING_HOOK;
                intake.setPower(INTAKE_OFF);
                wrist.setPosition(WRIST_FOLDED_IN);
            }

            else if (gamepad1.dpad_down){
                /* this moves the arm down to lift the robot up once it has been hooked */
                armPosition = ARM_WINCH_ROBOT;
                intake.setPower(INTAKE_OFF);
                wrist.setPosition(WRIST_FOLDED_IN);
        }
            else if (gamepad1.right_trigger>0.5) {
           wrist.setPosition(WRIST_FOLDED_OUT);
                
            }
 
             armPcontroller();
 
 
 
             /* Check to see if our arm is over the current limit, and report via telemetry. */
             if (((DcMotorEx) armMotor).isOverCurrent()){
                 telemetry.addLine("MOTOR EXCEEDED CURRENT LIMIT!");
             }
 
 
             /* send telemetry to the driver of the arm's current position and target position */
             telemetry.addData("armTarget: ", ArmTarget);
             telemetry.addData("arm Encoder: ", armMotor.getCurrentPosition()/ARM_TICKS_PER_DEGREE);
             telemetry.addData("arm Position: ", armPosition);
             telemetry.update();
 
         }
     }
     public void driveMechanum(double y, double x, double spin) {
         leftDrive.setPower(y+x+spin);
         leftDriveBack.setPower(y-x+spin);
         rightDrive.setPower(y-x-spin);
         rightDriveBack.setPower(y+x-spin);
         sleep(10); 
          }
         //right = positive, up = positive, left = negative, down = negative, s = super duper more awesome
         // than grandma grandpas and other peeps, sydney p.s. no one likes dand
     public void driveTank(double leftSpeed, double rightSpeed) {
         leftDrive.setPower(leftSpeed);
         rightDrive.setPower(rightSpeed);
         sleep(10);
          }
 
     public void stopDriveBrake() {
         leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
         rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
         leftDriveBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
         rightDriveBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
         leftDrive.setPower(0);
         rightDrive.setPower(0);
         leftDriveBack.setPower(0);
         rightDriveBack.setPower(0);
         }   
         
     public void armPcontroller(){
         armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
         double armPosition= (armMotor.getCurrentPosition())/ARM_TICKS_PER_DEGREE;
         double error = ArmTarget - armPosition;
         double kp=1.00/10;
         double speed=kp*error;
         armMotor.setPower(speed);
                     telemetry.addData("arm Position: ", armPosition);
                     telemetry.addData("arm speed: ", speed);
            
 
     }
 }