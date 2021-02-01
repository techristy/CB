/* Copyright (c) 2017 FIRST. All rights reserved.
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

import android.hardware.Camera;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.vuforia.CameraDevice;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="13353TeleOP", group="Linear Opmode")
//@Disabled
public class Tel11212020 extends LinearOpMode {
    public static int convertBoolean(boolean x){
        if(x){
            return 1;
        }
        else return 0;
    }
    // Declare OpMode members.

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    /** (Continuous/Regular) Servo and Motor Usage */

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor frontLeft = null;
    private DcMotor frontRight = null;
    private DcMotor backRight = null;
    private DcMotor backLeft = null;

    private DcMotor xRail = null;
    private Servo Pivot = null;

    private Servo leftHolder = null;
    private Servo rightHolder = null;

    private Servo capstoneServo = null;

    private CRServo contServo_left = null;
    private CRServo contServo_right = null;

    private Servo gripServo = null;

    private Servo FoundationRight = null;
    private Servo FoundationLeft = null;
    boolean check = false;
    double i;
    boolean check2 = false;
//    private Servo contArm_left = null;
//    private Servo contArm_right = null;

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    //@Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        /////
        boolean XYHold = false;        // Gripper Toggle
        int ClickNum = 0;                // A Button Toggle Count
        boolean AHold = false;          // A Button Press
        /////
        float LeftTrigVal = 0;           // Initializing Trigger Value for Strafing Left
        float RightTrigVal = 0;
        // Initializing Trigger Value for Strafing Right
        /////
        //boolean Torch = false;
//        Camera cam = Camera.open();
        /////

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

        /** (Continuous/Regular) Servo and Motor Usage */

        frontLeft  = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft  = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");

        xRail = hardwareMap.get(DcMotor.class, "xRailMotor");
        Pivot = hardwareMap.get(Servo.class, "pivotServo");

        gripServo = hardwareMap.get(Servo.class, "gripServo");
        capstoneServo = hardwareMap.get(Servo.class, "capstoneServo");

        contServo_left = hardwareMap.get(CRServo.class, "IntakeServoL");
        contServo_right = hardwareMap.get(CRServo.class, "IntakeServoR");

        leftHolder = hardwareMap.get(Servo.class, "leftHolder");
        rightHolder = hardwareMap.get(Servo.class, "rightHolder");

        FoundationLeft = hardwareMap.get(Servo.class, "FoundationMoverLeft");
        FoundationRight = hardwareMap.get(Servo.class, "FoundationMoverRight");


        xRail.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        Pivot.setPosition(0);
        gripServo.setPosition(0.7);
        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.FORWARD);
        capstoneServo.setPosition(0.1);
        rightHolder.setPosition(0.20);
        leftHolder.setPosition(0.86);
        gripServo.setPosition(0.3);
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        long millis = System.currentTimeMillis();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Setup a variable for each drive wheel to save power level for telemetry
            double leftPower;
            double rightPower;

            // Choose to drive using either Tank Mode, or POV Mode
            // Comment out the method that's not used.  The default below is POV.

            // POV Mode uses left stick to go forward, and right stick to turn.
            // - This uses basic math to combine motions and is easier to drive straight.
            //double drive = -gamepad1.left_stick_y;
            //double turn  =  gamepad1.right_stick_x;
            //leftPower    = Range.clip(drive + turn, -1.0, 1.0) ;
            //rightPower   = Range.clip(drive - turn, -1.0, 1.0) ;

            // Tank Mode uses one stick to control each wheel.
            // - This requires no math, but it is hard to drive forward slowly and keep straight.
            // leftPower  = -gamepad1.left_stick_y ;
            // rightPower = -gamepad1.right_stick_y ;

            // Send calculated power to wheels
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

            /** Strafing */

            frontLeft.setPower((0.6*gamepad1.left_stick_y) + gamepad1.left_trigger - gamepad1.right_trigger + (0.5 * (gamepad1.dpad_left ? 1 : 0)) - (0.5 * (gamepad1.dpad_right ? 1 : 0))
                    - (0.25 * (gamepad1.dpad_up ? 1 : 0)) + (0.3 * (gamepad1.dpad_down ? 1 : 0)));
            frontRight.setPower((0.6*gamepad1.right_stick_y) - gamepad1.left_trigger + gamepad1.right_trigger - (0.5 * (gamepad1.dpad_left ? 1 : 0)) + (0.5 * (gamepad1.dpad_right ? 1 : 0))
                    - (0.25 * (gamepad1.dpad_up ? 1 : 0)) + (0.3 * (gamepad1.dpad_down ? 1 : 0)));
            backLeft.setPower((0.6*gamepad1.left_stick_y) - gamepad1.left_trigger + gamepad1.right_trigger - (0.5 * (gamepad1.dpad_left ? 1 : 0)) + (0.5 * (gamepad1.dpad_right ? 1 : 0))
                    - (0.25 * (gamepad1.dpad_up ? 1 : 0)) + (0.3 * (gamepad1.dpad_down ? 1 : 0)));
            backRight.setPower((0.6*gamepad1.right_stick_y) + gamepad1.left_trigger - gamepad1.right_trigger + (0.5 * (gamepad1.dpad_left ? 1 : 0)) - (0.5 * (gamepad1.dpad_right ? 1 : 0))
                    - (0.25 * (gamepad1.dpad_up ? 1 : 0)) + (0.3 * (gamepad1.dpad_down ? 1 : 0)));

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

            /** Foundation Moving */

            if(gamepad1.x == true) {
                FoundationLeft.setPosition(0.85);    //Pull Position 0.75
                FoundationRight.setPosition(0.15);    //Pull Position 0.2
            }
            else{
                FoundationLeft.setPosition(0.28);
                FoundationRight.setPosition(0.88);
            }




/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

            /** Changing Intake Wheel Directions */
            /////////////////////////////////////////////////////////////////////////////////////////////////////////
            if(gamepad1.left_bumper) {
                contServo_left.setPower(-2);
                contServo_right.setPower(2);
            }
            else if(gamepad1.right_bumper || gamepad2.x){
                contServo_left.setPower(0.05);
                contServo_right.setPower(-0.05);
            }
            else{
                contServo_left.setPower(2);
                contServo_right.setPower(-2);
            }
            /*
            if(gamepad2.right_bumper){
                FoundationLeft.setPosition(0.46);
                FoundationRight.setPosition(0.355);
            }
            else if(gamepad2.left_bumper){
                FoundationLeft.setPosition(0);
                FoundationRight.setPosition(0.9);
            }*/

            /** X-Rails Slide and Pivot*/




//////////////////////////////////////////////////////////////////////////////////
            if(gamepad2.a)
            {
                rightHolder.setPosition(0.20);
                leftHolder.setPosition(0.86);
            }
            if(gamepad2.b && !(System.currentTimeMillis()<(millis+1000))){
                rightHolder.setPosition(0.88);
                leftHolder.setPosition(0.2);
            }
            if(gamepad2.x){
                gripServo.setPosition(0.05);
            }
            else gripServo.setPosition(0.3);

            ////////////////////////////////////////////////////
            /*if(gamepad2.a)
            {
                rightHolder.setPosition(0.17);
                leftHolder.setPosition(0.93);
                gripServo.setPosition(0.05);
            }
            if(gamepad2.b)
            {
                gripServo.setPosition(0.3);
                rightHolder.setPosition(0.88);
                leftHolder.setPosition(0.2);
            }
            if(gamepad2.x){
                rightHolder.setPosition(0.28);
                leftHolder.setPosition(0.89);
            }*/



            if(gamepad2.dpad_left)
            {
                Pivot.setPosition(0);
            }
            if(gamepad2.dpad_right)
            {
                Pivot.setPosition(1);
            }






            // Slide //

            if(gamepad2.dpad_up) {
                xRail.setPower(-0.7);
            } else if(gamepad2.dpad_down) {
                xRail.setPower(0.4);
            } else{
                xRail.setPower(0.0);
            }

            //CAPSTONE//

            if(gamepad2.left_bumper){
                capstoneServo.setPosition(0.1);
            }
            else{
                capstoneServo.setPosition(0.56);
            }

            // Pivot //ttttttttttttttttttttttttttttttttw e3rfeow qerer
            /*if((gamepad2.dpad_left && (Pivot.getPosition()<0.5))||check){
                if(check){
                    i=i+0.5;
                    Pivot.setPosition(i/100);
                    if(i==100){
                        check=false;
                    }
                }
                else{
                    check=true;
                    i=5;
                }
                //Pivot.setPosition(0.95);
            }

            else if((gamepad2.dpad_right && Pivot.getPosition()>0.5)||check2){
                if(check2){
                    i=i-0.5;
                    Pivot.setPosition(i/100);
                    if(i==0){
                        check2=false;
                    }
                }
                else{
                    check2=true;
                    i=100;
                }
                //Pivot.setPosition(0.95);
            }*/

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

            /** Strafing Test (Depreciated)


             ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

             /** Torch Toggle */

//            if(gamepad1.b == true) {
//                Camera.Parameters p = cam.getParameters();
//                p.setFlashMode(Camera.Parameters.FLASH_MODE_TORCH);
//                cam.setParameters(p);
//                cam.startPreview();
//            } else {
//                cam.stopPreview();
//                cam.release();
//            }
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

            /** Show the elapsed game time and wheel power. */

            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "left (%.2f), right (%.2f)", frontLeft.getPower(), frontRight.getPower());
            telemetry.update();

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

        }
    }
}