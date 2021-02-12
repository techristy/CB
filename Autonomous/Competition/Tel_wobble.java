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

import com.qualcomm.hardware.bosch.BNO055IMU;
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

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


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

@TeleOp(name="Tel wobble", group="Linear Opmode")
//@Disabled
public class  Tel_wobble extends LinearOpMode {
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
    private DcMotor liftMotor = null;
    private DcMotor  intake = null;
    private DcMotor shooter = null;
    //private DcMotor shooter = null;
    /*private DcMotor xRail = null;
    private Servo Pivot = null;

    private Servo leftHolder = null;
    private Servo rightHolder = null;

    private Servo capstoneServo = null;

    private CRServo contServo_left = null;
    */
    private Servo gripServo = null;
    private Servo shooterServo = null;
    //private Servo gripServo = null;

    //private Servo FoundationRight = null;

    //private Servo FoundationLeft = null;
    boolean check = false;
    double i;
    boolean check2 = false;
    BNO055IMU imu;
    Orientation lastAngles = new Orientation();
    double                  globalAngle, power = .50, correction, rotation;
    PIDController           pidRotate, pidDrive;

//    private Servo contArm_left = null;
//    private Servo contArm_right = null;

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    //@Override
    public void runOpMode(){


        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        pidRotate = new PIDController(.003, .00003, 0);
        pidDrive = new PIDController(.05, 0, 0);

        telemetry.addData("Mode", "calibrating...");
        telemetry.update();
        while (!isStopRequested() && !imu.isGyroCalibrated())
        {
            sleep(50);
            idle();
        }

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
        liftMotor = hardwareMap.get(DcMotor.class, "wobbleMotor");
        intake = hardwareMap.get(DcMotor.class,"intakeMotor");
        shooter = hardwareMap.get(DcMotor.class,"shooterMotor");
        shooterServo = hardwareMap.get(Servo.class,"shooterServo");
        //shooter = hardwareMap.get(DcMotor.class,"shooter");


        //xRail = hardwareMap.get(DcMotor.class, "xRailMotor");
        //Pivot = hardwareMap.get(Servo.class, "pivotServo");

        //gripServo = hardwareMap.get(Servo.class, "gripServo");
        //capstoneServo = hardwareMap.get(Servo.class, "capstoneServo");

        //contServo_left = hardwareMap.get(CRServo.class, "IntakeServoL");
        gripServo = hardwareMap.get(Servo.class, "wobbleServo");
        //leftHolder = hardwareMap.get(Servo.c0lass, "leftHolder");
        //rightHolder = hardwareMap.get(Servo.class, "rightHolder");

        //FoundationLeft = hardwareMap.get(Servo.class, "FoundationMoverLeft");
        //FoundationRight = hardwareMap.get(Servo.class, "FoundationMoverRight");


        //xRail.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        //Pivot.setPosition(0);
        //gripServo.setPosition(0.7);

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.FORWARD);
        //capstoneServo.setPosition(0.1);
        //rightHolder.setPosition(0.20);
        //leftHolder.setPosition(0.86);
        //gripServo.setPosition(0.3);
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        long millis = System.currentTimeMillis();
        runtime.reset();
        resetAngle();
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            telemetry.addData(getAngle()+"", "");
            telemetry.update();
            if(gamepad2.right_bumper){
                intake.setPower(0.9) ;
            }
            else{intake.setPower(0);}
            //intake.setPower(gamepad2.left_stick_x*0.9);
            if(gamepad2.left_bumper){
                shooter.setPower(0.8);
            }
            else if(gamepad2.left_trigger > 0.1){
                shooter.setPower(0.65);
            }
            else{shooter.setPower(0);}
            if(gamepad2.a){
                shooterServo.setPosition(0);
            }
            else if(gamepad2.b){
                shooterServo.setPosition(1);
            }
            else{}
            //////////////////////////////////////////////////////////////

            /** Strafing */

            frontLeft.setPower(-((1 * gamepad1.left_stick_y) + 3*gamepad1.left_trigger - 3*gamepad1.right_trigger + (0.75 * (gamepad1.dpad_left ? 1 : 0)) - (0.75 * (gamepad1.dpad_right ? 1 : 0))
                    - (0.25 * (gamepad1.dpad_up ? 1 : 0)) + (0.3 * (gamepad1.dpad_down ? 1 : 0))));
            frontRight.setPower(-((1 * gamepad1.right_stick_y) - 3*gamepad1.left_trigger + 3*gamepad1.right_trigger - (0.75 * (gamepad1.dpad_left ? 1 : 0)) + (0.75 * (gamepad1.dpad_right ? 1 : 0))
                    - (0.25 * (gamepad1.dpad_up ? 1 : 0)) + (0.3 * (gamepad1.dpad_down ? 1 : 0))));
            backLeft.setPower(-((1 * gamepad1.left_stick_y) - 0.6*gamepad1.left_trigger + 0.6*gamepad1.right_trigger - (0.5 * (gamepad1.dpad_left ? 1 : 0)) + (0.5 * (gamepad1.dpad_right ? 1 : 0))
                    - (0.25 * (gamepad1.dpad_up ? 1 : 0)) + (0.3 * (gamepad1.dpad_down ? 1 : 0))));
            backRight.setPower(-((1 * gamepad1.right_stick_y) + 0.6*gamepad1.left_trigger - 0.6*gamepad1.right_trigger + (0.5 * (gamepad1.dpad_left ? 1 : 0)) - (0.5 * (gamepad1.dpad_right ? 1 : 0))
                    - (0.25 * (gamepad1.dpad_up ? 1 : 0)) + (0.3 * (gamepad1.dpad_down ? 1 : 0))));




            /**Shooter*/
            if(gamepad2.x){
                gripServo.setPosition(0.1);
            }
            else if(gamepad2.y){
                gripServo.setPosition(0.9);
            }

            liftMotor.setPower((0.6 * gamepad2.right_stick_y) - (0.5 * (gamepad2.dpad_left ? 1 : 0)) + (0.5 * (gamepad2.dpad_right ? 1 : 0)));
            //- (0.4 * (gamepad2.dpad_up ? 1 : 0)) + (0.4 * (gamepad2.dpad_down ? 1 : 0))

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

            /** Show the elapsed game time and wheel power. */

            //telemetry.addData("Status", "Run Time: " + runtime.toString());
            //telemetry.addData("Motors", "left (%.2f), right (%.2f)", frontLeft.getPower(), frontRight.getPower());
            //telemetry.update();

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

        }
    }

    private double getAngle()
    {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;
    }

    private void resetAngle()
    {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = 0;
    }
}