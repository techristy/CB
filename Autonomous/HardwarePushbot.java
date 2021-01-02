package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import java.lang.reflect.Array;
import com.qualcomm.robotcore.eventloop.opmode.*;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.Range;

import java.util.List;
import org.firstinspires.ftc.robotcore.external.*;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import java.util.Timer;
import android.app.Activity;
import android.graphics.Color;
import android.view.View;

public class HardwarePushbot {

    private LinearOpMode myOpMode;
    public DcMotor  frontLeftWheel   = null;
    public DcMotor  frontRightWheel  = null;
    public DcMotor  backLeftWheel   = null;
    public DcMotor  backRightWheel  = null;
    public Servo FoundationMoverRight = null;
    public CRServo IntakeServoR = null;
    public Servo FoundationMoverLeft = null;
    public CRServo IntakeServoL = null;
    public DcMotor XRailMotor = null;
    public Servo PivotServo = null;
    public Servo capstoneServo = null;
    private double  driveAxial      = 0 ;   // Positive is forward
    private double  driveLateral    = 0 ;   // Positive is right
    private double  driveYaw        = 0 ;   // Positive is CCW
//    public DcMotor  hang     = null;
//    public DcMotor  extend     = null;
//    public Servo push = null;
//    public Servo park = null;

    /* local OpMode members. */
    HardwareMap hardwareMap =  null;
    private ElapsedTime period = new ElapsedTime();

    /* Constructor */
    public HardwarePushbot(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        //hardwareMap = ahwMap;
        hardwareMap = ahwMap;
        // Define and Initialize Motors
        frontLeftWheel  = hardwareMap.get(DcMotor.class, "frontLeftWheel");
        frontRightWheel = hardwareMap.get(DcMotor.class, "frontRightWheel");
        backLeftWheel   = hardwareMap.get(DcMotor.class, "backLeftWheel");
        backRightWheel  = hardwareMap.get(DcMotor.class, "backRightWheel");
        FoundationMoverLeft = hardwareMap.get(Servo.class, "FoundationMoverLeft");
        FoundationMoverRight = hardwareMap.get(Servo.class, "FoundationMoverRight");
        IntakeServoL = hardwareMap.get(CRServo.class, "IntakeServoL");
        IntakeServoR = hardwareMap.get(CRServo.class, "IntakeServoR");
        XRailMotor = hardwareMap.get(DcMotor.class, "xRailMotor");
        PivotServo = hardwareMap.get(Servo.class, "pivotServo");
        capstoneServo = hardwareMap.get(Servo.class, "capstoneServo");
        //hang            = hardwareMap.get(DcMotor.class, "hang");
        //extend          = hardwareMap.get(DcMotor.class, "extend");
        //push            = hardwareMap.get(Servo.class, "markerPush");
        //park            = hardwareMap.get(Servo.class, "craterPark");

        frontLeftWheel.setDirection(DcMotor.Direction.FORWARD);
        frontRightWheel.setDirection(DcMotor.Direction.REVERSE);
        backLeftWheel.setDirection(DcMotor.Direction.FORWARD);
        backRightWheel.setDirection(DcMotor.Direction.REVERSE);
//        hang.setDirection(DcMotor.Direction.FORWARD);
//        extend.setDirection(DcMotor.Direction.FORWARD);

        // Set all motors to zero power
        changeSpeed(0);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        changeMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Initialize ALL installed servos.
        // push.setPosition(MID_SERVO);
    }

    public void changeMode(DcMotor.RunMode x){
        frontLeftWheel.setMode(x);
        backLeftWheel.setMode(x);
        frontRightWheel.setMode(x);
        backRightWheel.setMode(x);
    }

    public void changeSpeed(double x){
        frontLeftWheel.setPower(x);
        frontRightWheel.setPower(x);
        backRightWheel.setPower(x);
        backLeftWheel.setPower(x);
    }

    public void sleep(int i) {
    }

    public void moveRobot() {
        // calculate required motor speeds to acheive axis motions
        double frontLeftDrivePower = driveLateral + driveAxial - driveYaw;
        double backLeftDrivePower = driveAxial - driveLateral - driveYaw;
        double frontRightDrivePower = driveAxial - driveLateral + driveYaw;
        double backRightDrivePower = driveAxial + driveLateral + driveYaw;

        // normalize all motor speeds so no values exceeds 100%.
        double max = Math.max(Math.abs(frontLeftDrivePower), Math.abs(backLeftDrivePower));
        max = Math.max(Math.max(max, Math.abs(frontRightDrivePower)), backRightDrivePower);
        if (max > 1.0)
        {
            frontLeftDrivePower /= max;
            backLeftDrivePower /= max;
            frontRightDrivePower /= max;
            backRightDrivePower /= max;
        }

        // Set drive motor power levels.
        frontLeftWheel.setPower(frontLeftDrivePower);
        backLeftWheel.setPower(backLeftDrivePower);
        frontRightWheel.setPower(frontRightDrivePower);
        backRightWheel.setPower(backRightDrivePower);

        // Display Telemetry
        myOpMode.telemetry.addData("Axes  ", "A[%+5.2f], L[%+5.2f], Y[%+5.2f]", driveAxial, driveLateral, driveYaw);
        myOpMode.telemetry.addData("Wheels", "FL[%+5.2f], BL[%+5.2f], FR[%+5.2f],BR[%+5.2f]", frontLeftDrivePower,backLeftDrivePower,frontRightDrivePower,backRightDrivePower);
    }
    public void setAxial(double axial)      {driveAxial = Range.clip(axial, -1, 1);}
    public void setLateral(double lateral)  {driveLateral = Range.clip(lateral, -1, 1); }
    public void setYaw(double yaw)          {driveYaw = Range.clip(yaw, -1, 1); }

}
