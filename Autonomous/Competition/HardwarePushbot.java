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
import android.graphics.RenderNode;
import android.view.View;

public class HardwarePushbot {

    private LinearOpMode myOpMode;
    public DcMotor frontLeft = null;
    public DcMotor frontRight = null;
    public DcMotor backRight = null;
    public DcMotor backLeft = null;
    public DcMotor intakeMotor = null;
    public DcMotor wobbleMotor = null;
    public Servo wobbleServo = null;
    public DcMotor shooterMotor = null;
    public Servo shooterServo = null;
    public Servo webcamServo = null;
    //public DcMotor shooterMotor = null;
    //public Servo FoundationMoverRight = null;
    //public CRServo IntakeServoR = null;
    //public Servo FoundationMoverLeft = null;
    //public CRServo IntakeServoL = null;
    //public DcMotor XRailMotor = null;
    //public Servo PivotServo = null;
    //public Servo capstoneServo = null;
    private double driveAxial = 0;   // Positive is forward
    private double driveLateral = 0;   // Positive is right
    private double driveYaw = 0;   // Positive is CCW
//    public DcMotor  hang     = null;
//    public DcMotor  extend     = null;
//    public Servo push = null;
//    public Servo park = null;

    /* local OpMode members. */
    HardwareMap hardwareMap = null;
    private ElapsedTime period = new ElapsedTime();

    /* Constructor */
    public HardwarePushbot() {

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        //hardwareMap = ahwMap;
        hardwareMap = ahwMap;
        // Define and Initialize Motors
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        wobbleMotor = hardwareMap.get(DcMotor.class, "wobbleMotor");
        wobbleServo = hardwareMap.get(Servo.class, "wobbleServo");
        shooterMotor = hardwareMap.get(DcMotor.class, "shooterMotor");
        shooterServo = hardwareMap.get(Servo.class, "shooterServo");
        webcamServo = hardwareMap.get(Servo.class, "webcamServo");
        //FoundationMoverLeft = hardwareMap.get(Servo.class, "FoundationMoverLeft");
        //FoundationMoverRight = hardwareMap.get(Servo.class, "FoundationMoverRight");
        //IntakeServoL = hardwareMap.get(CRServo.class, "IntakeServoL");
        //IntakeServoR = hardwareMap.get(CRServo.class, "IntakeServoR");
        //XRailMotor = hardwareMap.get(DcMotor.class, "xRailMotor");
        //PivotServo = hardwareMap.get(Servo.class, "pivotServo");
        //capstoneServo = hardwareMap.get(Servo.class, "capstoneServo");
        //hang            = hardwareMap.get(DcMotor.class, "hang");
        //extend          = hardwareMap.get(DcMotor.class, "extend");
        //push            = hardwareMap.get(Servo.class, "markerPush");
        //park            = hardwareMap.get(Servo.class, "craterPark");

        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.REVERSE);
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

    public void changeMode(DcMotor.RunMode x) {
        frontLeft.setMode(x);
        backLeft.setMode(x);
        frontRight.setMode(x);
        backRight.setMode(x);
    }

    public void changeSpeed(double x) {
        frontLeft.setPower(x);
        frontRight.setPower(x);
        backRight.setPower(x);
        backLeft.setPower(x);
    }
}
