package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
/*
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
*/
import java.util.ArrayList;
import java.util.List;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;


import static org.firstinspires.ftc.teamcode.ConceptTensorFlowObjectDetectionWebcam.LABEL_FIRST_ELEMENT;
import static org.firstinspires.ftc.teamcode.ConceptTensorFlowObjectDetectionWebcam.LABEL_SECOND_ELEMENT;
import static org.firstinspires.ftc.teamcode.ConceptTensorFlowObjectDetectionWebcam.TFOD_MODEL_ASSET;
//import static org.firstinspires.ftc.teamcode.WebcamTest.VUFORIA_KEY;

@Autonomous(name="RightRed", group="Pushbot")

public class RightRed extends LinearOpMode
{

    private static int valQUAD = -1;
    private static int valSingle = -1;
    private static int valZero = -1;

    private static float offsetX = .75f/8f;//changing this moves the three rects and the three circles left or right, range : (-2, 2) not inclusive
    private static float offsetY = 1.5f/8f;//changing this moves the three rects and circles up or down, range: (-4, 4) not inclusive

    private static float[] midPos = {4f/8f+offsetX, 4f/8f+offsetY};//0 = col, 1 = row
    private static float[] leftPos = {2f/8f+offsetX, 4f/8f+offsetY};
    private static float[] rightPos = {6f/8f+offsetX, 4f/8f+offsetY};


    //DRIVE, IMU, AND ACCEL CONSTANTS

    //BNO055IMU imu;
    //Orientation lastAngles = new Orientation();
    double                  globalAngle, power = .50, correction, rotation;
    PIDController           pidRotate, pidDrive;
    HardwarePushbot robot = new HardwarePushbot();
    private ElapsedTime runtime = new ElapsedTime();
    static final double COUNTS_PER_MOTOR_REV = 1120 ;    //ANDYMARK Motor Encoder ticks
    static final double DRIVE_GEAR_REDUCTION = 1.0;
    static final double WHEEL_DIAMETER_INCHES = 4.0 ;
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);
    static  double DRIVE_SPEED = 0.9;
    static final double TURN_SPEED = 0.575;
    LogisticFunction function;


    double servoStartingPosition = 0.5;

    double distanceBetweenBlocks = 9.5;


    //boolean center;

    boolean left;

    boolean right;


    boolean targetVisible;

    double blockPosition;

    boolean values[] = new boolean[3];

    String positionArray[] = null;


    //for 90 degrees
    //bigger turn = 26.5
    //smaller turn = 21.6

    @Override
    public void runOpMode() throws InterruptedException{

        teleUpdate("status", "Starting runOpMode");
        robot.init(hardwareMap);
        robot.changeMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.changeMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //robot.FoundationMoverLeft.setPosition(0.3);    //Pull Position 0.75
        //robot.FoundationMoverRight.setPosition(0.88);
        function = new LogisticFunction(0.6);
        teleUpdate("status", "Starting runOpMode");
        robot.init(hardwareMap);
        robot.frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //robot.FoundationMoverLeft.setPosition(0.3);    //Pull Position 0.75
        //robot.FoundationMoverRight.setPosition(0.88);
        robot.changeMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        // parameters.mode                = BNO055IMU.SensorMode.IMU;
        //parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        //parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        //parameters.loggingEnabled      = false;
        teleUpdate("WWWWWWWWWWWWWWWWWWWWWWWWWWWWW","");
        //imu = hardwareMap.get(BNO055IMU.class, "imu2");
        teleUpdate("EEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEee","");
        //imu.initialize(parameters);
        teleUpdate("QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQqqq","");
        pidRotate = new PIDController(.003, .00003, 0);
        teleUpdate("AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA","");
        pidDrive = new PIDController(.05, 0, 0);

        telemetry.addData("Mode", "calibrating...");
        telemetry.update();
        //while (!isStopRequested() && !imu.isGyroCalibrated())
        {
            //    sleep(50);
            //    idle();
        }
        telemetry.addData("Mode", "waiting for start");
        //telemetry.addData("imu calib status", imu.getCalibrationStatus().toString());
        telemetry.update();
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
        String VUFORIA_KEY =
                "  ARGKFNf/////AAABmeWmTIKr70aQrGH7lC6M8xBPdcMfnaNjD/dopWNwdsWuQbrZLFQZZBr/eFBlpHuykY0IY4f9Y34OVFaL4NRxmFd4ghxNkwK3Cjl/4Jo6bf/v+ovD7Tqdf8cT0A3McQF2rxOPE8fsmaC2TfCr8nZquqbbaTZT7bxtuvi8skuLfHg0BNRGaKtEYyPaJ+wdvAcJZ8+2rZ6q+77Ooh2teMYGmJRe+KDD8LmIMn5Jh/r/Lbm9WqjmxuSV6NxwAwpqTPydgJAE/19fXRVbC4+vGWAiiAxd/UIrLxDtgwekkiudCLSa1r1Y8XjtaTeUUWYXl7+iAxkAOX3ZYa84fFrPGnFvYdhjnIuRGo4AgL6dvb/pQEaK ";

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");
        VuforiaLocalizer vuforia = null;
        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
        TFObjectDetector tfod;
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.8f;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
        Thread.sleep(1000);
        waitForStart();

        //encoderDrive(30, "drive");
        //Thread.sleep(1000);
        int[] values = {valQUAD,valSingle,valZero};
        //webcam.closeCameraDevice();
        if(values[0]==0){
            teleUpdate("QUAD","");
            navigation("c");
            encoderDrive(5,0.5,"strafe");
            encoderDrive(-100,0.9,"drive");
            halfTurn("counterclockwise");
            halfTurn("counterclockwise");
            robot.wobbleMotor.setPower(4000);
            robot.wobbleMotor.setPower(0);
            robot.wobbleServo.setPosition(0.1);

        }
        else if(values[1]==0){
            teleUpdate("SINGLE","");
            navigation("b");
            encoderDrive(-75,0.5,"drive");
            halfTurn("counterclockwise");
            robot.wobbleMotor.setPower(4000);
            robot.wobbleMotor.setPower(0);
            robot.wobbleServo.setPosition(0.1);
        }
        else{
            teleUpdate("ZERO","");
//            navigation("a");
            encoderDrive(5,0.5,"strafe");
            encoderDrive(-60,0.9,"drive");
            halfTurn("counterclockwise");
            halfTurn("counterclockwise");
            robot.wobbleMotor.setPower(4000);
            robot.wobbleMotor.setPower(0);
            robot.wobbleServo.setPosition(0.1);

        }
        //drive to the blocks and start vuforia
    }

    public void blockServoControlLeft(boolean control){
        if(control){

            //robot.FoundationMoverLeft.setPosition(0.8);    //Pull Position 0.75
        }

        else  {
            //robot.FoundationMoverLeft.setPosition(0.3);    //Pull Position 0.75
        }

    }
    public void blockServoControlRight(boolean control){
        if(control){

            //robot.FoundationMoverRight.setPosition(0.2);    //Pull Position 0.75
        }

        else  {
            //robot.FoundationMoverRight.setPosition(0.88);    //Pull Position 0.75
        }

    }

    public void navigation(String type) throws InterruptedException{


        if(type.equals("a")) {
            encoderDrive(1,0.5,"strafe");
            encoderDrive(7,0.9,"drive");
            halfTurn("clockwise");
            robot.wobbleMotor.setPower(4000);
            robot.wobbleMotor.setPower(0);
            robot.wobbleServo.setPosition(0.1);
            /*blockServoControlRight(true);
            Thread.sleep(333);
            encoderDrive(-1,  0.9, "drive");
            halfTurn("clockwise");
            encoderDrive(41,  1, "drive");
            blockServoControlRight(false);
            encoderDrive(-64, 1, "drive");
            halfTurn("counterclockwise");
            encoderDrive(5,  0.9, "drive");
            blockServoControlRight(true);
            Thread.sleep(300);
            encoderDrive(-5.5,  0.9, "drive");
            halfTurn("clockwise");
            encoderDrive(65, 1, "drive");
            blockServoControlRight(false);
            encoderDrive(-11,  0.7, "drive");
            */
        }

        if(type.equals("b")) {
            //encoderDrive(27,0.9,"drive");
            //encoderDrive(3,0.9,"strafe");
            /*blockServoControlRight(true);
            Thread.sleep(333);
            encoderDrive(-1,  1.2, "drive");
            halfTurn("clockwise");
            encoderDrive(50,  1, "drive");
            blockServoControlRight(false);
            encoderDrive(-61.5, 1, "drive");
            halfTurn("counterclockwise");
            encoderDrive(6.5,  0.9, "drive");
            blockServoControlLeft(true);
            Thread.sleep(300);
            encoderDrive(-5.5,  0.9, "drive");
            halfTurn("clockwise");
            encoderDrive(65, 1, "drive");
            blockServoControlLeft(false);
            encoderDrive(-11,  0.7, "drive");
            encoderDrive(4,0.8,"strafe");
            */
        }

        if(type.equals("c")) {
            //encoderDrive(28,0.9,"drive");
            //encoderDrive(12,0.9,"strafe");
            /*blockServoControlRight(true);
            Thread.sleep(333);
            encoderDrive(-1,  1.2, "drive");
            halfTurn("clockwise");
            encoderDrive(57,  1, "drive");
            blockServoControlRight(false);
            encoderDrive(-60, 1, "drive");
            halfTurn("counterclockwise");
            encoderDrive(7.4,1,"strafe");
            encoderDrive(2,  0.9, "drive");
            blockServoControlLeft(true);
            Thread.sleep(300);
            encoderDrive(-4.5,  0.9, "drive");
            halfTurn("clockwise");
            encoderDrive(74, 1, "drive");
            blockServoControlLeft(false);
            encoderDrive(-15,  0.8, "drive");
            encoderDrive(4,0.8,"strafe");
            */
        }
    }


    public void fullTurn(String type){
        resetAngle();
        if(type.equals("counterclockwise")){
            long time = System.currentTimeMillis();
            while (getAngle() <= 180 && (System.currentTimeMillis()<(time+3000))) {
                power = (.75*2*0.684/5.063) * (-Math.pow((((getAngle())+2.9)/37.4),2) + 4.5*((getAngle()+2.9)/37.4)) + 0.159;
                telemetry.addLine(""+power);
                telemetry.addLine(""+getAngle());
                telemetry.update();
                robot.frontLeft.setPower(-power);
                robot.frontRight.setPower(power);
                robot.backRight.setPower(power);
                robot.backLeft.setPower(-power);
            }
            robot.frontLeft.setPower(0);
            robot.frontRight.setPower(0);
            robot.backRight.setPower(0);
            robot.backLeft.setPower(0);
        }
        if(type.equals("clockwise")){
            long time = System.currentTimeMillis();
            while (getAngle() >= -180 && (System.currentTimeMillis()<(time+3000))) {
                power = (.75*2*0.684/5.063) * (-Math.pow((((-getAngle())+2.9)/37.4),2) + 4.5*((-getAngle()+2.9)/37.4)) + 0.159;
                telemetry.addLine(""+power);
                telemetry.addLine(""+getAngle());
                telemetry.update();
                robot.frontLeft.setPower(power);
                robot.frontRight.setPower(-power);
                robot.backRight.setPower(-power);
                robot.backLeft.setPower(power);
            }
            robot.frontLeft.setPower(0);
            robot.frontRight.setPower(0);
            robot.backRight.setPower(0);
            robot.backLeft.setPower(0);
        }
        robot.changeMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.changeMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void halfTurn(String type){
        resetAngle();
        if(type.equals("counterclockwise")){
            long time = System.currentTimeMillis();
            while (getAngle() <= 82 && (System.currentTimeMillis()<(time+2000))) {
                power = (.75*0.684/5.063) * (-Math.pow((((getAngle())+6.5)/19.5),2) + 4.5*((getAngle()+6.5)/19.5)) + 0.159;
                teleUpdate(""+power,"");
                robot.frontLeft.setPower(-power);
                robot.backRight.setPower(power);
                robot.frontRight.setPower(power);
                robot.backLeft.setPower(-power);
            }
            robot.frontLeft.setPower(0);
            robot.backRight.setPower(0);
            robot.frontRight.setPower(0);
            robot.backLeft.setPower(0);
        }
        if(type.equals("clockwise")){
            long time = System.currentTimeMillis();
            while (getAngle() >= -82 && (System.currentTimeMillis()<(time+2000))) {
                power = (.75*0.684/5.063) * (-Math.pow((((-getAngle())+6.5)/19.5),2) + 4.5*((-getAngle()+6.5)/19.5)) + 0.159;
                teleUpdate(""+power,"");
                robot.frontLeft.setPower(power);
                robot.backRight.setPower(-power);
                robot.frontRight.setPower(-power);
                robot.backLeft.setPower(power);
            }
            robot.frontLeft.setPower(0);
            robot.backRight.setPower(0);
            robot.frontRight.setPower(0);
            robot.backLeft.setPower(0);
        }
        robot.changeMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.changeMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }


    public void encoderDrive(double inches, double pow, String driveMode) throws InterruptedException {
        if(driveMode.equals("drive")){
            //settargetposition is inverse
            //if setpower command for backward is -, then getpowers for both are both positive
            pidDrive.setSetpoint(0);
            pidDrive.setOutputRange(0, power);
            pidDrive.setInputRange(-90, 90);
            pidDrive.enable();
            resetAngle();
            int startPos1 = robot.frontLeft.getCurrentPosition();
            int startPos2 = robot.backLeft.getCurrentPosition();
            int startPos3 = robot.frontRight.getCurrentPosition();
            int startPos4 = robot.backRight.getCurrentPosition();
            robot.frontLeft.setTargetPosition((int)(inches*COUNTS_PER_INCH));
            robot.backRight.setTargetPosition((int)(inches*COUNTS_PER_INCH));
            robot.frontRight.setTargetPosition((int)(inches*COUNTS_PER_INCH));
            robot.backLeft.setTargetPosition((int)(inches*COUNTS_PER_INCH));
            double currentPosInches;
            if(inches>=0) {
                //robot.changeSpeed(power);
                while (robot.frontRight.getTargetPosition()>robot.frontRight.getCurrentPosition()||
                        robot.frontLeft.getTargetPosition()>robot.frontLeft.getCurrentPosition()||
                        robot.backRight.getTargetPosition()>robot.backRight.getCurrentPosition()||
                        robot.backLeft.getTargetPosition()>robot.backLeft.getCurrentPosition()) {
                    correction = pidDrive.performPID(getAngle());
                    currentPosInches = ((robot.frontLeft.getCurrentPosition() - startPos1) / COUNTS_PER_INCH);
                    teleUpdate("CURRENTPOSINCHES: "+currentPosInches+"","");
                    power = function.getPowerAt(currentPosInches, inches, pow, "drive");
                    robot.frontLeft.setPower(power - correction);
                    robot.backRight.setPower(power + correction);
                    robot.frontRight.setPower(power + correction);
                    robot.backLeft.setPower(power - correction);
                }
            }
            else{
                //robot.changeSpeed(-power);
                while (robot.frontRight.getTargetPosition()<robot.frontRight.getCurrentPosition()||
                        robot.frontLeft.getTargetPosition()<robot.frontLeft.getCurrentPosition()||
                        robot.backRight.getTargetPosition()<robot.backRight.getCurrentPosition()||
                        robot.backLeft.getTargetPosition()<robot.backLeft.getCurrentPosition()) {
                    correction = pidDrive.performPID(getAngle());
                    currentPosInches = ((robot.frontLeft.getCurrentPosition() - startPos1) / COUNTS_PER_INCH * -1);
                    power = -function.getPowerAt(currentPosInches, -inches, pow, "drive");
                    teleUpdate("POWER: "+ power+"","");
                    robot.frontLeft.setPower((power - correction));
                    robot.backRight.setPower((power + correction));
                    robot.frontRight.setPower((power + correction));
                    robot.backLeft.setPower((power - correction));
                }
            }
            robot.changeSpeed(0);
        }
        else if(driveMode.equals("strafe")){/////LEFT IS POSITIVE
            pidDrive.setSetpoint(0);
            pidDrive.setOutputRange(0, power);
            pidDrive.setInputRange(-90, 90);
            pidDrive.enable();
            resetAngle();

            int startPos1 = robot.frontLeft.getCurrentPosition();
            int startPos2 = robot.backLeft.getCurrentPosition();
            int startPos3 = robot.frontRight.getCurrentPosition();
            int startPos4 = robot.backRight.getCurrentPosition();
            robot.frontLeft.setTargetPosition((int)(-inches*COUNTS_PER_INCH));
            robot.backRight.setTargetPosition((int)(-inches*COUNTS_PER_INCH));
            robot.frontRight.setTargetPosition((int)(inches*COUNTS_PER_INCH));
            robot.backLeft.setTargetPosition((int)(inches*COUNTS_PER_INCH));
            double currentPosInches;
            //power = 0.9;
            robot.changeSpeed(power);
            if(inches>0) {
                while (robot.frontRight.getTargetPosition()>robot.frontRight.getCurrentPosition()||
                        robot.frontLeft.getTargetPosition()<robot.frontLeft.getCurrentPosition()||
                        robot.backRight.getTargetPosition()<robot.backRight.getCurrentPosition()||
                        robot.backLeft.getTargetPosition()>robot.backLeft.getCurrentPosition()) {
                    telemetry.addData("Correction", correction);
                    telemetry.update();
                    correction = pidDrive.performPID(getAngle());
                    currentPosInches = ((robot.frontRight.getCurrentPosition() - startPos1) / COUNTS_PER_INCH);
                    power = function.getPowerAt(currentPosInches, inches, pow, "strafe");
                    robot.frontLeft.setPower(-(power + correction));
                    robot.backRight.setPower(-(power - correction));
                    robot.frontRight.setPower((power + correction));
                    robot.backLeft.setPower((power - correction));             //STRAFE
                }
            }
            else{
                while (robot.frontRight.getTargetPosition()<robot.frontRight.getCurrentPosition()||
                        robot.frontLeft.getTargetPosition()>robot.frontLeft.getCurrentPosition()||
                        robot.backRight.getTargetPosition()>robot.backRight.getCurrentPosition()||
                        robot.backLeft.getTargetPosition()<robot.backLeft.getCurrentPosition()) {
                    telemetry.addData("Correction", correction);
                    telemetry.update();
                    correction = pidDrive.performPID(getAngle());
                    currentPosInches = ((robot.frontRight.getCurrentPosition() - startPos1) / COUNTS_PER_INCH * -1);
                    power = -function.getPowerAt(currentPosInches, -inches, pow, "strafe");
                    robot.frontLeft.setPower(-(power + correction));
                    robot.backRight.setPower(-(power - correction));
                    robot.frontRight.setPower((power + correction));
                    robot.backLeft.setPower((power - correction));             //STRAFE
                }
            }
            robot.changeSpeed(0);
        }
        robot.changeMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.changeMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Thread.sleep(100);
    }
    public void teleUpdate(String label, String description){
        if (robot != null && robot.frontLeft != null && robot.frontRight != null) {
            telemetry.addLine().addData("Current Position",  "Running at %7d :%7d", robot.frontLeft.getCurrentPosition(), robot.frontRight.getCurrentPosition());
        }
        telemetry.addLine().addData(label + ": ", description);
        telemetry.update();
    }
    private double getAngle()
    {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.
        //Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        //double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        //if (deltaAngle < -180)
        //    deltaAngle += 360;
        //else if (deltaAngle > 180)
        //    deltaAngle -= 360;

        //globalAngle += deltaAngle;

        //lastAngles = angles;

        return globalAngle;
    }
    private void resetAngle()
    {
        //lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = 0;
    }
    enum Stage
    {//color difference. greyscale
        detection,//includes outlines
        THRESHOLD,//b&w
        RAW_IMAGE,//displays raw view
    }

    private Stage stageToRenderToViewport = Stage.detection;
    private Stage[] stages = Stage.values();


}




