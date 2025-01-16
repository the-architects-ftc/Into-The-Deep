package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotorEx;



import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import com.qualcomm.robotcore.hardware.ColorSensor;

import java.util.Timer;


public class CommonUtil extends LinearOpMode {

    Orientation myRobotOrientation;

    double ENC2DIST = 200/29; //2000.0/48.0; // FW/BW
    double ENC2DIST_SIDEWAYS = 291.1/34.44;
    ElapsedTime timer = new ElapsedTime();

    //imu init
    BHI260IMU imu;
    BHI260IMU.Parameters myIMUParameters;
    YawPitchRollAngles robotOrientation;

    //motor / servo init
    DcMotor bl = null;
    DcMotor fl = null;
    DcMotor fr = null;
    DcMotor br = null;

    DcMotor m2 = null;



    Servo s3 = null;

    Servo s2 = null;
    Servo s5 = null;
    Servo lc = null;
    Servo rc = null;

    CRServo s4 = null;

    //All Our functions!

    // Initialize
    public void initialize(HardwareMap hardwareMap){

        //setup
        telemetry.setAutoClear(true);

        // map imu
        imu = hardwareMap.get(BHI260IMU.class,"imu");
        myIMUParameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP,RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD)
        );
        imu.initialize(myIMUParameters);
        imu.resetYaw();

        // Start imu initialization

        telemetry.addData("initialize:Gyro Status", "Initialized");
        telemetry.update();
        // map motors
        bl = hardwareMap.get(DcMotorEx.class, "LB");
        fl = hardwareMap.get(DcMotor.class, "LF");
        fr = hardwareMap.get(DcMotor.class, "RF");
        br = hardwareMap.get(DcMotor.class, "RB");


        m2 = hardwareMap.get(DcMotor.class, "lSlide");



        s3 = hardwareMap.get(Servo.class, "BOP");
        s4 = hardwareMap.get(CRServo.class, "iWheel");

        s2 = hardwareMap.get(Servo.class,"iElbow");
        s5 = hardwareMap.get(Servo.class,"iWrist");
        lc = hardwareMap.get(Servo.class,"ClawL");
        rc = hardwareMap.get(Servo.class,"ClawR");


        //s3.setDirection(Servo.Direction.REVERSE);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        m2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }





    // Set motor directions
    public void setMotorOrientation()
    {
        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        bl.setDirection(DcMotor.Direction.REVERSE);
        fl.setDirection(DcMotor.Direction.REVERSE);
        fr.setDirection(DcMotor.Direction.FORWARD);
        br.setDirection(DcMotor.Direction.FORWARD);


    }

    public void weirdturn(){
        fl.setPower(0);
        fr.setPower(1);
        bl.setPower(-1);
        br.setPower(1);
        sleep(400);
        fl.setPower(0);
        fr.setPower(0);
        bl.setPower(0);
        br.setPower(0);
    }



    public void wierdforward(int time, int comb){
        bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        bl.setDirection(DcMotor.Direction.FORWARD);
//        fl.setDirection(DcMotor.Direction.FORWARD);
//        fr.setDirection(DcMotor.Direction.FORWARD);
//        br.setDirection(DcMotor.Direction.FORWARD);
        if (comb == 1){
            m2.setDirection(DcMotor.Direction.REVERSE);
            m2.setPower(1);
        }
        br.setPower(0.65); //was 0.9
        fl.setPower(0.5);
        fr.setPower(0.5);
        bl.setPower(0.5);


        sleep(time);
        fl.setPower(0);
        fr.setPower(0);
        bl.setPower(0);
        br.setPower(0);
        sleep(20);
        m2.setPower(0);
    }


    public void wierdbackward(int time){
        bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fl.setPower(-0.8);
        fr.setPower(-0.8);
        bl.setPower(-0.8);
        br.setPower(-0.8);
        sleep(time);
        fl.setPower(-0);
        fr.setPower(-0);
        bl.setPower(-0);
        br.setPower(-0);
    }

    public void wierdslideup(int time){
        m2.setDirection(DcMotor.Direction.REVERSE);
        m2.setPower(1);
        sleep(time);
        m2.setPower(0);
    }
    public void wierdslidedown(int time){
        m2.setDirection(DcMotor.Direction.FORWARD);
        m2.setPower(1);
        sleep(time);
        m2.setPower(0);
    }


    //reset encoder counts
    public void resetMotorEncoderCounts()
    {
        // Reset encoder counts kept by motors
        bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fl.setMode((DcMotor.RunMode.STOP_AND_RESET_ENCODER));
        fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        br.setMode((DcMotor.RunMode.STOP_AND_RESET_ENCODER));
        telemetry.addData("Encoder", "Count Reset");  // telemetry: Mode Waiting
        telemetry.update();

    }

    public void drone_Test(){
        s3.setPosition(0);
    }

    //motor power 0
    public void setMotorToZeroPower()
    {
        bl.setPower(0);
        fl.setPower(0);
        fr.setPower(0);
        br.setPower(0);
    }

    public void setZeroPowerBehavior(){

        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }


//    public void intakeOn(){
//        s4.setPower(1);
//    }
//    public void intakeReverse(){
//        s4.setPower(-1);
//    }
//    public void intakeOff(){
//        s4.setPower(0);
//    }
    public void whipitout() {
        s5.setPosition(0.5); //was0.5
        s2.setPosition(0.4);

    }

    public void comein(){
        s5.setPosition(1);
        s2.setPosition(0.1);
    }

    public void bsketrdy(){
        s3.setDirection(Servo.Direction.FORWARD);
        s3.setPosition(0.55); //up
    }

    public void armReleaseP1(){
        s2.setPosition(0.8);
        s5.setPosition(-0.2);
    }

    public void armReleaseP2(){
        s2.setPosition(0);
        s5.setPosition(1);
        sleep(100);
        armMiddle();
    }




    public void armDown() {
        s5.setPosition(0.5);
        s2.setPosition(1);
        //skibidi sigma i am the rizzla
    }
    public void armMiddle() {
        s2.setPosition(0.8);
        sleep(200);
        s5.setPosition(0.5);

    }
    public void clawOpen() {
        rc.setDirection(Servo.Direction.FORWARD);
        lc.setDirection(Servo.Direction.REVERSE);
        rc.setPosition(1);
        lc.setPosition(1);
    }
    public void clawClose() {
        rc.setDirection(Servo.Direction.FORWARD);
        lc.setDirection(Servo.Direction.REVERSE);
        rc.setPosition(0);
        lc.setPosition(0);
    }
    public void basketUp() { s3.setDirection(Servo.Direction.FORWARD);
        s3.setPosition(0.25); }
    public void basketDown() { s3.setDirection(Servo.Direction.FORWARD);
        s3.setPosition(0.75); }
    public double PID_Turn (double targetAngle, double currentAngle, String minPower) {
        double sign = 1;
        double power = (targetAngle - currentAngle) * 0.0054; // was 0.006
        if (minPower.equalsIgnoreCase("on")&& (power != 0)) {
            sign = Math.signum(power);
            power = Math.max(Math.abs(power), 0.2);
            power = power*sign;
//            if (power < 0.2){
//                power = 0.2;
//            }
        }
        return power;
    }

    public double PID_FB (double targetEC, double currentEC, double Mpower)
    {
        double power = (targetEC -currentEC)*0.003;
        //if (Mpower > 0.3){
        //    Mpower = 0.3;
        //}
        if (power > Mpower) {
            power = Mpower;
        }else if(power < 0.2) {
            power = 0.2; //-1 * (Mpower);
        }
        return power;
    }



    //move forwards with gyro
    public int moveForward_wDistance_wGyro(double DistanceAbsIn,double Mpower,int timeToStop)
    {

        ElapsedTime runtime= new ElapsedTime();
        double currZAngle = 0;
        double prevZAngle = 0;
        int currEncoderCount = 0;
        int prevEncoderCount = 0;
        double currErrEC = 0;
        double encoderAbsCounts = ENC2DIST*DistanceAbsIn;
        telemetry.addData("EC Target", encoderAbsCounts);
        telemetry.update();

        // Resetting encoder counts
        resetMotorEncoderCounts();
        runtime.reset();
        double power2 = 0;

        // Setting motor to run in runToPosition
        bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        double bl_pos = bl.getCurrentPosition();
        double fl_pos = fl.getCurrentPosition();
        double br_pos = br.getCurrentPosition();
        double fr_pos = fr.getCurrentPosition();
        double min_pos = Math.min(Math.min(br_pos,bl_pos),Math.min(fl_pos,fr_pos));

        // move forward
        while ((min_pos < encoderAbsCounts) && (runtime.seconds() < timeToStop)) {
            myRobotOrientation = imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
            currZAngle = myRobotOrientation.thirdAngle;
            double correction = PID_Turn(0, currZAngle, "off");
            correction = 0;
            currEncoderCount = (int)(min_pos);
            double power = PID_FB(encoderAbsCounts, Math.abs(currEncoderCount), Mpower);
            int moveTime = (int) ((10 * Mpower) * ((double) 50 / 3));
            int movePause = (int) ((10 * Mpower) * ((double) 10 / 3));
            telemetry.addData("moveTime", moveTime);
            telemetry.addData("movePause", movePause);
            telemetry.update();

            bl.setPower(power - correction); //was power for the bottom few
            fl.setPower(power - correction);
            fr.setPower(power - correction);
            br.setPower(power - correction);
            telemetry.addData("fw:power", power);
            telemetry.addData("curr pos", min_pos);
            telemetry.addData("encoderAbsCounts", encoderAbsCounts);
            telemetry.update();
            sleep(moveTime);
            setMotorToZeroPower();
            sleep(movePause);
            bl_pos = bl.getCurrentPosition();
            fl_pos = fl.getCurrentPosition();
            br_pos = br.getCurrentPosition();
            fr_pos = fr.getCurrentPosition();
            min_pos = Math.min(Math.min(br_pos,bl_pos),Math.min(fl_pos,fr_pos));
        }
        telemetry.addData("curr pos", min_pos);
        telemetry.addData("encoderAbsCounts", encoderAbsCounts);
        telemetry.update();
        // apply zero power to avoid continuous power to the wheels
        setMotorToZeroPower();
        // return current encoder count
        //currEncoderCount = bl.getCurrentPosition();
        myRobotOrientation = imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
        imu.resetYaw();
        return (int)(min_pos);
    }

    //move backwards with gyro correction
    public int moveBackwards_wDistance_wGyro(double DistanceAbsIn,double Mpower,int timeToStop)
    {
        ElapsedTime runtime= new ElapsedTime();
        double currZAngle = 0;
        double prevZAngle = 0;
        int currEncoderCount = 0;
        int prevEncoderCount = 0;
        double currErrEC = 0;

        double encoderAbsCounts = ENC2DIST *DistanceAbsIn;
        telemetry.addData("bw:encoder target", encoderAbsCounts);
        telemetry.update();

        // Resetting encoder counts
        resetMotorEncoderCounts();
        runtime.reset();

        // Setting motor to run in runToPosition
        bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        double bl_pos = bl.getCurrentPosition();
        double fl_pos = fl.getCurrentPosition();
        double br_pos = br.getCurrentPosition();
        double fr_pos = fr.getCurrentPosition();
        double max_pos = Math.max(Math.max(br_pos,bl_pos),Math.max(fl_pos,fr_pos));

        while((max_pos > -encoderAbsCounts) && (runtime.seconds() < timeToStop)) {
            myRobotOrientation = imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
            currZAngle = myRobotOrientation.thirdAngle;
            double correction = PID_Turn(0,currZAngle,"off");
            correction = 0;
            currEncoderCount = (int)(max_pos);
            double power = PID_FB(encoderAbsCounts,Math.abs(currEncoderCount),Mpower);

            bl.setPower(-power-correction);
            fl.setPower(-power-correction);
            fr.setPower(-power+correction);
            br.setPower(-power+correction);
            telemetry.addData("bw:power", power);
            telemetry.addData("curr pos", max_pos);
            telemetry.addData("encoderAbsCounts", -encoderAbsCounts);
            telemetry.update();
            bl_pos = bl.getCurrentPosition();
            fl_pos = fl.getCurrentPosition();
            br_pos = br.getCurrentPosition();
            fr_pos = fr.getCurrentPosition();
            max_pos = Math.max(Math.max(br_pos,bl_pos),Math.max(fl_pos,fr_pos));

            //idle();
        }

//        turnToZeroAngle();
        // apply zero power to avoid continuous power to the wheels
        setMotorToZeroPower();

        // return current encoder count
        //currEncoderCount = bl.getCurrentPosition();
        myRobotOrientation = imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
        telemetry.addData("curr pos", max_pos);
        telemetry.addData("encoderAbsCounts", -encoderAbsCounts);
        telemetry.update();
        imu.resetYaw();
        return ((int)(max_pos));
    }





    //public void0.4);
    //}



    public void turn(String direction, double targetAngle,int timeToStop)
    {
        ElapsedTime runtime= new ElapsedTime();
        imu.resetYaw();
        if (direction.equalsIgnoreCase("right")){
            myRobotOrientation = imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);

            telemetry.addData("turnRight:Init Angle",myRobotOrientation.thirdAngle);
            telemetry.update();

            runtime.reset();

            while ((Math.abs(targetAngle-Math.abs(myRobotOrientation.thirdAngle))>0.1) && (runtime.seconds() < timeToStop)) {
                myRobotOrientation = imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
                double power = PID_Turn(targetAngle, Math.abs(myRobotOrientation.thirdAngle),"on");
                bl.setPower(power);
                fl.setPower(power);
                fr.setPower(-power);
                br.setPower(-power);
            }
            telemetry.addData("turnRight:Curr Angle",myRobotOrientation.thirdAngle);
            telemetry.update();
            bl.setPower(0);
            fl.setPower(0);
            fr.setPower(0);
            br.setPower(0);
        } else if(direction.equalsIgnoreCase("left")){
            myRobotOrientation = imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
            telemetry.addData("turnLeft:Init Angle",myRobotOrientation.thirdAngle);
            telemetry.update();

            runtime.reset();

            while ((Math.abs(targetAngle-Math.abs(myRobotOrientation.thirdAngle))>0.1) && (runtime.seconds() < timeToStop)) {
                myRobotOrientation = imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
                double power = PID_Turn(targetAngle, Math.abs(myRobotOrientation.thirdAngle),"on");
                bl.setPower(-power);
                fl.setPower(-power);
                fr.setPower(power);
                br.setPower(power);
            }
            telemetry.addData("turnLeft:Curr Angle",myRobotOrientation.thirdAngle);
            telemetry.update();
            bl.setPower(0);
            fl.setPower(0);
            fr.setPower(0);
            br.setPower(0);

        }
        imu.resetYaw(); // [ AARUSH ]

    }

    public void turnToZeroAngle()
    {
        myRobotOrientation = imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
        double targetAngle = myRobotOrientation.thirdAngle;
        if (targetAngle > 0)
        {
            telemetry.addData("turnToZeroAngle:targetAngle",targetAngle);
            telemetry.update();
            turn("right", Math.abs(targetAngle),3);
        }
        else if (targetAngle < 0)
        {
            telemetry.addData("turnToZeroAngle:targetAngle",targetAngle);
            telemetry.update();
            turn("left", Math.abs(targetAngle),3);
        }
        imu.resetYaw();
    }



    public void moveSideways_wCorrection(String direction, double DistanceAbsIn, double motorAbsPower,int timeToStop)
    {
        ElapsedTime runtime= new ElapsedTime();
        //turnToZeroAngle();
        int currEncoderCount = 0;
        double encoderAbsCounts = ENC2DIST_SIDEWAYS*DistanceAbsIn; //2000/42
//        telemetry.addData("sideways:target ", encoderAbsCounts);
//        telemetry.update();
        setMotorOrientation();
        // Resetting encoder counts
        resetMotorEncoderCounts();
//        telemetry.addData("Encoder count target",encoderAbsCounts);

        // Setting motor to run in runToPosition\
        bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Wait for robot to finish this movement
        double refEC = 0;


        runtime.reset();

        while ((refEC < encoderAbsCounts) && (runtime.seconds() < timeToStop)) {

            double frEC = fr.getCurrentPosition();
            double blEC = bl.getCurrentPosition();
            double flEC = fl.getCurrentPosition();
            double brEC = br.getCurrentPosition();
            double frCorr = 1;
            double blCorr = 1;
            double flCorr = 1;
            double brCorr = 1;
            if (frEC != 0 ) {
                frEC = Math.abs(frEC);
                refEC = frEC;
                blEC = Math.abs(blEC);
                refEC = Math.min(refEC, blEC);
                flEC = Math.abs(flEC);
                refEC = Math.min(refEC, flEC);
                brEC = Math.abs(brEC);
                refEC = Math.min(refEC, brEC);
                if (refEC == 0) {
                    refEC = 1;
                }
                frCorr = refEC / frEC;
                blCorr = refEC / blEC;
                flCorr = refEC / flEC;
                brCorr = refEC / brEC;
            }
            double flPow = -motorAbsPower * flCorr;
            double blPow = motorAbsPower * blCorr;
            double frPow = motorAbsPower * frCorr;
            double brPow = -motorAbsPower * brCorr;

            if (direction.equalsIgnoreCase("left")) {
                fl.setPower(flPow);
                bl.setPower(blPow);
                fr.setPower(frPow);
                br.setPower(brPow);
//                telemetry.addData("sideways:currEncoderCount ", currEncoderCount);
//                telemetry.update();

            }
            else if (direction.equalsIgnoreCase("right")) {
                fl.setPower(-flPow);
                bl.setPower(-blPow);
                fr.setPower(-frPow);
                br.setPower(-brPow);
            }
            idle();
        }
        //turnToZeroAngle();

        // apply zero power to avoid continuous power to the wheels
        setMotorToZeroPower();

        // return current encoder count
        currEncoderCount = bl.getCurrentPosition();
        myRobotOrientation = imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
//        telemetry.addData("sideways:currEncoderCount (final)", currEncoderCount);
//        telemetry.update();
        imu.resetYaw();

    }

    public void encoder_test(double encoderAbsCounts, double power)
    {
        bl.setDirection(DcMotor.Direction.REVERSE);
        fl.setDirection(DcMotor.Direction.REVERSE);
        fr.setDirection(DcMotor.Direction.FORWARD);
        br.setDirection(DcMotor.Direction.FORWARD);
        resetMotorEncoderCounts();
        bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        while (bl.getCurrentPosition() < encoderAbsCounts) {
            myRobotOrientation = imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
            bl.setPower(power);
            fl.setPower(power);
            fr.setPower(power);
            br.setPower(power);
            telemetry.update();
            idle();
        }
//        bl.setPower(-2*power);
//        fl.setPower(-2*power);
        fr.setPower(-2*power);
//        br.setPower(-2*power);
        sleep(200);
        bl.setPower(0);
        fl.setPower(0);
        fr.setPower(0);
        br.setPower(0);

        telemetry.addData("encoder count b1",bl.getCurrentPosition());
        telemetry.addData("encoder count br",br.getCurrentPosition());
        telemetry.addData("encoder count fl",fl.getCurrentPosition());
        telemetry.addData("encoder count fr",fr.getCurrentPosition());
        telemetry.update();

        sleep(5000);
        telemetry.addData("encoder count b1",bl.getCurrentPosition());
        telemetry.addData("encoder count br",br.getCurrentPosition());
        telemetry.addData("encoder count fl",fl.getCurrentPosition());
        telemetry.addData("encoder count fr",fr.getCurrentPosition());
        telemetry.update();

    }

    public void slideUp(double power, int encoderAbsCounts,int timeToStop) {
        ElapsedTime runtime = new ElapsedTime();
        m2.setDirection(DcMotor.Direction.REVERSE);
        m2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        telemetry.addData("Start count", m2.getCurrentPosition());
        telemetry.update();

        while ((m2.getCurrentPosition() > -encoderAbsCounts) && (runtime.seconds() < timeToStop)){
            m2.setPower(power);
            telemetry.addData("Count M2",m2.getCurrentPosition());
            telemetry.update();
            idle();
        }
        m2.setPower(0.05); // set power to 0 so the motor stops running

    }

    public void slideDown(double power, int encoderAbsCounts,int timeToStop) {
        ElapsedTime runtime = new ElapsedTime();
        m2.setDirection(DcMotor.Direction.FORWARD);
        m2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        telemetry.addData("Start count", m2.getCurrentPosition());
        telemetry.update();

        while ((m2.getCurrentPosition() < encoderAbsCounts)&&(runtime.seconds() < timeToStop)){
            m2.setPower(-power);
            telemetry.addData("Count M2",m2.getCurrentPosition());
            telemetry.update();
            idle();
        }
        m2.setPower(0);
        // set power to 0 so the motor stops running

    }

    public void realign_Sideways(String direction){


        if (direction.equalsIgnoreCase("left")) {
            bl.setPower(-0.2);
            fl.setPower(0.2);
            fr.setPower(0.2);
            br.setPower(-0.2);
        }
        else if (direction.equalsIgnoreCase("right")) {
            bl.setPower(0.2);
            fl.setPower(-0.2);
            fr.setPower(-0.2);
            br.setPower(0.2);
        }
        sleep(1500);
        bl.setPower(0);
        fl.setPower(0);
        fr.setPower(0);
        br.setPower(0);

    }

    public void realign_FB(String direction){
        if (direction.equalsIgnoreCase("forward")) {
            bl.setPower(0.2);
            fl.setPower(0.2);
            fr.setPower(0.2);
            br.setPower(0.2);
        }
        else if (direction.equalsIgnoreCase("backward")) {
            bl.setPower(-0.2);
            fl.setPower(-0.2);
            fr.setPower(-0.2);
            br.setPower(-0.2);
        }
        sleep(1500);
        bl.setPower(0);
        fl.setPower(0);
        fr.setPower(0);
        br.setPower(0);

    }


    @Override
    public void runOpMode() throws InterruptedException {
    }


}
