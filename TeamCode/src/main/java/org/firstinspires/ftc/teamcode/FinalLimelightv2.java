package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//fix moving forward
//add ftc dashboard
//make stuff move faster
//add claw
@Autonomous(name = "FinalLimelightv2", group = "Limelight")
public class FinalLimelightv2 extends CommonUtil {
    //define limelight
    private Limelight3A limelight;
    double thres = 4.15; //calibrate the limelight to find the perfect ta value for thes bc of new crosshair

    double disin = 0.0;
    int check = 0;
    double ta = 0.0;
    double oldta = 0.0;
    int leck = 0;
    @Override
    public void runOpMode() {
        initialize(hardwareMap);
        setMotorOrientation();
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        telemetry.setMsTransmissionInterval(11);
        telemetry.update();
        limelight.pipelineSwitch(3);
        limelight.start();
        waitForStart();

        while (opModeIsActive()) {


            LLResult result = limelight.getLatestResult();
            double tx = result.getTx();
            if (result != null && result.isValid()) {
                while (tx < -1) {
                    result = limelight.getLatestResult();
                    tx = result.getTx();

                    //move it to the left
                    moveSideways_wCorrection("left", 2, 0.7, 1);
                    result = limelight.getLatestResult();
                    tx = result.getTx();
                }
                if (tx >= -1 && tx <= 1) {
                    check = 1;
                }
                result = limelight.getLatestResult();
                tx = result.getTx();
                while (tx > 1){
                    result = limelight.getLatestResult();
                    tx = result.getTx();
                    //move it to the left
                    moveSideways_wCorrection("right", 2, 0.7, 1);
                    result = limelight.getLatestResult();
                    tx = result.getTx();
                }
                if (tx >= -1 && tx <= 1) {
                    check = 1;
                }
                if (check == 1){
                    while (Math.abs(ta - thres)> 0.3){
                        disin = Math.abs(ta-thres) * 2.5; //adjust the 2.5 to lower if u want it to move less and higher if u want it to move more
                        telemetry.addData("disin",disin);
                        telemetry.update();
                        if (disin < 1 ){
                            telemetry.addData("Finished","Moving");
                            telemetry.update();

                            moveSideways_wCorrection("right", 3.6, 0.8, 5);
                            moveSideways_wCorrection("right", 3.6, 0.8, 5);
                            moveForward_wDistance_wGyro(6.5,0.5,1);
                            sleep(99999);
                        }
                        moveForward_wDistance_wGyro(disin + 8,0.6,1);
                        setMotorToZeroPower();
                        turn("right",1,1);
                        setMotorToZeroPower();
                        result = limelight.getLatestResult();
                        oldta = ta;
                        ta = result.getTa();
                        if (Math.abs(ta - thres)<= 0.3){
                            telemetry.addData("Finished","Moving");
                            telemetry.update();
                            moveSideways_wCorrection("right", 3.6, 0.8, 5);
                            moveSideways_wCorrection("right", 3.6, 0.8, 5);
                            moveForward_wDistance_wGyro(6.5,0.5,1);
                            sleep(99999);

                        }

                        if (Math.abs(ta - oldta) <= 0.1){
                            moveSideways_wCorrection("right", 3.6, 0.8, 5);
                            moveSideways_wCorrection("right", 3.6, 0.8, 5);
                            moveForward_wDistance_wGyro(6.5,0.5,1);
                            sleep(99999);
                        }
                        sleep(100);
                        telemetry.addData("Ta",ta);
                        telemetry.update();
                    }
                    result = limelight.getLatestResult();
                    oldta = ta;
                    ta = result.getTa();
                    if (Math.abs(ta - thres)<= 0.3){

                        turn("right",5,1);
                        telemetry.addData("Finished","Moving");
                        telemetry.update();

                        moveSideways_wCorrection("right", 3.6, 0.8, 5);
                        moveSideways_wCorrection("right", 3.6, 0.8, 5);
                        moveForward_wDistance_wGyro(6.5,0.5,1);
                        sleep(99999);

                    }
                    result = limelight.getLatestResult();
                    oldta = ta;
                    ta = result.getTa();
                }
                moveForward_wDistance_wGyro(disin + 8,0.6,1);
                setMotorToZeroPower();
                turn("right",1,1);
                setMotorToZeroPower();
                result = limelight.getLatestResult();
                oldta = ta;
                ta = result.getTa();
                if (Math.abs(ta - thres)<= 0.3){
                    telemetry.addData("Finished","Moving");
                    telemetry.update();
                    moveSideways_wCorrection("right", 3.6, 0.8, 5);
                    moveSideways_wCorrection("right", 3.6, 0.8, 5);
                    moveForward_wDistance_wGyro(6.5,0.5,1);
                    sleep(99999);

                }

                if (Math.abs(ta - oldta) <= 0.1){
                    moveSideways_wCorrection("right", 3.6, 0.8, 5);
                    moveSideways_wCorrection("right", 3.6, 0.8, 5);
                    moveForward_wDistance_wGyro(6.5,0.5,1);
                    sleep(99999);
                }
                result = limelight.getLatestResult();
                oldta = ta;
                ta = result.getTa();


            }

        }
    }
}


