package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
@Autonomous(name = "LimelightStep3", group = "Limelight")
public class LimelightStep3 extends CommonUtil{
    //define limelight
    private Limelight3A limelight;
    double thresx = -21; //calibrate for threshold for tx
    double lpow = 0.0;
    double rpow = 0.0;
    double disin = 0;

    @Override
    public void runOpMode(){
        //motor setup
        initialize(hardwareMap);
        setMotorOrientation();
        //limelight setup
        limelight = hardwareMap.get(Limelight3A.class,"limelight");
        limelight.setPollRateHz(100);
        telemetry.setMsTransmissionInterval(11);
        telemetry.update();
        limelight.pipelineSwitch(3);
        limelight.start();
        //wait for start
        waitForStart();

        while (opModeIsActive()){
            LLResult result = limelight.getLatestResult();
            //check if there is target and is detected
            if (result != null && result.isValid()) {
                //get tx value
                result = limelight.getLatestResult();
                double tx = result.getTx();

                while (tx < thresx){
                    telemetry.addData("Tx",tx);
                    telemetry.update();
                    disin = Math.abs(tx-thresx) * 1.7;
                    if (disin < 0.7 ){
                        telemetry.addData("Finished","Moving");
                        telemetry.update();
                        sleep(999999999);
                    }
                    lpow = 0.5;
                    moveSideways_wCorrection("left",disin -1,0.7,2);
                    setMotorToZeroPower();
                    result = limelight.getLatestResult();
                    tx = result.getTx();
                    telemetry.addData("Tx",tx);
                    telemetry.update();
                }
                while (tx > 0.5 + thresx){
                    telemetry.addData("Tx",tx);
                    telemetry.update();
                    disin = Math.abs(tx-thresx) * 1.7;
                    if (disin < 0.7 ){
                        telemetry.addData("Finished","Moving");
                        telemetry.update();
                        sleep(999999999);
                    }
                    lpow = 0.5;
                    moveSideways_wCorrection("right",disin + 0.5,lpow,1);
                    setMotorToZeroPower();
                    result = limelight.getLatestResult();
                    tx = result.getTx();
                    telemetry.addData("Tx",tx);
                    telemetry.update();
                }

                if (Math.abs(tx - thresx) <= 0.5) {
                    moveForward_wDistance_wGyro(3,0.5,3);
                    sleep(999999);
                }
            }
            else{
                moveSideways_wCorrection("left",disin +1,0.7,2);
            }
        }
    }
}
