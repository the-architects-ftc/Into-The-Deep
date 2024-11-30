package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
@Autonomous(name = "LimelightStep3", group = "Limelight")
public class LimelightStep3 extends CommonUtil{
    //define limelight
    private Limelight3A limelight;
    double thresx = 0.0; //calibrate for threshold for tx
    double lpow = 0.0;
    double rpow = 0.0;
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
                double tx = result.getTx();

                while (tx - thresx < 0){

                    lpow = Math.abs(tx-thresx);
                    if (lpow > 0.6){
                        lpow = 0.6;
                    }
                    if (lpow < 0.4){
                        lpow = 0.4;
                    }
                    moveSideways_wCorrection("left",1,lpow,150);
                    setMotorToZeroPower();
                    tx = result.getTx();
                }
                while (tx - thresx > 0.1){
                    rpow = Math.abs(tx-thresx);
                    if (rpow > 0.6){
                        rpow = 0.6;
                    }
                    if (rpow < 0.4){
                        rpow = 0.4;
                    }
                    moveSideways_wCorrection("right",1,rpow,150);
                    setMotorToZeroPower();

                    tx = result.getTx();
                }
            }
        }
    }
}
