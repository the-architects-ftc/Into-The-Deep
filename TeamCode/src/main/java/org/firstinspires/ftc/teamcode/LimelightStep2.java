package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
@Autonomous(name = "LimelightStep2", group = "Limelight")
public class LimelightStep2 extends CommonUtil{
    //define limelight
    private Limelight3A limelight;
    double thres = 2.0; //calibrate the limelight to find the perfect ta value for thes bc of new crosshair
    double fpow = 0.0;
    double bpow = 0.0;

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
                //get ta value
                double ta = result.getTa();
                //check if sample is far from the claw
                while (Math.abs(ta - thres)>= 0.3){
                    result = limelight.getLatestResult();
                    ta = result.getTa();
                    int time = 200;
                    //if too high make it 0.6
                    if (fpow > 0.4) {
                        time += 70;
                    }
                    //if too low make it 0.2
                    if (fpow < 0.2) {
                        time -= 70;
                    }
                    fpow = 0.35;
                    //set it to power
                    fl.setPower(fpow);
                    fr.setPower(fpow);
                    bl.setPower(fpow);
                    br.setPower(fpow);
                    //sleep for time P.S. can also add or remove time in the if fpow statements
                    sleep(time);
                    setMotorToZeroPower();
                    result = limelight.getLatestResult();
                    ta = result.getTa();
                    sleep(750);
                    telemetry.addData("Ta",ta);
                    telemetry.update();


                }
                if (Math.abs(ta - thres)<= 0.5){
                    telemetry.addData("Finished","Moving");
                    telemetry.update();
                    sleep(99999999);

                }



//                while (ta - thres > 0.1){
//                    int time = 150;
//                    bpow = Math.abs(ta - thres);
//                    //if too high make it 0.6
//                    if (bpow > 0.6) {
//                        bpow = 0.6;
//                    }
//                    //if too low make it 0.2
//                    if (bpow < 0.2) {
//                        bpow = 0.2;
//                    }
//                    //set it to power
//                    fl.setPower(-bpow);
//                    fr.setPower(-bpow);
//                    bl.setPower(-bpow);
//                    br.setPower(-bpow);
//                    //sleep for time P.S. can also add or remove time in the if bpow statements
//                    sleep(time);
//                    setMotorToZeroPower();
//                    ta = result.getTa();
//                }




            }
        }
    }
}