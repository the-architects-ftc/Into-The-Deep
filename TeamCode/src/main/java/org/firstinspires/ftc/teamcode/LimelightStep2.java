package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
@Autonomous(name = "LimelightStep2", group = "Limelight")
public class LimelightStep2 extends CommonUtil{
    //define limelight
    private Limelight3A limelight;
    double thres = 4.15; //calibrate the limelight to find the perfect ta value for thes bc of new crosshair

    double disin = 0.0;

    @Override
    public void runOpMode(){
        //motor setup
        initialize(hardwareMap);
        setMotorOrientation();
        //limelight setup
        limelight = hardwareMap.get(Limelight3A.class, "org/firstinspires/ftc/teamcode/limelight");
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
                double oldta = ta;
                //check if sample is far from the claw
                while (Math.abs(ta - thres)> 0.3){
                    disin = Math.abs(ta-thres) * 2.5; //adjust the 2.5 to lower if u want it to move less and higher if u want it to move more
                    telemetry.addData("disin",disin);
                    telemetry.update();
                    if (disin < 1 ){
                        telemetry.addData("Finished","Moving");
                        telemetry.update();

                        sleep(999999999);
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
                        sleep(99999999);

                    }

                    if (Math.abs(ta - oldta) <= 0.1){
                        sleep(99999999);
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

                    sleep(99999999);

                }
                result = limelight.getLatestResult();
                oldta = ta;
                ta = result.getTa();
//                while (ta-thres > 0.3){
//                    disin = Math.abs(ta-thres) * 2.5; //adjust the 2.5 to lower if u want it to move less and higher if u want it to move more
//                    moveBackwards_wDistance_wGyro(38,0.7,3);
//                    setMotorToZeroPower();
//                    result = limelight.getLatestResult();
//                    ta = result.getTa();
//                    sleep(100);
//                    telemetry.addData("Ta",ta);
//                    telemetry.update();
//



//                }
//                if (Math.abs(ta - thres)<= 0.3){
//                    telemetry.addData("Finished","Moving");
//                    telemetry.update();
//                    sleep(99999999);

//                }



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