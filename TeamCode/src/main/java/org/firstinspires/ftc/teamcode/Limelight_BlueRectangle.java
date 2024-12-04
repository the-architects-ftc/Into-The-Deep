package org.firstinspires.ftc.teamcode;


import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Limelight_BlueRectangle", group = "Limelight")
public class Limelight_BlueRectangle extends CommonUtil{

    private Limelight3A limelight;


    @Override
    public void runOpMode(){

        int check = 1;
        double thres = 0.0; //callibrate new ta value because of new crosshair
        //Initialize Hardware
        initialize(hardwareMap);
        setMotorOrientation();
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        telemetry.setMsTransmissionInterval(11);
        telemetry.update();
        limelight.pipelineSwitch(3);
        limelight.start();



        waitForStart();

        while (opModeIsActive()){
            LLResult result = limelight.getLatestResult();
            if (result != null && result.isValid()) {
                double tx = result.getTx();
                double ty = result.getTy();
                double ta = result.getTa();

                telemetry.addData("Target X", tx);
                telemetry.addData("Target Y", ty);
                telemetry.addData("Target Area", ta);
                telemetry.update();


                //sideways step 1 go to a negative x-axis for sample
                if (tx > 0) {
                    moveSideways_wCorrection("right", 3, 0.7, 500);
                }

                //forward/backward step 2 go to the right ta for sample
                ta = result.getTa();
                while (check == 1);
                {
                    ta = result.getTa();
                    while (ta - thres < 0) ;
                    {

                        int time = 150;
                        double fpow = Math.abs(ta - thres);
                        if (fpow > 0.6) {
                            fpow = 0.6;
                            time += 75;
                        }

                        if (fpow < 0.2) {
                            fpow = 0.2;
                            time -= 75;
                        }
                        fl.setPower(fpow);
                        fr.setPower(fpow);
                        bl.setPower(fpow);
                        br.setPower(fpow);
                        sleep(time);
                        setMotorToZeroPower();
                        ta = result.getTa();
                    }
                    while (ta - thres > 0.1) ;
                    {
                        ta = result.getTa();

                        int time = 150;
                        double bpow = Math.abs(ta - thres);
                        if (bpow > 0.6) {
                            bpow = 0.6;
                            time += 75;
                        }
                        if (bpow < 0.2) {
                            bpow = 0.2;
                            time -= 75;
                        }
                        fl.setPower(-bpow);
                        fr.setPower(-bpow);
                        bl.setPower(-bpow);
                        br.setPower(-bpow);
                        sleep(time);
                        setMotorToZeroPower();
                        ta = result.getTa();
                    }
                    ta = result.getTa();

                    if (ta - thres >= 0 && ta-thres <= 0.1){
                        check = 0;

                    }
                }

//
//                while (check == 0){
//                    if (Math.abs(tx+13) < 0.1) {
//                        telemetry.addData("Sideways:", "Left");
//                        telemetry.update();
//                        double lowt = tx + 13;
//                        fl.setPower(-lowt);
//                        fr.setPower(lowt);
//                        bl.setPower(lowt);
//                        br.setPower(-lowt);
//                        sleep(150);
//
//                    }
//                    if (Math.abs(tx+13) > 0.1) {
//                        telemetry.addData("Sideways:", "Right");
//                        telemetry.update();
//                        double lowt = tx + 13;
//                        fl.setPower(lowt);
//                        fr.setPower(-lowt);
//                        bl.setPower(-lowt);
//                        br.setPower(lowt);
//                        sleep(150);
//
//                    }
//
//                    if (Math.abs(tx+13) <= 1){
//                        telemetry.addData("Side to side movement","Completed TX");
//                        telemetry.update();
//                        check = 1; //end
//                    }
//
//
//                }
//
//
//                //add claw stuff here
            }
        }


    }
}
