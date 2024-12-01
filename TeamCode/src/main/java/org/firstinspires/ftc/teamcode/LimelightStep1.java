package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;


@Autonomous(name = "LimelightStep1", group = "Limelight")
public class LimelightStep1 extends CommonUtil{
    //define limelight
    private Limelight3A limelight;

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
        telemetry.addData("here","here");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()){

            //check if there is target and is detected
            telemetry.addData("here","here");
            telemetry.update();
            LLResult result = limelight.getLatestResult();

            result = limelight.getLatestResult();
            double tx = result.getTx();
            //check if it is on the right side
            while (tx > -15) { //was 0
                result = limelight.getLatestResult();
                tx = result.getTx();
                telemetry.addData("here","here");
                telemetry.update();
                //move it to the left
                moveSideways_wCorrection("right", 3, 0.7, 500);
                result = limelight.getLatestResult();
                tx = result.getTx();
                }
            sleep(99999);
            }
        }
    }

