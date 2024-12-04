package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;


@Autonomous(name = "LimelightStep1", group = "Limelight")
public class LimelightStep1 extends CommonUtil {
    //define limelight
    private Limelight3A limelight;

    @Override
    public void runOpMode() {
        //motor setup
        initialize(hardwareMap);
        setMotorOrientation();
        //limelight setup
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        telemetry.setMsTransmissionInterval(11);
        telemetry.update();
        limelight.pipelineSwitch(3);
        limelight.start();
        //wait for start
        telemetry.addData("here", "here");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {

            //check if there is target and is detected
            telemetry.addData("here", "here");
            telemetry.update();
            LLResult result = limelight.getLatestResult();
            double tx = result.getTx();
            if (result != null && result.isValid()) {
                //check if it is on the right side
                while (tx < -1) { //was 0
                    result = limelight.getLatestResult();
                    tx = result.getTx();
                    telemetry.addData("here", "here");
                    telemetry.update();
                    //move it to the left
                    moveSideways_wCorrection("left", 2, 0.7, 1);
                    result = limelight.getLatestResult();
                    tx = result.getTx();
                }
                if (tx >= -1 && tx <= 1) {
                    sleep(99999);
                }
                result = limelight.getLatestResult();
                tx = result.getTx();
                while (tx > 1){
                    result = limelight.getLatestResult();
                    tx = result.getTx();
                    telemetry.addData("here", "here");
                    telemetry.update();
                    //move it to the left
                    moveSideways_wCorrection("right", 2, 0.7, 1);
                    result = limelight.getLatestResult();
                    tx = result.getTx();
                }
                if (tx >= -1 && tx <= 1) {
                    sleep(99999);
                }
            }
        }
    }
}
