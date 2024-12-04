package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;


@Autonomous(name = "LimelightStep4", group = "Limelight")
public class LimelightStep4 extends CommonUtil {
    //define limelight
    private Limelight3A limelight;

    @Override
    public void runOpMode() {
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
                moveSideways_wCorrection("right", 3.6, 0.8, 5);
                moveSideways_wCorrection("right", 3.6, 0.8, 5);
                moveForward_wDistance_wGyro(6.5,0.5,1);
                sleep(99999);
            }
        }

            }
        }


