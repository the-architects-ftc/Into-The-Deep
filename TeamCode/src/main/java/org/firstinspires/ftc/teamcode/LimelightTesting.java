package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "LLTEST", group = "Limelight")
public class LimelightTesting extends CommonUtil {
    private Limelight3A limelight;
    double thres = 0.5; //calibrate the limelight to find the perfect ta value for thes bc of new crosshair

    double disin = 0.0;
    int inin = 1;
    int check = 0;
    double ta = 0.0;
    double oldta = 0.0;

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
                while (tx < -1) { //lower for accuracy
                    sidewaysleft(40); //increase for larger movement decrease for less movement
                    result = limelight.getLatestResult();
                    tx = result.getTx();
                }
                if (tx >= -1 && tx <= 1) {
                    check = 1;
                }
                result = limelight.getLatestResult();
                tx = result.getTx();
                while (tx > 1) { //lower for accuracy
                    sidewaysright(40);//increase for larger movement decrease for less movement
                    result = limelight.getLatestResult();
                    tx = result.getTx();
                }
                if (tx >= -1 && tx <= 1) {
                    check = 1;
                }

                //get the robot go to the spot so it can move forward/backward after the left and right movement and then figure out the ta value and change it
                result = limelight.getLatestResult();
                ta = result.getTa();
                while (check == 1) {
                    //backward movement
                    while (ta - thres >= 0.3 && ta - thres <= 0.6) {
                        wierdbackward(25);
                        sleep(100);
                        result = limelight.getLatestResult();
                        ta = result.getTa();
                    }
                    result = limelight.getLatestResult();
                    ta = result.getTa();
                    while (ta - thres <= 0.3) {
                        check = 0;
                    }
                    result = limelight.getLatestResult();
                    ta = result.getTa();
                    while (ta - thres >= 0.6) {
                        wierdbackward(50);
                        sleep(100);
                        result = limelight.getLatestResult();
                        ta = result.getTa();
                    }
                    result = limelight.getLatestResult();
                    ta = result.getTa();
                    while (ta - thres <= 0.3) {
                        check = 0;
                    }
                    result = limelight.getLatestResult();
                    ta = result.getTa();


                    //move forward
                    while (ta - thres >= -0.5 && ta - thres <= -0.01) {
                        wierdforward(25,0);
                        sleep(100);
                        result = limelight.getLatestResult();
                        ta = result.getTa();
                    }
                    result = limelight.getLatestResult();
                    ta = result.getTa();
                    while (ta - thres <= 0.3) {
                        check = 0;
                    }
                    result = limelight.getLatestResult();
                    ta = result.getTa();
                    while (ta - thres <= -0.5) {
                        wierdforward(50,0);
                        sleep(100);
                        result = limelight.getLatestResult();
                        ta = result.getTa();
                    }
                    result = limelight.getLatestResult();
                    ta = result.getTa();
                    while (ta - thres <= 0.3) {
                        check = 0;
                    }
                    result = limelight.getLatestResult();
                    ta = result.getTa();

                }

                telemetry.addData("finished","thingy");
                telemetry.update();
            }
        }
    }
}


