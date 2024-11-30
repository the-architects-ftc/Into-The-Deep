package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;


//reset limelight custom ip and host
//configure control hub name to limelight
//place limelight on something on robot for timebeing

@Autonomous(name = "Limelight_YellowRectangle", group = "Linear Opmode")
public class Limelight_YellowRectangle extends CommonUtil {
    private Limelight3A limelight;
    private static final double ALIGN_THRESHOLD = -14; // Degrees within the center
    private static final double AREA_THRESHOLD = 0.75; // Stop moving when target area >= threshold

    @Override
    public void runOpMode() {
        // Initialize hardware
        initialize(hardwareMap);
        setMotorOrientation();
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        if (limelight == null) {
            telemetry.addData("Error", "Limelight not found in hardwareMap");
            telemetry.update();
        } else {
            telemetry.addData("Success", "Limelight initialized");
            telemetry.update();
        }

        limelight.setPollRateHz(100);
        telemetry.setMsTransmissionInterval(11);
        telemetry.update();
        limelight.pipelineSwitch(3);
        limelight.start();

        waitForStart();

        while (opModeIsActive()) {
            LLResult result = limelight.getLatestResult();
            if (result != null && result.isValid()) {
                double tx = result.getTx();
                double ty = result.getTy();
                double ta = result.getTa();

                telemetry.addData("Target X", tx);
                telemetry.addData("Target Y", ty);
                telemetry.addData("Target Area", ta);
                telemetry.update();


                if (Math.abs(tx - ALIGN_THRESHOLD) > 0.1) {
                    if (tx > 0) {
                        telemetry.addData("Sideways:","Right");
                        telemetry.update();
                        fl.setPower(0.4);
                        fr.setPower(-0.4);
                        bl.setPower(-0.4);
                        br.setPower(0.4);
                        sleep(500);
                        setMotorToZeroPower();

                    } else {
                        telemetry.addData("Sideways:","Left");
                        telemetry.update();
                        fl.setPower(-0.4);
                        fr.setPower(0.4);
                        bl.setPower(0.4);
                        br.setPower(-0.4);
                        sleep(500);
                        setMotorToZeroPower();
                    }
                }

                // Move forward if area is less than the threshold
                if (ta < AREA_THRESHOLD) {
                    telemetry.addData("Forward:","Forward");
                    telemetry.update();
                    fl.setPower(0.3);
                    fr.setPower(0.3);
                    bl.setPower(0.3);
                    br.setPower(0.3);
                    sleep(100);
                    setMotorToZeroPower();
                } else {
                    setMotorToZeroPower();
                    telemetry.addData("Reached Target", "Stopping");
                    telemetry.update();
                    break;
                }
            } else {
                telemetry.addData("Limelight", "No Targets");
                telemetry.update();
                setMotorToZeroPower();
            }
        }

        limelight.stop();
    }
}
