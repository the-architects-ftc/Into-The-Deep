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
    private static final double ALIGN_THRESHOLD = 2.0; // Degrees within the center
    private static final double AREA_THRESHOLD = 5.0; // Stop moving when target area >= threshold
    private static final double TX_CORRECTION_POWER = 0.2; // Power for horizontal alignment
    private static final double FORWARD_POWER = 0.3; // Power for moving forward

    @Override
    public void runOpMode() {
        // Initialize hardware
        initialize(hardwareMap);
        setMotorOrientation();
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        telemetry.setMsTransmissionInterval(11);
        telemetry.update();
        limelight.pipelineSwitch(1);
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

                // Align horizontally (adjust strafing)
                if (Math.abs(tx) > ALIGN_THRESHOLD) {
                    if (tx > 0) {
                        moveSideways_wCorrection("right", 1, TX_CORRECTION_POWER, 1); // Strafe right
                    } else {
                        moveSideways_wCorrection("left", 1, TX_CORRECTION_POWER, 1); // Strafe left
                    }
                }

                // Move forward if area is less than the threshold
                if (ta < AREA_THRESHOLD) {
                    moveForward_wDistance_wGyro(2, FORWARD_POWER, 1); // Move forward
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
