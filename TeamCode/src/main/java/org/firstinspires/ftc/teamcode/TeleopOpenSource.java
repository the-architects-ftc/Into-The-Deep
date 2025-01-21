package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="Simple_Teleop", group="Exercises")
public class TeleopOpenSource extends LinearOpMode {
    DcMotor bl = null;
    DcMotor fl = null;
    DcMotor fr = null;
    DcMotor br = null;

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize wheel motors
        bl = hardwareMap.get(DcMotor.class, "LB");
        fl = hardwareMap.get(DcMotor.class, "LF");
        fr = hardwareMap.get(DcMotor.class, "RF");
        br = hardwareMap.get(DcMotor.class, "RB");

        bl.setDirection(DcMotor.Direction.REVERSE);
        fl.setDirection(DcMotor.Direction.REVERSE);
        fr.setDirection(DcMotor.Direction.FORWARD);
        br.setDirection(DcMotor.Direction.FORWARD);

        telemetry.addData("Mode", "waiting");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Read joystick values
            double leftY = -gamepad1.left_stick_y; // Forward/Backward
            double leftX = gamepad1.left_stick_x;  // Strafing
            double rightX = gamepad1.right_stick_x; // Rotation

            // Calculate motor powers
            double blPower = leftY + leftX + rightX;
            double flPower = leftY - leftX + rightX;
            double brPower = leftY - leftX - rightX;
            double frPower = leftY + leftX - rightX;

            // Set motor powers
            bl.setPower(blPower);
            fl.setPower(flPower);
            br.setPower(brPower);
            fr.setPower(frPower);

            // Telemetry for debugging
            telemetry.addData("BL Power", blPower);
            telemetry.addData("FL Power", flPower);
            telemetry.addData("BR Power", brPower);
            telemetry.addData("FR Power", frPower);
            telemetry.update();
        }
    }
}
