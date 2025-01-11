package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
//import java.lang.Math;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


@TeleOp(name="Teleop_slide", group="Exercises")
//@Disabled
public class teleopslide extends LinearOpMode {

    DcMotor m2 = null;
    double m2Power;


    // called when init button is  pressed.
    @Override
    public void runOpMode() throws InterruptedException {

        m2 = hardwareMap.get(DcMotor.class, "M2");
        m2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();
        while (opModeIsActive()) {
            if (gamepad1.a && gamepad1.b) {
                m2Power = 0;
                m2.setPower(m2Power);
            } else if (gamepad1.b) {
                m2Power = 1;
                m2.setPower(m2Power);
            } else if (gamepad1.a) {
                m2Power = -1;
                m2.setPower(m2Power);
            } else {
                m2Power = 0.05;
                m2.setPower(m2Power);
            }


        }
    }
}
