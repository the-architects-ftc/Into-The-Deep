// Code written and explained by ChatGPT
// This code provides simple TeleOp control for a mecanum-drive robot.
// It allows movement in all directions (forward, backward, left, right, diagonal, and rotation).

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="Simple_Teleop", group="Exercises") // Marks this as a TeleOp program
public class TeleopOpenSource extends LinearOpMode {
    // Declare the motors for the wheels
    DcMotor bl = null; // Back Left motor
    DcMotor fl = null; // Front Left motor
    DcMotor fr = null; // Front Right motor
    DcMotor br = null; // Back Right motor

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize the wheel motors from the robot configuration
        bl = hardwareMap.get(DcMotor.class, "LB"); // Back Left motor (named "LB" in robot config)
        fl = hardwareMap.get(DcMotor.class, "LF"); // Front Left motor (named "LF" in robot config)
        fr = hardwareMap.get(DcMotor.class, "RF"); // Front Right motor (named "RF" in robot config)
        br = hardwareMap.get(DcMotor.class, "RB"); // Back Right motor (named "RB" in robot config)

        // Set the direction of each motor
        // Reverse the left motors to ensure correct movement direction
        bl.setDirection(DcMotor.Direction.REVERSE);
        fl.setDirection(DcMotor.Direction.REVERSE);
        fr.setDirection(DcMotor.Direction.FORWARD);
        br.setDirection(DcMotor.Direction.FORWARD);

        // Display status on the driver station
        telemetry.addData("Mode", "waiting"); // Shows "waiting" until start button is pressed
        telemetry.update();

        // Wait for the start button to be pressed
        waitForStart();

        while (opModeIsActive()) {
            // Read the joystick values from gamepad 1
            double leftY = -gamepad1.left_stick_y; // Forward/Backward movement (inverted because joystick Y is negative up)
            double leftX = gamepad1.left_stick_x;  // Strafing left/right movement
            double rightX = gamepad1.right_stick_x; // Rotation (turning left/right)

            // Calculate power for each motor using mecanum drive formulas
            double blPower = leftY + leftX + rightX; // Back Left motor power
            double flPower = leftY - leftX + rightX; // Front Left motor power
            double brPower = leftY - leftX - rightX; // Back Right motor power
            double frPower = leftY + leftX - rightX; // Front Right motor power

            // Set the calculated power to each motor
            bl.setPower(blPower);
            fl.setPower(flPower);
            br.setPower(brPower);
            fr.setPower(frPower);

            // Display motor powers for debugging purposes
            telemetry.addData("BL Power", blPower); // Back Left power
            telemetry.addData("FL Power", flPower); // Front Left power
            telemetry.addData("BR Power", brPower); // Back Right power
            telemetry.addData("FR Power", frPower); // Front Right power
            telemetry.update(); // Updates the telemetry on the driver station
        }
    }
}

/*
Robot Configuration Instructions:
1. Open the Robot Configuration on your FTC Driver Station.
2. Add four motors to your configuration:
   - Name them as follows:
     - Back Left motor: "LB"
     - Front Left motor: "LF"
     - Front Right motor: "RF"
     - Back Right motor: "RB"
3. Set the motor directions:
   - Set "LB" and "LF" as REVERSE.
   - Set "RF" and "RB" as FORWARD.
4. Ensure all motors are properly connected to the REV Hub.

This code is designed to work out of the box with the specified motor setup.
Enjoy using this open-source code for your robot!
*/
