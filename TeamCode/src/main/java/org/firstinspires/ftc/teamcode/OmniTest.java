/* Copyright (c) 2021 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import android.text.Spannable;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorControllerEx;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.Objects;

/*
 * This file contains an example of a Linear "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When a selection is made from the menu, the corresponding OpMode is executed.
 *
 * This particular OpMode illustrates driving a 4-motor Omni-Directional (or Holonomic) robot.
 * This code will work with either a Mecanum-Drive or an X-Drive train.
 * Both of these drives are illustrated at https://gm0.org/en/latest/docs/robot-design/drivetrains/holonomic.html
 * Note that a Mecanum drive must display an X roller-pattern when viewed from above.
 *
 * Also note that it is critical to set the correct rotation direction for each motor.  See details below.
 *
 * Holonomic drives provide the ability for the robot to move in three axes (directions) simultaneously.
 * Each motion axis is controlled by one Joystick axis.
 *
 * 1) Axial:    Driving forward and backward               Left-joystick Forward/Backward
 * 2) Lateral:  Strafing right and left                     Left-joystick Right and Left
 * 3) Yaw:      Rotating Clockwise and counter clockwise    Right-joystick Right and Left
 *
 * This code is written assuming that the right-side motors need to be reversed for the robot to drive forward.
 * When you first test your robot, if it moves backward when you push the left stick forward, then you must flip
 * the direction of all 4 motors (see code below).
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@TeleOp(name="DRIVE", group="Linear OpMode")
//@Disabled
public class OmniTest extends LinearOpMode {

    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor LeftFront = null;
    private DcMotor LeftBack = null;
    private DcMotor RightFront = null;
    private DcMotor RightBack = null;

    private DcMotor intakeMotor = null;

    private DcMotor transfer = null;

    private DcMotorEx launcher = null;


    private Servo ColorServo = null;

    boolean launcherIsAtMaxSpeed = false;
    boolean hasSpunToFarSpeed = false;
    double velocity = 0;
    double closeTargetSpeed = -1300;
    double farTargetSpeed = -1580;

    String lastTrigger = "FAR";


    @Override
    public void runOpMode() {

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        LeftFront = hardwareMap.get(DcMotor.class, "LeftFront");
        LeftBack = hardwareMap.get(DcMotor.class, "LeftBack");
        RightFront = hardwareMap.get(DcMotor.class, "RightFront");
        RightBack = hardwareMap.get(DcMotor.class, "RightBack");

        ColorServo = hardwareMap.get(Servo.class, "Color");

        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        transfer = hardwareMap.get(DcMotor.class, "transfer");
        launcher = hardwareMap.get(DcMotorEx.class, "launcher");

        launcher.setVelocityPIDFCoefficients(30.0, 0, 1.0, 12.0);//P is correction of the motor F is to hold the speed
        //d was 2 before

        LeftFront.setDirection(DcMotor.Direction.FORWARD);
        LeftBack.setDirection(DcMotor.Direction.FORWARD);
        RightFront.setDirection(DcMotor.Direction.REVERSE);
        RightBack.setDirection(DcMotor.Direction.REVERSE);

        launcher.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        // Wait for the game to start (driver presses START)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            double launcherVel = launcher.getVelocity();
            double max;

            //ColorServo.scaleRange(0.277, 0.666);


            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            double axial = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
            double lateral = gamepad1.left_stick_x;
            double yaw = gamepad1.right_stick_x;

            // Combine the joystick requests for each axis-motion to determine each wheel's power.
            // Set up a variable for each drive wheel to save the power level for telemetry.
            double frontLeftPower = axial + lateral + yaw;
            double frontRightPower = axial - lateral - yaw;
            double backLeftPower = axial - lateral + yaw;
            double backRightPower = axial + lateral - yaw;

            // Normalize the values so no wheel power exceeds 100%
            // This ensures that the robot maintains the desired motion.
            max = Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower));
            max = Math.max(max, Math.abs(backLeftPower));
            max = Math.max(max, Math.abs(backRightPower));

            if (max > 1.0) {
                frontLeftPower /= max;
                frontRightPower /= max;
                backLeftPower /= max;
                backRightPower /= max;
            }


            // Send calculated power to wheels
            // Holding A makes the robot move slower so it can park easier
            if (gamepad1.a) {
                LeftFront.setPower(frontLeftPower / 3);
                RightFront.setPower(frontRightPower / 3);
                LeftBack.setPower(backLeftPower / 3);
                RightBack.setPower(backRightPower / 3);
            } else {
                LeftFront.setPower(frontLeftPower);
                RightFront.setPower(frontRightPower);
                LeftBack.setPower(backLeftPower);
                RightBack.setPower(backRightPower);
            }

            //This stuff is for the intake motor
            //this makes the intake motor pull in
            if (gamepad1.left_bumper) {
                intakeMotor.setPower(-1);
            } else {
                intakeMotor.setPower(0);
            }

            // CLOSE
            if (gamepad1.right_trigger > 0) {
                launcher.setVelocity(closeTargetSpeed);

            } else if (gamepad1.left_trigger > 0){//far
                //launcher.setVelocity(farTargetSpeed);
                /*if(!hasSpunToFarSpeed) {
                    while (-launcher.getVelocity() < -farTargetSpeed - 350) {//slowly speed up 10 at a time to reach the target velocity or higher
                        launcher.setVelocity(velocity);
                        velocity -= 5;
                        telemetry.addData("Launcher Velocity: ", launcher.getVelocity());
                        telemetry.addData("Target Velocity: ", velocity);
                        telemetry.update();
                    }
                    hasSpunToFarSpeed = true;
                }
                launcher.setVelocity(farTargetSpeed);

                 */
                launcher.setVelocity(farTargetSpeed);


            } else{
                velocity = 0;
                hasSpunToFarSpeed = false;
                launcher.setPower(0);//power is zero if its not spinning in shooting direction

            }



            /*if(launcher.getVelocity() >= closeTargetSpeed && launcher.getVelocity() != 0){
                ColorServo.scaleRange(0.277, 0.5);
                ColorServo.setPosition(closeTargetSpeed/launcher.getVelocity());
            }else if(launcher.getVelocity() <= closeTargetSpeed){
                ColorServo.scaleRange(0.5, 0.666);
                ColorServo.setPosition(farTargetSpeed/launcher.getVelocity());
            }*/

            telemetry.addData("velocity", launcher.getVelocity());
            telemetry.addData("hasSpunToFarSpeed: ", hasSpunToFarSpeed);
            telemetry.addData("motorPIDF:",launcher.getPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER));
            telemetry.update();

            if (gamepad1.right_bumper) {
                transfer.setPower(-1);
            } else {
                transfer.setPower(0);
            }


            launcherIsAtMaxSpeed = launcherVel <= 1450;

        }
    }
}