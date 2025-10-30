/* Copyright (c) 2023 FIRST. All rights reserved.
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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.Vector;

/*
 * This OpMode illustrates the basics of AprilTag recognition and pose estimation, using
 * the easy way.
 *
 * For an introduction to AprilTags, see the FTC-DOCS link below:
 * https://ftc-docs.firstinspires.org/en/latest/apriltag/vision_portal/apriltag_intro/apriltag-intro.html
 *
 * In this sample, any visible tag ID will be detected and displayed, but only tags that are included in the default
 * "TagLibrary" will have their position and orientation information displayed.  This default TagLibrary contains
 * the current Season's AprilTags and a small set of "test Tags" in the high number range.
 *
 * When an AprilTag in the TagLibrary is detected, the SDK provides location and orientation of the tag, relative to the camera.
 * This information is provided in the "ftcPose" member of the returned "detection", and is explained in the ftc-docs page linked below.
 * https://ftc-docs.firstinspires.org/apriltag-detection-values
 *
 * To experiment with using AprilTags to navigate, try out these two driving samples:
 * RobotAutoDriveToAprilTagOmni and RobotAutoDriveToAprilTagTank
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list.
 */
@TeleOp(name = "Concept: AprilTag Easy", group = "Concept")

public class CameraGimTestScript extends LinearOpMode {

    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera
    private Servo XServo = null;
    private Servo YServo = null;

    private Vector<Double> previous = new Vector<>(3);

    private Vector<Double> current = new Vector<>(3);
    private int ScanPos = 0;

    private boolean TagFound = false;
    private double ServoPos = 0;

    private int CurrentMotifID = 0;




    /**
     * The variable to store our instance of the AprilTag processor.
     */
    private AprilTagProcessor aprilTag;

    /**
     * The variable to store our instance of the vision portal.
     */
    private VisionPortal visionPortal;

    @Override
    public void runOpMode() {

        StartVector(previous, 0, 0, 0);

        StartVector(current, 0, 0, 0);
//
 //        XServo = hardwareMap.get(Servo.class, "x_servo");
//        YServo = hardwareMap.get(Servo.class, "y_servo");
//
        initAprilTag();

        // Wait for the DS start button to be touched.
        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch START to start OpMode");

        telemetry.update();




        waitForStart();

        if (opModeIsActive()) {
            while (opModeIsActive()) {

                telemetryAprilTag();

                // Push telemetry to the Driver Station.
                telemetry.update();

                // Save CPU resources; can resume streaming when needed.
                if (gamepad1.dpad_down) {
                    visionPortal.stopStreaming();
                } else if (gamepad1.dpad_up) {
                    visionPortal.resumeStreaming();
                }


 //               if(gamepad1.a){
 //                   XServo.setPosition(0);
 //               }
 //               if(gamepad1.b){
 //                   XServo.setPosition(1);
  //              }
                /*if(TagFound){
                    double DeltaX = 0 - (double)current.get(0);
                    double DeltaY = (double)current.get(1);

                    double DeltaA = Math.atan(DeltaX/DeltaY);
                    if(DeltaX < 0){
                        DeltaA += Math.PI;
                    }

                    double ServoA = DeltaA / (2*Math.PI);

                    ServoPos = XServo.getPosition();

                    if(ServoPos + ServoA > 1){
                        ServoPos -= 1;
                    }else if(ServoPos + ServoA < 0){
                        ServoPos += 1;
                    }

                    XServo.setPosition(ServoPos + ServoA);

                    previous = current;
                }else{
                    XServo.setPosition(ScanPos);
                    if(XServo.getPosition() == 0){
                        ScanPos = 1;
                    }else if(XServo.getPosition() == 1){
                        ScanPos = 0;
                    }
                }*/


                // Share the CPU.
                sleep(20);
            }
        }

        // Save more CPU resources when camera is no longer needed.
        visionPortal.close();

    }   // end method runOpMode()

    /**
     * Initialize the AprilTag processor.
     */
    private void initAprilTag() {

        // Create the AprilTag processor the easy way.
        aprilTag = AprilTagProcessor.easyCreateWithDefaults();

        // Create the vision portal the easy way.
        if (USE_WEBCAM) {
            visionPortal = VisionPortal.easyCreateWithDefaults(
                    hardwareMap.get(WebcamName.class, "Webcam 1"), aprilTag);
        } else {
            visionPortal = VisionPortal.easyCreateWithDefaults(
                    BuiltinCameraDirection.BACK, aprilTag);
        }

    }   // end method initAprilTag()

    /**
     * Add telemetry about AprilTag detections.
     */
    private void telemetryAprilTag() {

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());

        TagFound = !currentDetections.isEmpty();

        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {

                SetVector(current, detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z);
                CurrentMotifID = detection.id;
                telemetry.addLine(String.format("\n==== (ID %d) %s", CurrentMotifID, detection.metadata.name));
                telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
            } else {
                telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
            }
        }   // end for() loop

        // Tell Driver the current Motif
            telemetry.addLine("Current Motif:");
            if (CurrentMotifID == 21) {
                telemetry.addLine("Green Purple Purple");
            } else if (CurrentMotifID == 22) {
                telemetry.addLine("Purple Green Purple");
            } else if (CurrentMotifID == 23) {
                telemetry.addLine("Purple Purple Green");
            } else {
                telemetry.addLine("Unknown");
            }


    // Add "key" information to telemetry
        telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
        telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
        telemetry.addLine("RBE = Range, Bearing & Elevation");

    }   // end method telemetryAprilTag()


    public void SetVector(Vector vector, double X, double Y, double Z)
    {
        vector.set(0, X);
        vector.set(1, Y);
        vector.set(2, Z);
    }

    public void StartVector(Vector vector, double X, double Y, double Z){
        vector.add(X);
        vector.add(Y);
        vector.add(Z);
    }
}   // end class
