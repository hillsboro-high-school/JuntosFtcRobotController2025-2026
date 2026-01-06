package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import java.util.ArrayList;
import java.util.List;
import java.util.Vector;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

@Autonomous(name="Q3BlueAuto", group="Linear OpMode")

public class Q3BlueAuto extends LinearOpMode{

    private DcMotor FL = null;
    private DcMotor BL = null;
    private DcMotor FR = null;
    private DcMotor BR = null;

    // Initial Starting Location of the Robot
    private double halfTileMat = 12; // Inches
    double robotX;
    double robotY;
    double robotTheta;

    // Robot Getters
    public double getRobotX(){
        return robotX;
    }
    public double getRobotY(){
        return robotY;
    }
    public double getRobotTheta(){return robotTheta;}

    // Robot Setters
    public void setRobotX(double xCord){
        robotX = xCord;
    }
    public void setRobotY(double yCord){
        robotY = yCord;
    }
    public void setRobotTheta(double theta){robotTheta = theta;}

    double translationVectorAngle;
    double rotError;  // Rotation Error
    double rotBias; // Const for translation

    public double getTranslationVectorAngle(){return translationVectorAngle;}
    public double getRotError(){return rotError;}
    public double getRotBias(){return rotBias;}

    public void setTranslationVectorAngle(double angle) {
        translationVectorAngle = angle;
    }
    public void setRotError(double rotation) {
        rotError = rotation;
    }
    public void setRotBias(double rotationBias){rotBias = rotationBias;}

    // Pinpoint Defined
    GoBildaPinpointDriver pinpoint;

    @Override
    // Init function
    public void runOpMode() {
        FL = hardwareMap.get(DcMotor.class, "LeftFront");
        BL = hardwareMap.get(DcMotor.class, "LeftBack");
        FR = hardwareMap.get(DcMotor.class, "RightFront");
        BR = hardwareMap.get(DcMotor.class, "RightBack");

        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        // Configure the sensor
        configurePinpoint();

        // Set starting location at (0,0)
        setRobotX(0);
        setRobotY(0);

        // Set the location of the robot - this should be the place you are starting the robot from
        pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, getRobotX(), getRobotY(), AngleUnit.DEGREES, 0));

        waitForStart();

        run();
    }

    public void run(){
        while(true){
            pinpoint.update();
            Pose2D pinpointData = pinpoint.getPosition();
            telemPinpoint(pinpointData);
        }
    }

    public void setPower(double translationPID, double rotPID){
        // Make the math calculations constansts in this function
        // Add atan2
        double FL_power = (Math.cos(getTranslationVectorAngle()) + Math.sin(getTranslationVectorAngle()))*translationPID + getRotError() * getRotBias() * rotPID;
        double FR_power;
        double BL_power;
        double BR_power;
    }

    public void configurePinpoint(){
        /*
         *  Set the odometry pod positions relative to the point that you want the position to be measured from.
         *
         *  The X pod offset refers to how far sideways from the tracking point the X (forward) odometry pod is.
         *  Left of the center is a positive number, right of center is a negative number.
         *
         *  The Y pod offset refers to how far forwards from the tracking point the Y (strafe) odometry pod is.
         *  Forward of center is a positive number, backwards is a negative number.
         */
        pinpoint.setOffsets(-125, 95, DistanceUnit.MM); //these are tuned for 3110-0002-0001 Product Insight #1
        // x = 95        y = -125 - previous
        /*
         * Set the kind of pods used by your robot. If you're using goBILDA odometry pods, select either
         * the goBILDA_SWINGARM_POD, or the goBILDA_4_BAR_POD.
         * If you're using another kind of odometry pod, uncomment setEncoderResolution and input the
         * number of ticks per unit of your odometry pod.  For example:
         *     pinpoint.setEncoderResolution(13.26291192, DistanceUnit.MM);
         */
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);

        /*
         * Set the direction that each of the two odometry pods count. The X (forward) pod should
         * increase when you move the robot forward. And the Y (strafe) pod should increase when
         * you move the robot to the left.
         */
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.REVERSED);

        /*
         * Before running the robot, recalibrate the IMU. This needs to happen when the robot is stationary
         * The IMU will automatically calibrate when first powered on, but recalibrating before running
         * the robot is a good idea to ensure that the calibration is "good".
         * resetPosAndIMU will reset the position to 0,0,0 and also recalibrate the IMU.
         * This is recommended before you run your autonomous, as a bad initial calibration can cause
         * an incorrect starting value for x, y, and heading.
         */
        pinpoint.resetPosAndIMU();
    }

    public void telemPinpoint(Pose2D pose2D){
        telemetry.addData("X coordinate (IN)", pose2D.getX(DistanceUnit.INCH));
        telemetry.addData("Y coordinate (IN)", pose2D.getY(DistanceUnit.INCH));
        telemetry.addData("Heading angle (DEGREES)", pose2D.getHeading(AngleUnit.DEGREES));
        telemetry.update();
    }
}
