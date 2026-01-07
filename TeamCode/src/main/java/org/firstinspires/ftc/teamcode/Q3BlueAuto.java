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
import org.opencv.core.Mat;

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

    // Target Vars
    double targetX;
    double targetY;

    public double getTargetX(){return targetX;}
    public double getTargetY(){return targetY;}
    public void setTargetX(double newTargetX){targetX = newTargetX;}
    public void setTargetY(double newTargetY){targetY = newTargetY;}


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
        double localRobotX = getRobotX();
        double localRobotY = getRobotY();
        setTargetX(2*halfTileMat);  // 2*halfTileMat = One full tile mat. Cord is based on half tile mats to avoid anything weird
        setTargetY(2*halfTileMat);

        double distError = Math.sqrt(Math.pow((getTargetX()-localRobotX), 2) + Math.pow((getTargetY()-localRobotY), 2));

        while(distError > 0.2) {
            setPower(1, 1);

            pinpoint.update();
            Pose2D robotCurPos = pinpoint.getPosition();
            telemPinpoint(robotCurPos);
            telemetry.addData("TargetX", getTargetX());
            telemetry.addData("TargetY", getTargetY());
            telemetry.addData("RobotX", getRobotX());
            telemetry.addData("RobotY", getRobotY());

            setRobotX(localRobotX + pinpoint.getPosX(DistanceUnit.INCH));
            setRobotY(localRobotY + pinpoint.getPosY(DistanceUnit.INCH));
            distError = Math.sqrt(Math.pow((getTargetX()-getRobotX()), 2) + Math.pow((getTargetY()-getRobotY()), 2));
            telemetry.addData("Dist error", distError);
        }
    }

    public void setPower(double translationPID, double rotPID){
        // Math calculations for easier reading
        setTranslationVectorAngle(Math.atan2((getTargetX()-getRobotX()), (getTargetY()-getRobotY())));
        double cosVal = (Math.cos(getTranslationVectorAngle())) / Math.sqrt(2);
        double sinVal = Math.sin(getTranslationVectorAngle());

        // multiples rotation error, rotation bias, and rotations pid for easier reading
        // setting rot error and bias to zero so it isn't in effect
        setRotError(0);
        setRotBias(0);
        double rotationControl = getRotError() * getRotBias() * rotPID;

        // Calculates power needed for each motor to get to the relative position
        double FL_power = (cosVal + sinVal)*translationPID + rotationControl;
        double FR_power = (-cosVal - sinVal)*translationPID + rotationControl;
        double BL_power = (cosVal - sinVal)*translationPID + rotationControl;
        double BR_power = (-cosVal + sinVal)*translationPID + rotationControl;

        // find max motor and checks if its greater than 100% power and if so equalizes everything to be under 100% (50% rn to test things)
        double maxMotor = Math.abs(Math.max(Math.max(Math.abs(FR_power), Math.abs(FL_power)), Math.max(Math.abs(BL_power), Math.abs(BR_power))));
        if (maxMotor > 0.5){
            FL_power /= maxMotor/0.5;
            FR_power /= maxMotor/0.5;
            BR_power /= maxMotor/0.5;
            BL_power /= maxMotor/0.5;
        }


        telemetry.addData("Translation Vector Angle", Math.toDegrees(getTranslationVectorAngle()));
        telemetry.addData("FL Power", FL_power);
        telemetry.addData("FR Power", FR_power);
        telemetry.addData("BR Power", BR_power);
        telemetry.addData("BL Power", BL_power);
        telemetry.update();

        // sets the calculated power to all the motors
        FL.setPower(FL_power);
        FR.setPower(FR_power);
        BR.setPower(BR_power);
        BL.setPower(BL_power);
    }

    // Configure settings for Pinpoint
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
                GoBildaPinpointDriver.EncoderDirection.FORWARD);

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

    // Writes out pinpoint data onto screen
    public void telemPinpoint(Pose2D pose2D){
        telemetry.addData("X coordinate (IN)", pose2D.getX(DistanceUnit.INCH));
        telemetry.addData("Y coordinate (IN)", pose2D.getY(DistanceUnit.INCH));
        telemetry.addData("Heading angle (DEGREES)", pose2D.getHeading(AngleUnit.DEGREES));
        //telemetry.update();
    }

}
