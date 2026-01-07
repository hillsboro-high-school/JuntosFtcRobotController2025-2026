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
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
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

    // rotation and translation vars
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

    double FL_power;
    double FR_power;
    double BR_power;
    double BL_power;

    public double getFL_power(){return FL_power;}
    public double getFR_power(){return FR_power;}
    public double getBR_power(){return BR_power;}
    public double getBL_power(){return BL_power;}

    public void setFL_power(double power){FL_power = power;}
    public void setFR_power(double power){FR_power = power;}
    public void setBR_power(double power){BR_power = power;}
    public void setBL_power(double power){BL_power = power;}

    public void resetMotorVarPower(){
        setFL_power(0);
        setFR_power(0);
        setBR_power(0);
        setBL_power(0);
    }

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
        // using a local robot x/y for each loop
        double localRobotX = getRobotX();
        double localRobotY = getRobotY();
        double robotVelocity;

        // X and Y are swapped -> EX: (1,0) = (0,1)
        setTargetX(2*halfTileMat);  // 2*halfTileMat = One full tile mat. Cord is based on half tile mats to avoid anything weird
        setTargetY(0);

        //Setting initial errors to seed while loop with good values
        double distError = Math.sqrt(Math.pow((getTargetX()-getRobotX()), 2) + Math.pow((getTargetY()-getRobotY()), 2));
        double distErrorThreshold = 0.2; // Inches
        // Add in rotational error calculation

        while(distError > distErrorThreshold) { //Check for completion condition in translation and rotation
            //perform distance/angle error calculation
            distError = Math.sqrt(Math.pow((getTargetX()-getRobotX()), 2) + Math.pow((getTargetY()-getRobotY()), 2));

            //Perform field vector calculation
            // Test by hard coding angles and seeing which way the robot moves
            setTranslationVectorAngle(Math.atan2((getTargetX()-getRobotX()), (getTargetY()-getRobotY())));//takes y,x and returns angle in radians

            //Calculate Translation and Rotational PID terms
            //Rolling average controls how much influence the most recent calculation has over the value.
            // A Small rolling average gives it low influence. Needs to be between 0 and 1
            //P=Previous P value * (1-P Rolling Ave) + distError * P Rolling Ave
            //D=Previous D value * (1-D Rolling Ave) + Velocity/distError * D Rolling Ave
            //I=If Velocity < threshold -> Previous I + 1/abs(velocity)

            // PID= Pa*P-Da*D+Ia*I
            // Will need to set bounds to make sure this does not go below a threshold like .3

            setRelativePower(1, 1); //This function calculates relative motor powers using filed vector and rotational error

            normalizePower(); //This function normalizes motor power to avoid any power being >1

            setPower();  //Set motor powers

            //Update pinpoint and query robot current position and velocity
            pinpoint.update();
            Pose2D robotCurPos = pinpoint.getPosition();
            robotVelocity = pinpoint.getHeadingVelocity(UnnormalizedAngleUnit.DEGREES);

            //Update robot current position
            setRobotX(localRobotX + robotCurPos.getX(DistanceUnit.INCH));
            setRobotY(localRobotY + robotCurPos.getY(DistanceUnit.INCH));

            //Output critical values to track robot performance
            grandTelemetryFunction(distError, robotCurPos, robotVelocity);

        }
    }

    public void setRelativePower(double translationPID, double rotPID){
        //The fxns with "translationsAngle..." are just get and set angle
        // Math calculations for easier reading
        double cosVal = (Math.cos(getTranslationVectorAngle())) / Math.sqrt(2);
        double sinVal = Math.sin(getTranslationVectorAngle());

        // multiples rotation error, rotation bias, and rotations pid for easier reading
        // setting rot error and bias to zero so it isn't in effect
        setRotError(0);
        setRotBias(0); //Needs to be a constant > 0
        double rotationControl = getRotError() * getRotBias() * rotPID; //This calculates final rotational power

        // Calculates power needed for each motor to get to the relative position
        setFL_power((cosVal + sinVal)*translationPID + rotationControl);
        setFR_power((-cosVal - sinVal)*translationPID + rotationControl);
        setBL_power ((cosVal - sinVal)*translationPID + rotationControl);
        setBR_power ((-cosVal + sinVal)*translationPID + rotationControl);
    }

    public void normalizePower(){
        double maxMotorThreshold = 0.5; // 50% motor power
        double maxMotor = Math.abs(Math.max(Math.max(Math.abs(getFR_power()), Math.abs(getFL_power())), Math.max(Math.abs(getBL_power()), Math.abs(getBR_power()))));
        if (maxMotor > maxMotorThreshold){
            setFL_power(getFL_power() / maxMotor/0.5);
            setFR_power(getFR_power() / maxMotor/0.5);
            setBR_power(getBR_power() / maxMotor/0.5);
            setBL_power(getBL_power() / maxMotor/0.5);
        }
    }

    public void setPower(){
        FL.setPower(getFL_power());
        FR.setPower(getFR_power());
        BL.setPower(getBL_power());
        BR.setPower(getBR_power());
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

    // Writes out ALL data onto screen
    public void grandTelemetryFunction(double distError, Pose2D pose2D, double robotVel){
        telemetry.addData("X coordinate (IN)", pose2D.getX(DistanceUnit.INCH));
        telemetry.addData("Y coordinate (IN)", pose2D.getY(DistanceUnit.INCH));
        telemetry.addData("Heading angle (DEGREES)", pose2D.getHeading(AngleUnit.DEGREES));

        telemetry.addData("Robot Velocity", robotVel);
        telemetry.addData("TargetX", getTargetX());
        telemetry.addData("TargetY", getTargetY());
        telemetry.addData("RobotX", getRobotX());
        telemetry.addData("RobotY", getRobotY());
        telemetry.addData("Dist error", distError);

        telemetry.addData("FR_Power", getFR_power());
        telemetry.addData("FL_Power", getFL_power());
        telemetry.addData("BL_Power", getBL_power());
        telemetry.addData("BR_Power", getBR_power());
        telemetry.update();
    }

}
