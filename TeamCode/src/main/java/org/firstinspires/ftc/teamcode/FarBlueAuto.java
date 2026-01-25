package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;

import java.util.ArrayList;
import java.util.List;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

@Autonomous(name="FarBlueAuto", group="Linear OpMode")

public class FarBlueAuto extends LinearOpMode{

    private DcMotor FL = null;
    private DcMotor BL = null;
    private DcMotor FR = null;
    private DcMotor BR = null;
    private DcMotor INTAKE = null;
    private DcMotor TRANSFER = null;
    private DcMotorEx LAUNCHER = null;


    // Initial Starting Location of the Robot
    private final double halfTileMat = 12; // Inches
    double robotX;
    double robotY;
    double robotTheta;
    double startX;
    double startY;
    double startTheta;

    // Robot Getters
    public double getRobotX(){
        return robotX;
    }
    public double getRobotY(){
        return robotY;
    }
    public double getRobotTheta(){return robotTheta;}
    public double getStartX(){return startX;}
    public double getStartY(){return startY;}
    public double getStartTheta(){return startTheta;}


    // Robot Setters
    public void setRobotX(double xCord){
        robotX = xCord;
    }
    public void setRobotY(double yCord){
        robotY = yCord;
    }
    public void setRobotTheta(double theta){robotTheta = theta;}
    public void setStartX(double newStartX){startX = newStartX;}
    public void setStartY(double newStartY){startY = newStartY;}
    public void setStartTheta(double newStartTheta){startTheta = newStartTheta;}



    // rotation and translation vars
    double translationVectorAngle;
    double distError;
    double rotError;  // Rotation Error
    double rotBias; // Const for translation

    public double getTranslationVectorAngle(){return translationVectorAngle;}
    public double getRotError(){return rotError;}
    public double getRotBias(){return rotBias;}
    public double getDistError(){return distError;}


    public void setTranslationVectorAngle(double angle) {translationVectorAngle = angle;} // Subtracting PI/2 gives unit circle measurements
    public void setRotError(double rotation) {
        rotError = rotation;
    }
    public void setRotBias(double rotationBias){rotBias = rotationBias;}
    public void setDistError(double distance) {
        distError = distance;
    }


    // Target Vars
    double targetX;
    double targetY;
    double targetTheta;

    public double getTargetX(){return targetX;}
    public double getTargetY(){return targetY;}
    public double getTargetTheta(){return targetTheta;}
    public void setTargetX(double newTargetX){targetX = newTargetX;}
    public void setTargetY(double newTargetY){targetY = newTargetY;}
    public void setTargetTheta(double newTheta){targetTheta = newTheta;}

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

    double launcherVelocity = -1590;  // is -1660 in TeleOp


    @Override
    // Init function
    public void runOpMode() {
        // Map Wheel Motors from the driver station
        FL = hardwareMap.get(DcMotor.class, "LeftFront");
        BL = hardwareMap.get(DcMotor.class, "LeftBack");
        FR = hardwareMap.get(DcMotor.class, "RightFront");
        BR = hardwareMap.get(DcMotor.class, "RightBack");

        // Add brakes to each wheel motor
        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Map Misc
        LAUNCHER = hardwareMap.get(DcMotorEx.class, "launcher");
        INTAKE = hardwareMap.get(DcMotor.class, "intakeMotor");
        TRANSFER = hardwareMap.get(DcMotor.class, "transfer");

        LAUNCHER.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LAUNCHER.setVelocityPIDFCoefficients(28.0, 0, 1.0, 12.0);//P is correction of the motor F is to hold the speed


        // Map pinpoint
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        // Configure the sensor
        configurePinpoint();

        // Set startX, startY, startTheta to use for reference later
        setStartX(0.5*halfTileMat);
        setStartY(4 * halfTileMat);
        setStartTheta(0); // Has to be zero

        // X and Y are switched: (1,0) = (0,1)
        setRobotX(getStartX());
        setRobotY(getStartY());
        setRobotTheta(getStartTheta());

        // Set the location of the robot - this should be the place you are starting the robot from
        pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, getRobotX(), getRobotY(), AngleUnit.DEGREES, getRobotTheta()));

        telemetry.addData("robotX", getRobotX());
        telemetry.addData("robotY", getRobotY());
        telemetry.addData("robotTheta", getRobotTheta());
        telemetry.addData("STATUS:", pinpoint.getDeviceStatus());
        telemetry.update();

        // Wait for user to press run on DS
        waitForStart();

        // Calls main run fxn
        run();
        LAUNCHER.setVelocity(-launcherVelocity);
    }

    public void run(){

        // Create a list of all coords you want the robot to move to during auto
        // shoot -> 1 = YES 0 = NO
        List<List<Double>> coordinates = new ArrayList<List<Double>>();
        append(coordinates, halfTileMat,5 * halfTileMat,25, 1);
        append(coordinates, 2.5 * halfTileMat, 4 * halfTileMat,90, 0);
        append(coordinates, 2.5 * halfTileMat, halfTileMat,90, 0);
        append(coordinates, halfTileMat, 5 * halfTileMat,25, 1);
        append(coordinates,  halfTileMat, halfTileMat,90, 0);
        append(coordinates, halfTileMat, 5 * halfTileMat,25, 1);
        append(coordinates, 3 * halfTileMat, 5 * halfTileMat,90, 0);

        for(int loop=0; loop<coordinates.size(); loop++) {

            // using a local robot x/y for each loop
            double robotVelocity = 0; // could make global

            // X and Y are swapped -> EX: (1,0) = (0,1)
            setTargetX(coordinates.get(loop).get(0));  // 2*halfTileMat = One full tile mat. Cord is based on half tile mats to avoid anything weird
            setTargetY(-coordinates.get(loop).get(1));
            setTargetTheta(coordinates.get(loop).get(2));

            // PID Vars
            /*
            double p = 1;
            double i = 0;
            double d = 1;

            double Kp = 1.3;
            double Ki = 0.2;
            double Kd = 0.9;

             */


            //Setting initial errors to seed while loop with good values
            setDistError(Math.sqrt(Math.pow((getTargetX() - getRobotX()), 2) + Math.pow((getTargetY() - getRobotY()), 2)));
            setRotError(getRobotTheta()-getTargetTheta());

            double distErrorThreshold = 0.5; // Inches
            double rotErrorThreshold = 1; // Degrees

            INTAKE.setPower(-1);
            LAUNCHER.setVelocity(launcherVelocity);


            while (getDistError() > distErrorThreshold || Math.abs(getRotError()) > rotErrorThreshold) { //Check for completion condition in translation and rotation
                //perform distance/angle error calculation
                setDistError(Math.sqrt(Math.pow((getTargetX() - getRobotX()), 2) + Math.pow((getTargetY() - getRobotY()), 2)));
                setRotError(getRobotTheta()-getTargetTheta());

                //Perform field vector calculation
                // Test by hard coding angles and seeing which way the robot moves
                double aTanVal = Math.atan2((getTargetY()- getRobotY()), (getTargetX() - getRobotX()));
                setTranslationVectorAngle(aTanVal - Math.toRadians(getRobotTheta())); //takes y,x and returns angle in radians
                //getRobot

                //Calculate Translation and Rotational PID terms
                //Rolling average controls how much influence the most recent calculation has over the value.
                // A Small rolling average gives it low influence. Needs to be between 0 and 1

                // Uncomment once rotation is working theta is changing as the robot moves towards the object
                /*
                double rollingAvgP = 0.9;
                double rollingAvgD = 0.5;
                double thresholdI = 0.2;

                // Keep the PID above threshold to avoid a zero
                double thresholdPID = 0.3;

                p = p * (1-rollingAvgP) + getDistError() * rollingAvgP;
                d = d * (1-rollingAvgD) + robotVelocity/getDistError() * rollingAvgD;
                if (robotVelocity < thresholdI){i = i + 1/Math.abs(robotVelocity);}

                double translationPID = Kp*p-Kd*d+Ki*i;
                if (translationPID < thresholdPID){translationPID = thresholdPID;}  // Keeps the PID above threshold to avoid a zero

                 */

                setRelativePower(1, 1); //This function calculates relative motor powers using filed vector and rotational error

                normalizePower(); //This function normalizes motor power to avoid any power being >1

                setPower();  //Set motor powers (setting to zero can help debug, use setPowerToZero fxn)

                //Update pinpoint and query robot current position and velocity
                pinpoint.update();
                Pose2D robotCurPos = pinpoint.getPosition();
                robotVelocity = pinpoint.getHeadingVelocity(UnnormalizedAngleUnit.DEGREES);

                //Update robot current position
                setRobotX(robotCurPos.getX(DistanceUnit.INCH));
                setRobotY(robotCurPos.getY(DistanceUnit.INCH));
                setRobotTheta(robotCurPos.getHeading(AngleUnit.DEGREES));

                //Output critical values to track robot performance
                grandTelemetryFunction(getDistError(), aTanVal, robotCurPos, robotVelocity);

                if(getDistError() < 1){
                    TRANSFER.setPower(-0.5);
                }else{
                    TRANSFER.setPower(0);
                }

            }
            resetMotorVarPower();
            setPowerToZero();
            shoot(coordinates.get(loop).get(3));
            TRANSFER.setPower(0);
        }
        INTAKE.setPower(0);
        LAUNCHER.setVelocity(-launcherVelocity);
        LAUNCHER.setPower(0);
    }

    public void setRelativePower(double translationPID, double rotPID){
        //The fxns with "translationsAngle..." are just get and set angle
        // Math calculations for easier reading
        //double testAngle = 0;
        double cosVal = Math.cos(getTranslationVectorAngle()) / Math.sqrt(2);
        double sinVal = Math.sin(getTranslationVectorAngle());

        // multiples rotation error, rotation bias, and rotations pid for easier reading
        // setting rot error and bias to zero so it isn't in effect

        setRotBias(0.1); //Needs to be a constant > 0
        double rotationControl = getRotError() * getRotBias() * rotPID; //This calculates final rotational power

        telemetry.addData("Rotation Control", rotationControl);

        // Calculates power needed for each motor to get to the relative position
        setFL_power((cosVal - sinVal)*translationPID + rotationControl);
        setFR_power((-cosVal - sinVal)*translationPID + rotationControl);
        setBL_power ((cosVal + sinVal)*translationPID + rotationControl);
        setBR_power ((-cosVal + sinVal)*translationPID + rotationControl);
    }

    public void normalizePower(){
        double maxMotorThreshold = 0.75; // 50% motor power
        double maxMotor = Math.abs(Math.max(Math.max(Math.abs(getFR_power()), Math.abs(getFL_power())), Math.max(Math.abs(getBL_power()), Math.abs(getBR_power()))));
        if (maxMotor > maxMotorThreshold){
            setFL_power(getFL_power() / (maxMotor/maxMotorThreshold));
            setFR_power(getFR_power() / (maxMotor/maxMotorThreshold));
            setBR_power(getBR_power() / (maxMotor/maxMotorThreshold));
            setBL_power(getBL_power() / (maxMotor/maxMotorThreshold));
        }
    }

    public void setPower(){
        FL.setPower(getFL_power());
        FR.setPower(getFR_power());
        BL.setPower(getBL_power());
        BR.setPower(getBR_power());
    }

    public void setPowerToZero(){
        FL.setPower(0);
        FR.setPower(0);
        BL.setPower(0);
        BR.setPower(0);
    }

    public void shoot(double shoot){
        double endTime = getRuntime() + 5.5;
        if (shoot == 1){
            LAUNCHER.setVelocity(launcherVelocity);
            while(getRuntime() < endTime) {
                telemetry.addData("Launcher VEL", LAUNCHER.getVelocity());
                telemetry.update();

                if (LAUNCHER.getVelocity()-10 <= launcherVelocity) {
                    TRANSFER.setPower(-1);
                } else if (LAUNCHER.getVelocity() >= 0.9*launcherVelocity) {
                    TRANSFER.setPower(0);
                }

            }
        }
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
        pinpoint.setOffsets(-95, 5, DistanceUnit.MM); //these are tuned for 3110-0002-0001 Product Insight #1

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

    private List<List<Double>> append(List<List<Double>> start, double xCord, double yCord, double theta, double shoot){
        List<Double> newlistpoint = new ArrayList<Double>();

        newlistpoint.add(xCord-getStartX());
        newlistpoint.add(yCord-getStartY());
        newlistpoint.add(theta);
        newlistpoint.add(shoot);
        start.add(newlistpoint);
        return start;
    }


    // Writes out ALL data onto screen
    public void grandTelemetryFunction(double distError, double aTanVal, Pose2D pose2D, double robotVel){
        telemetry.addData("Starting X", getStartX());
        telemetry.addData("Starting Y", getStartY());
        telemetry.addData("Starting Theta", getStartTheta());

        telemetry.addData("X coordinate (IN)", pose2D.getX(DistanceUnit.INCH));
        telemetry.addData("Y coordinate (IN)", pose2D.getY(DistanceUnit.INCH));
        telemetry.addData("Heading angle (DEGREES)", pose2D.getHeading(AngleUnit.DEGREES));
        telemetry.addData("Robot Velocity", robotVel);

        telemetry.addData("TargetX", getTargetX());
        telemetry.addData("TargetY", getTargetY());
        telemetry.addData("TargetTheta", getTargetTheta());
        telemetry.addData("rotError", getRotError());
        telemetry.addData("RobotX", getRobotX());
        telemetry.addData("RobotY", getRobotY());
        telemetry.addData("RobotTheta", getRobotTheta());
        telemetry.addData("Dist error", distError);
        telemetry.addData("arcTangent Value", Math.toDegrees(aTanVal));
        telemetry.addData("Translation angle", Math.toDegrees(getTranslationVectorAngle()));

        telemetry.addData("Launcher VEL", LAUNCHER.getVelocity());
        /*
        telemetry.addData("FR_Power", getFR_power());
        telemetry.addData("FL_Power", getFL_power());
        telemetry.addData("BL_Power", getBL_power());
        telemetry.addData("BR_Power", getBR_power());

         */
        telemetry.update();
    }

}

