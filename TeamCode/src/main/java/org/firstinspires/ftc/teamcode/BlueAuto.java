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


@Autonomous(name="BlueAuto", group="Linear OpMode")


public class BlueAuto extends LinearOpMode {

    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor FL = null;
    private DcMotor BL = null;
    private DcMotor FR = null;
    private DcMotor BR = null;
    private DcMotor SM = null;
    private  DcMotor IM = null;

    double FR_power = 0;
    double FL_power = 0;
    double BL_power = 0;
    double BR_power = 0;

    private DcMotor transfer = null;

    private DcMotorEx launcher = null;

    private CRServo assistantServo = null;

    private DcMotor leftEncoderMotor = null;
    private double leftEncoderPos = 0.0;

    private DcMotor rightEncoderMotor = null;
    private double rightEncoderPos = 0.0;

    private DcMotor centerEncoderMotor = null;
    private double centerEncoderPos = 0;

    private boolean rightStop = false;
    private boolean leftStop = false;

    private int tileMatLength = 24;  // Inches
    private int halfeTileMat = 12;  // Inches

    double robotX = 0*halfeTileMat;
    double robotY = 0*halfeTileMat;
    double robotTheta = 0;
    double targetTheta;
    Vector<Double> turnCalculations = new Vector<Double>(0);

    double Kp = 0.7;
    double Ki = 0;
    double Kd = 0.05;

    GoBildaPinpointDriver pinpoint;

    IMU imu;

    @Override
    public void runOpMode() {

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        FL = hardwareMap.get(DcMotor.class, "LeftFront");
        BL = hardwareMap.get(DcMotor.class, "LeftBack");
        FR = hardwareMap.get(DcMotor.class, "RightFront");
        BR = hardwareMap.get(DcMotor.class, "RightBack");

        IM = hardwareMap.get(DcMotor.class, "intakeMotor");
        transfer = hardwareMap.get(DcMotor.class, "transfer");
        launcher = hardwareMap.get(DcMotorEx.class, "launcher");

        assistantServo = hardwareMap.get(CRServo.class, "assistantServo");
        //SM = hardwareMap.get(DcMotor.class, "shootMotor");

        centerEncoderMotor = hardwareMap.get(DcMotor.class, "LeftBack");

        leftEncoderMotor = hardwareMap.get(DcMotor.class, "LeftFront");
        rightEncoderMotor = hardwareMap.get(DcMotor.class, "RightFront");

        imu = hardwareMap.get(IMU.class, "imu");

        leftEncoderMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        rightEncoderMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        centerEncoderMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        // Configure the sensor
        configurePinpoint();

        // Set the location of the robot - this should be the place you are starting the robot from

        pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0));

        resetTicks();
        setNormalDrive();  // Sets all motors to correct forward/reverse

        telemetry.addData("Pinpoint status: ", pinpoint.getDeviceStatus());
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        resetTicks();


        while(opModeIsActive()) {

            /*
            ALL POWER MUST BE NEGATIVE
            Sleep is used when testing, set to 0 for max time
             */


            List<List<Double>> coordinates = new ArrayList<List<Double>>();
            append(coordinates, 1 * halfeTileMat, 0 * halfeTileMat,0);                        // data1 = x
            //append(coordinates, 4 * halfeTileMat, 10 * halfeTileMat, 0);                                      // data2 = y
            //append(coordinates, 5 * halfeTileMat, 7 * halfeTileMat, 1);                                        // data3 = turn amount
            //append(coordinates, halfeTileMat, 5 * halfeTileMat, 0);
            //append(coordinates, 5 * halfeTileMat, 7 * halfeTileMat, 1);

            for(int i=0; i<coordinates.size(); i++){
                turnCalculations = TurnCalc(robotX, robotY, robotTheta, coordinates.get(i).get(0), coordinates.get(i).get(1), coordinates.get(i).get(2));
                pidControl(turnCalculations.get(2), robotX, robotY, coordinates.get(i).get(0), coordinates.get(i).get(1), coordinates.get(i).get(2), robotTheta);
                robotX = coordinates.get(i).get(0); // sets new robotX
                robotY = coordinates.get(i).get(1); // sets new robotY
            }

            // Turn off everything and end auto
            break;
        }
    }

    public Vector<Double> TurnCalc(double robotX, double robotY, double robotTheta, double targetX, double targetY, double targetTheta){
        double distError = Math.sqrt((Math.pow(targetX - robotX, 2)) + (Math.pow(targetY - robotY, 2)));
        double fieldTheta = Math.toDegrees(Math.atan2((targetY-robotY), (targetX-robotX)));
        double deltaTheta = targetTheta-robotTheta;

        Vector<Double> endCalcVector = new Vector<Double>(0);
        endCalcVector.add(deltaTheta);
        endCalcVector.add(fieldTheta);
        endCalcVector.add(distError);
        return endCalcVector;
    }

    public void relativePower(double deltaTheta, double fieldTheta, double c){
        setVarPowerZero();

        setTranslation(Math.toRadians(fieldTheta));
        setRotation(deltaTheta, c);

        equalizePower();
    }

    public void pidControl(double distError, double robotX, double robotY, double targetX, double targetY, double targetTheta, double rT){
        double integralSum = 0;
        double lastError = 0;
        double derivative;
        double out;
        double p;
        double i;
        double d;

        pinpoint.resetPosAndIMU();

        Vector<Double> instTurnCalc = new Vector<Double>(0);
        ElapsedTime timer = new ElapsedTime();

        while(distError > 0.2){
            pinpoint.update();
            Pose2D pose2D = pinpoint.getPosition();

            // X AND Y ARE SWAPPED
            instTurnCalc = TurnCalc(robotX + pose2D.getY(DistanceUnit.INCH), robotY + pose2D.getX(DistanceUnit.INCH), rT+pose2D.getHeading(AngleUnit.DEGREES), targetX, targetY, targetTheta);

            distError = instTurnCalc.get(2);
            derivative = (distError-lastError) / timer.seconds();
            integralSum = integralSum + (distError*timer.seconds());

            relativePower(instTurnCalc.get(0), instTurnCalc.get(1), 0.1);

            p = Kp*distError;
            i = Ki*integralSum;
            d = Kd*derivative;
            out = p+i+d;

            telemetry.addData("X coordinate (IN)", pose2D.getY(DistanceUnit.INCH));
            telemetry.addData("Y coordinate (IN)", pose2D.getX(DistanceUnit.INCH));
            telemetry.addData("Heading angle (DEGREES)", pose2D.getHeading(AngleUnit.DEGREES));
            telemetry.addData("TargetX (IN)", targetX);
            telemetry.addData("TargetY (IN)", targetY);
            telemetry.addData("Target Heading", targetTheta);
            telemetry.addData("DistError", distError);
            telemetry.addData("FieldTheta", instTurnCalc.get(1));
            telemetry.addData("DeltaTheta", instTurnCalc.get(0));
            telemetry.addData("Direction", pose2D.getHeading(AngleUnit.DEGREES)-instTurnCalc.get(1));

            telemetry.addData("FR", FR_power);
            telemetry.addData("FL", FL_power);
            telemetry.addData("BR", BR_power);
            telemetry.addData("BL", BL_power);

            /*
            telemetry.addData("P", p);
            telemetry.addData("I", i);
            telemetry.addData("D", d);
            telemetry.addData("PID", out);
             */
            telemetry.update();

            setPower(out);
            lastError = distError;
            timer.reset();
        }
        stopAllPower();
        sleep(5000);
    }

    public void setPower(double c){
        FR_power *= c;
        FL_power *= c;
        BL_power *= c;
        BR_power *= c;

        equalizePower();

        FR.setPower(FR_power);
        FL.setPower(FL_power);
        BL.setPower(BL_power);
        BR.setPower(BR_power);
    }

    public void setTranslation(double theta){
        FR_power += (Math.sin(theta)-Math.cos(theta));
        FL_power += (Math.sin(theta)+Math.cos(theta));
        BL_power += (Math.sin(theta)-Math.cos(theta));
        BR_power += (Math.sin(theta)+Math.cos(theta));
    }

    public void setRotation(double theta, double c){
        if (Math.abs(theta) >= 2.5){
            FR_power += theta*c;
            FL_power -= theta*c;
            BL_power -= theta*c;
            BR_power += theta*c;
        }
    }

    public void equalizePower(){
        double maxMotor = Math.abs(Math.max(Math.max(Math.abs(FR_power), Math.abs(FL_power)), Math.max(Math.abs(BL_power), Math.abs(BR_power))));
        if (maxMotor > 0.5){
            FR_power /= maxMotor/0.5;
            FL_power /= maxMotor/0.5;
            BL_power /= maxMotor/0.5;
            BR_power /= maxMotor/0.5;
        }
    }


    public void setVarPowerZero(){
        FR_power = 0;
        FL_power = 0;
        BL_power = 0;
        BR_power = 0;
    }

    public double getRobotTheta(){
        pinpoint.update();
        Pose2D pose2D = pinpoint.getPosition();
        return pose2D.getHeading(AngleUnit.DEGREES);
    }

    private List<List<Double>> append(List<List<Double>> start, double data1, double data2, double data3){
        List<Double> newlistpoint = new ArrayList<Double>();

        newlistpoint.add(data1);
        newlistpoint.add(data2);
        newlistpoint.add(data3);
        start.add(newlistpoint);
        return start;

    }
    public void driveForward(double targetTicks, double power, long sleep) {
        resetTicks();
        setAllPower(-power);
        telemAllTicks("Forward");

        while (!(rightStop && leftStop)) {
            if (getRightTicks() >= targetTicks) {
                rightStop = true;
                stopRightPower();
            }
            if (getLeftTicks() >= targetTicks) {
                leftStop = true;
                stopLeftPower();
            }
            telemAllTicks("Forward");
        }

        telemAllTicks("Forward");

        rightStop = false;
        leftStop = false;

        stopAllPower();
        resetTicks();

        sleep(500*sleep);
    }
    public void driveBackward(double targetTicks, double power, long sleep) {
        resetTicks();
        setAllPower(power);

        while (!(rightStop && leftStop)) {
            if (Math.abs(getRightTicks()) >= targetTicks) {
                rightStop = true;
                stopRightPower();
            }
            if (Math.abs(getLeftTicks()) >= targetTicks) {
                leftStop = true;
                stopLeftPower();
            }
            telemAllTicks("Backward");
        }

        telemAllTicks("Backward");

        rightStop = false;
        leftStop = false;
        stopAllPower();
        resetTicks();

        sleep(500*sleep);
    }

    public void strafeRight(double targetTicks, double power, long sleep) {
        setStrafingDrive();
        resetTicks();
        setAllPower(power);

        telemAllTicks("Right");

        while (getCenterTicks() > -targetTicks){
            telemAllTicks("Right");
        }

        telemAllTicks("Right");

        stopAllPower();
        resetTicks();
        setNormalDrive();

        sleep(500*sleep);
    }

    public void strafeLeft(double targetTicks, double power, long sleep) {
        setStrafingDrive();
        resetTicks();
        setAllPower(-power);

        telemAllTicks("Left");

        while (Math.abs(getCenterTicks()) < targetTicks){
            telemAllTicks("Left");
        }

        telemAllTicks("Left");

        stopAllPower();
        resetTicks();
        setNormalDrive();

        sleep(500*sleep);
    }


    public void turnLeft(double power, double targetAngle, YawPitchRollAngles orientation, long sleep){
        double yaw = orientation.getYaw();

        telemetry.addData("Yaw", yaw);
        telemetry.addData("Target", targetAngle);
        telemetry.update();

        if (targetAngle == 180){
            targetTheta -= 0.1;
        }
        if (targetAngle == -180){
            targetTheta += 0.1;
        }

        setRightPower(-power);
        setLeftPower(power);

        while(yaw < targetAngle) {
            orientation = imu.getRobotYawPitchRollAngles();
            yaw = orientation.getYaw();
            telemetry.addData("Yaw", yaw);
            telemetry.addData("Target", targetAngle);
            telemetry.update();
            //telemIMUOrientation(orientation, yaw);
        }

        /*
        if (yaw > targetAngle+2.5 || yaw < targetAngle-2.5) {
            turnRight(-0.15, targetAngle, orientation, 0);
        }

         */

        stopAllPower();
        resetTicks();
        imu.resetYaw();

        sleep(500*sleep);

        //telemIMUOrientation(orientation, yaw);
    }

    public void turnRight(double power, double targetAngle, YawPitchRollAngles orientation, long sleep){
        double yaw = orientation.getYaw();
        targetAngle = -targetAngle;

        telemetry.addData("Yaw", yaw);
        telemetry.addData("Target", targetAngle);
        telemetry.update();

        if (targetAngle == 180){
            targetTheta -= 0.1;
        }
        if (targetAngle == -180){
            targetTheta += 0.1;
        }

        setLeftPower(-power);
        setRightPower(power);

        while(yaw > targetAngle){
            orientation = imu.getRobotYawPitchRollAngles();
            yaw = orientation.getYaw();


            telemetry.addData("Yaw", yaw);
            telemetry.addData("Target", targetAngle);
            telemetry.update();
            //telemIMUOrientation(orientation, yaw);
        }

        /*
        if (yaw < (-targetAngle)-2.5 || yaw > (-targetAngle)+2.5) {
            turnLeft(-0.15, targetAngle, orientation, 0);
        }

         */

        stopAllPower();
        resetTicks();
        imu.resetYaw();

        sleep(500*sleep);

        //telemIMUOrientation(orientation, yaw);
    }


    public void setNormalDrive(){
        FR.setDirection(DcMotorSimple.Direction.REVERSE);
        FL.setDirection(DcMotorSimple.Direction.FORWARD);
        BR.setDirection(DcMotorSimple.Direction.REVERSE);
        BL.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    public void setStrafingDrive(){
        FR.setDirection(DcMotorSimple.Direction.REVERSE);
        FL.setDirection(DcMotorSimple.Direction.REVERSE);
        BR.setDirection(DcMotorSimple.Direction.FORWARD);
        BL.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    public void shoot(){
        intakeOn();
        transfer.setPower(1);
        assistantServo.setPower(1);
        sleep(3000);
        assistantServo.setPower(0);
        transfer.setPower(0);
        intakeOff();
    }

    public void intakeOn(){
        IM.setPower(1);
    }

    public void intakeOff(){
        IM.setPower(0);
    }


    public void resetTicks(){
        resetLeftTicks();
        resetRightTicks();
        resetCenterTicks();
    }

    public void setAllPower(double p){
        FL.setPower(p);
        FR.setPower(p);
        BL.setPower(p);
        BR.setPower(p);
    }

    public void stopAllPower(){
        FL.setPower(0);
        FR.setPower(0);
        BL.setPower(0);
        BR.setPower(0);
    }

    public void setLeftPower(double p){
        FL.setPower(p);
        BL.setPower(p);
    }

    public void setRightPower(double p){
        FR.setPower(p);
        BR.setPower(p);
    }

    public void stopLeftPower(){
        FL.setPower(0);
        BL.setPower(0);
    }

    public void stopRightPower(){
        FR.setPower(0);
        BR.setPower(0);
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

    public void resetLeftTicks(){
        leftEncoderPos = leftEncoderMotor.getCurrentPosition();
    }

    public double getLeftTicks(){
        return (leftEncoderMotor.getCurrentPosition() - leftEncoderPos);
    }

    public void resetRightTicks(){
        rightEncoderPos = rightEncoderMotor.getCurrentPosition();
    }

    public double getRightTicks(){
        return rightEncoderMotor.getCurrentPosition() - rightEncoderPos;
    }

    public void resetCenterTicks(){
        centerEncoderPos = centerEncoderMotor.getCurrentPosition();
    }

    public double getCenterTicks(){
        return (-(centerEncoderMotor.getCurrentPosition() - centerEncoderPos));
    }

    public void telemAllTicks(String dir){
        telemetry.addData("Direction", dir);
        telemetry.addData("Left pos", getLeftTicks());
        telemetry.addData("Right pos", getRightTicks());
        telemetry.addData("Center pos", getCenterTicks());
        telemetry.update();
    }

    public void telemIMUOrientation(YawPitchRollAngles orientation){
        telemetry.addData("Yaw (Z)", "%.2f Deg. (Heading)", orientation.getYaw(AngleUnit.DEGREES));
        telemetry.addData("Pitch (X)", "%.2f Deg.", orientation.getPitch(AngleUnit.DEGREES));
        telemetry.addData("Roll (Y)", "%.2f Deg.\n", orientation.getRoll(AngleUnit.DEGREES));
        telemetry.update();
    }
}
