package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.ColorSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import java.util.Arrays;
import java.util.Vector;
import java.util.ArrayList;
import java.util.List;

@Autonomous(name="AutoTest", group="Linear OpMode")

public class AutoTest extends LinearOpMode {

    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor FL = null;
    private DcMotor BL = null;
    private DcMotor FR = null;
    private DcMotor BR = null;

    private DcMotor leftEncoderMotor = null;
    private double leftEncoderPos = 0.0;

    private DcMotor rightEncoderMotor = null;
    private double rightEncoderPos = 0.0;

    private DcMotor centerEncoderMotor = null;
    private double centerEncoderPos = 0;

    private boolean rightStop = false;
    private boolean leftStop = false;

    private int tileMatLength = 12*2;  // Inches

    double localTargetTick;

    // Calculates the circumference for the Odometry pods
    // Divides by 25.4 to change mm to inches
    double OPcircumference = 2.0*Math.PI*(16.0/25.4);

    double curAngle;

    double fieldX = 2*tileMatLength;
    double fieldY = 2*tileMatLength;

    int robotX = 0;
    int robotY = 0;
    double robotTheta = 0;
    double targetTheta;
    double calcTheta;

    YawPitchRollAngles orientation;
    IMU imu;

    @Override
    public void runOpMode() {

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        FL = hardwareMap.get(DcMotor.class, "LeftFront");
        BL = hardwareMap.get(DcMotor.class, "LeftBack");
        FR = hardwareMap.get(DcMotor.class, "RightFront");
        BR = hardwareMap.get(DcMotor.class, "RightBack");

        centerEncoderMotor = hardwareMap.get(DcMotor.class, "LeftBack");

        leftEncoderMotor = hardwareMap.get(DcMotor.class, "LeftFront");
        rightEncoderMotor = hardwareMap.get(DcMotor.class, "RightFront");

        imu = hardwareMap.get(IMU.class, "imu");

        leftEncoderMotor.setDirection(DcMotorSimple.Direction.FORWARD);  // Directions taken from BlackBoxBot.java
        rightEncoderMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        centerEncoderMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.RIGHT;

        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        // Now initialize the IMU with this mounting orientation
        // Note: if you choose two conflicting directions, this initialization will cause a code exception.
        imu.initialize(new IMU.Parameters(orientationOnRobot));
        imu.resetYaw();

        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        imu.resetYaw();
        //curAngle = 0;

        resetTicks();
        setNormalDrive();  // Sets all motors to correct forward/reverse
        telemIMUOrientation(orientation);


        telemetry.addData("Runtime", getRuntime());
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        resetTicks();

        while (opModeIsActive()) {

            /*
            ALL POWER MUST BE NEGATIVE
            Sleep is used when testing, set to 0 for max time

             */
            // (0,0,0), (1,1), (0,1)


            ArrayList<ArrayList<Integer>> coordinates = new ArrayList<ArrayList<Integer>>();

            coordinates.add(0, new ArrayList<Integer>(Arrays.asList(0, tileMatLength, 0)));  // Coordinates are the last two numbers
            coordinates.add(1, new ArrayList<Integer>(Arrays.asList(tileMatLength, tileMatLength, 0))); // (x, y)
            coordinates.add(2, new ArrayList<Integer>(Arrays.asList(0, 0, 0)));
            coordinates.add(3, new ArrayList<Integer>(Arrays.asList(tileMatLength, 0, 0)));
            coordinates.add(4, new ArrayList<Integer>(Arrays.asList(0, tileMatLength, 0)));
            coordinates.add(5, new ArrayList<Integer>(Arrays.asList(0, 0, 0)));

            for(int i=0; i<coordinates.size(); i++){
                robotTheta=wayPoint(coordinates.get(i).get(0), coordinates.get(i).get(1), robotX, robotY, coordinates.get(i).get(2), robotTheta, orientation);
                robotX = coordinates.get(i).get(0);
                robotY = coordinates.get(i).get(1);
            }


            /*
            double distTarget = Math.sqrt((Math.pow(targetX-robotX, 2))+(Math.pow(targetY-robotY,2)));

            targetTheta = Math.atan((targetY-robotY)/(targetX-robotX));
            turnRight(-0.5, Math.toDegrees(targetTheta), orientation, 1);

            localTargetTick = inchesToTicks(distTarget);
            driveForward(localTargetTick, -0.5, 1);

            robotX = tileMatLength;
            robotY = tileMatLength;
            robotTheta = targetTheta;

            targetX = 0;
            targetY = tileMatLength;

            distTarget = Math.sqrt((Math.pow(targetX-robotX, 2))+(Math.pow(targetY-robotY,2)));
            targetTheta = Math.PI - Math.atan((targetY-robotY)/(targetX-robotX));
            turnLeft(-0.5, Math.toDegrees(targetTheta-robotTheta), orientation, 1);

            localTargetTick = inchesToTicks(distTarget);
            driveForward(localTargetTick, -0.5, 1);

             */

            break;
        }
    }

    public double wayPoint(int targetX, int targetY, int robotX, int robotY, int shoot, double robotTheta, YawPitchRollAngles orientation){
        double distTarget = Math.sqrt((Math.pow(targetX-robotX, 2))+(Math.pow(targetY-robotY,2)));

        calcTheta = Math.toDegrees(Math.atan((double)(targetY-robotY)/(double)(targetX-robotX)))-robotTheta;

        if (calcTheta < 0){
            calcTheta += 360;
        }

        telemetry.addData("CalcTheta", calcTheta);
        telemetry.update();
        sleep(2000);

        if (calcTheta <= 90){
            targetTheta = 90-calcTheta;
            turnRight(-0.5, targetTheta, orientation, 1);
        } else if(calcTheta >= 270){
            targetTheta = (360-calcTheta)+90;
            turnRight(-0.5, targetTheta, orientation, 1);
        } else if(calcTheta > 90 && calcTheta <=180){
            targetTheta = calcTheta-90;
            turnLeft(-0.5, targetTheta, orientation, 1);
        } else{
            targetTheta = calcTheta-180;
            turnLeft(-0.5, targetTheta, orientation, 1);
        }

        localTargetTick = inchesToTicks(distTarget);
        driveForward(localTargetTick, -0.5, 1);

        return targetTheta;
    }

    public void driveForward(double targetTicks, double power, long sleep) {
        resetTicks();
        setAllPower(power);
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
        setAllPower(-power);

        telemAllTicks("Backward");

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

    // Turns in one direction but doesn't turn back to back
    public void turnLeft(double power, double targetAngle, YawPitchRollAngles orientation, long sleep){
        double yaw = orientation.getYaw();

        telemetry.addData("Yaw", yaw);
        telemetry.addData("Target", targetAngle);
        telemetry.update();

        setRightPower(power);
        setLeftPower(-power);

        while(yaw < targetAngle) {
            orientation = imu.getRobotYawPitchRollAngles();
            yaw = orientation.getYaw();
            telemetry.addData("Yaw", yaw);
            telemetry.addData("Target", targetAngle);
            telemetry.update();
            //telemIMUOrientation(orientation, yaw);
        }

        stopAllPower();
        resetTicks();
        imu.resetYaw();

        sleep(500*sleep);

        //telemIMUOrientation(orientation, yaw);
    }

    public void turnRight(double power, double targetAngle, YawPitchRollAngles orientation, long sleep){
        double yaw = orientation.getYaw();
        targetAngle = -targetAngle;

        setLeftPower(power);
        setRightPower(-power);

        telemetry.addData("Yaw", yaw);
        telemetry.addData("Target", targetAngle);
        telemetry.update();

        while(yaw > targetAngle){
            orientation = imu.getRobotYawPitchRollAngles();
            yaw = orientation.getYaw();


            telemetry.addData("Yaw", yaw);
            telemetry.addData("Target", targetAngle);
            telemetry.update();
            //telemIMUOrientation(orientation, yaw);
        }

        stopAllPower();
        resetTicks();
        imu.resetYaw();

        sleep(500*sleep);

        //telemIMUOrientation(orientation, yaw);
    }




    public void setNormalDrive(){
        FR.setDirection(DcMotorSimple.Direction.FORWARD);
        FL.setDirection(DcMotorSimple.Direction.REVERSE);
        BR.setDirection(DcMotorSimple.Direction.FORWARD);
        BL.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void setStrafingDrive(){
        FR.setDirection(DcMotorSimple.Direction.REVERSE);
        FL.setDirection(DcMotorSimple.Direction.REVERSE);
        BR.setDirection(DcMotorSimple.Direction.FORWARD);
        BL.setDirection(DcMotorSimple.Direction.FORWARD);
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

    public void resetLeftTicks(){
        leftEncoderPos = leftEncoderMotor.getCurrentPosition();
    }

    public double getLeftTicks(){
        return (-(leftEncoderMotor.getCurrentPosition() - leftEncoderPos));
    }

    public void resetRightTicks(){
        rightEncoderPos = rightEncoderMotor.getCurrentPosition();
    }

    public double getRightTicks(){
        return -(rightEncoderMotor.getCurrentPosition() - rightEncoderPos);
    }

    public void resetCenterTicks(){
        centerEncoderPos = centerEncoderMotor.getCurrentPosition();
    }

    public double getCenterTicks(){
        return (centerEncoderMotor.getCurrentPosition() - centerEncoderPos);
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

    public double rightYawConversion(double yaw){
        if (yaw < 0){
            return (Math.abs(yaw));
        }
        else{
            return (360 - yaw);
        }
    }

    public double leftYawConversion(double yaw){
        if (yaw < 0){
            return (yaw+360);
        }
        else{
            return yaw;
        }
    }

    public double TicksToInches(double ticks){
        double rev = (double)ticks/2000;
        double inches = OPcircumference * rev;
        return inches;
    }
    public double inchesToTicks(double inches) {
        double rev = inches / OPcircumference;
        double tick = 2000 * rev;
        return tick;
    }
}
