package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import java.util.ArrayList;
import java.util.List;
import java.util.Vector;

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

    private DcMotor storage = null;

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


    double localTargetTick;

    // Calculates the circumference for the Odometry pods
    // Divides by 25.4 to change mm to inches
    double OPcircumference = 2.0*Math.PI*(24.0/25.4);

    double aprilTagX = halfeTileMat;
    double aprilTagY = 11*halfeTileMat;

    double robotX = 6*halfeTileMat;
    double robotY = 2*halfeTileMat;
    double robotTheta = 0;
    double targetTheta;
    double calcTheta;
    Vector<Double> turnCalculations = new Vector<Double>(0);

    double Kp = 2;
    double Ki = 0.5;
    double Kd = 0.5;

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

        IM = hardwareMap.get(DcMotor.class, "intakeMotor");
        storage = hardwareMap.get(DcMotor.class, "storage");
        launcher = hardwareMap.get(DcMotorEx.class, "launcher");

        assistantServo = hardwareMap.get(CRServo.class, "assistantServo");
        //SM = hardwareMap.get(DcMotor.class, "shootMotor");

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

            List<List<Double>> coordinates = new ArrayList<List<Double>>();
            append(coordinates, 8 * halfeTileMat, 8 * halfeTileMat,  1);  // data1 = x
            //append(coordinates, halfeTileMat, 7 * halfeTileMat, 0);             // data2 = y
            //append(coordinates, 5 * halfeTileMat, 7 * halfeTileMat, 1);   // data3 = shoot
            //append(coordinates, halfeTileMat, 5 * halfeTileMat, 0);             // 0 means don't shoot
            //append(coordinates, 5 * halfeTileMat, 7 * halfeTileMat, 1);   // 1 means shoot | can be simplifies with camera

            for(int i=0; i<coordinates.size(); i++){
                telemetry.addData("For loop", 1);
                telemetry.update();
                turnCalculations = TurnCalc(robotX, robotY, robotTheta, coordinates.get(i).get(0), coordinates.get(i).get(1), 90);
                relativePower(turnCalculations.get(0), turnCalculations.get(1));
                telemetry.addData("Motor1", FR_power);
                telemetry.addData("Motor2", FL_power);
                telemetry.addData("Motor3", BL_power);
                telemetry.addData("Motor4", BR_power);
                telemetry.update();
                sleep(3000);
                pidControl(turnCalculations.get(2));
                robotX = coordinates.get(i).get(0); // sets new robotX
                robotY = coordinates.get(i).get(1); // sets new robotY
            }
            // Turn off everything and end auto
            break;
        }
    }

    public Vector<Double> TurnCalc(double robotX, double robotY, double robotTheta, double targetX, double targetY, double targetTheta){
        double distError = Math.sqrt((Math.pow(targetX - robotX, 2)) + (Math.pow(targetY - robotY, 2)));
        double fieldTheta = Math.atan2((targetY-robotY), (targetX-robotY));
        double calcTheta = fieldTheta-robotTheta;

        // move these to top of program
        double direction;
        double rotation;
        // right = 0 | left = 1
        if(fieldTheta<0){
            direction = 0; // backward
        }
        else{
            direction = 1; // forward
        }
        if(calcTheta<0){
            rotation = 0;
        }
        else{
            rotation = 1;
        }
        Vector<Double> endCalcVector = new Vector<Double>(0);
        endCalcVector.add(direction);
        endCalcVector.add(rotation);
        endCalcVector.add(distError);
        telemetry.addData("TurnCalc", 1);
        telemetry.update();
        return endCalcVector;
    }

    public void relativePower(double direction, double rotation){
        if (direction>0){
            forwardBackward(1);
        }
        else{
            forwardBackward(-1);
        }
        if (rotation>0){
            clockwiseCounter(-1);
        }
        else{
            clockwiseCounter(1);
        }

        double maxMotor = Math.abs(Math.max(Math.max(FR_power, FL_power), Math.max(BL_power, BR_power)));
        if (maxMotor > 1){
            FR_power /= maxMotor;
            FL_power /= maxMotor;
            BL_power /= maxMotor;
            BR_power /= maxMotor;
        }
        telemetry.addData("Relative Power", 1);
        telemetry.update();

    }

    public void pidControl(double distError){
        double integralSum = 0;
        double lastError = 0;
        double error;
        double derivative;
        double out;
        ElapsedTime timer = new ElapsedTime();

        while(leftEncoderMotor.getCurrentPosition()<distError){
            leftEncoderPos = leftEncoderMotor.getCurrentPosition();

            error = distError - leftEncoderPos;
            derivative = (error-lastError) / timer.seconds();
            integralSum = integralSum + (error*timer.seconds());

            out = (Kp*error) + (Ki*integralSum) + (Kd*derivative);
            telemetry.addData("PID", out);
            telemetry.update();
            setPower(out);

            lastError = error;
            timer.reset();
        }
        stopAllPower();
    }

    public void setPower(double c){
        double maxMotor = Math.abs(Math.max(Math.max(FR_power, FL_power), Math.max(BL_power, BR_power)));
        if (maxMotor > 1){
            FR_power /= maxMotor;
            FL_power /= maxMotor;
            BL_power /= maxMotor;
            BR_power /= maxMotor;
        }
        FR_power *= c;
        FL_power *= c;
        BL_power *= c;
        BR_power *= c;

        FR.setPower(FR_power);
        FL.setPower(FL_power);
        BL.setPower(BL_power);
        BR.setPower(BR_power);
    }



    public void forwardBackward(int power){
        FR_power -= power;
        FL_power -= power;
        BL_power += power;
        BR_power += power;
    }

    public void leftRight(int power){
        FR_power -= power;
        FL_power += power;
        BL_power += power;
        BR_power -= power;
    }

    public void clockwiseCounter(int power){
        FR_power -= power;
        FL_power -= power;
        BL_power -= power;
        BR_power -= power;
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

        if (targetAngle == 180){
            targetTheta -= 0.1;
        }
        if (targetAngle == -180){
            targetTheta += 0.1;
        }

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

        if (yaw > targetAngle+2.5 || yaw < targetAngle-2.5) {
            turnRight(-0.15, targetAngle, orientation, 0);
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

        telemetry.addData("Yaw", yaw);
        telemetry.addData("Target", targetAngle);
        telemetry.update();

        if (targetAngle == 180){
            targetTheta -= 0.1;
        }
        if (targetAngle == -180){
            targetTheta += 0.1;
        }

        setLeftPower(power);
        setRightPower(-power);

        while(yaw > targetAngle){
            orientation = imu.getRobotYawPitchRollAngles();
            yaw = orientation.getYaw();


            telemetry.addData("Yaw", yaw);
            telemetry.addData("Target", targetAngle);
            telemetry.update();
            //telemIMUOrientation(orientation, yaw);
        }

        if (yaw < (-targetAngle)-2.5 || yaw > (-targetAngle)+2.5) {
            turnLeft(-0.15, targetAngle, orientation, 0);
        }

        stopAllPower();
        resetTicks();
        imu.resetYaw();

        sleep(500*sleep);

        //telemIMUOrientation(orientation, yaw);
    }


    public void setNormalDrive(){
        FR.setDirection(DcMotorSimple.Direction.FORWARD);
        FL.setDirection(DcMotorSimple.Direction.FORWARD);
        BR.setDirection(DcMotorSimple.Direction.FORWARD);
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
        storage.setPower(1);
        assistantServo.setPower(1);
        sleep(3000);
        assistantServo.setPower(0);
        storage.setPower(0);
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
        return rightEncoderMotor.getCurrentPosition() - rightEncoderPos;
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

    public double inchesToTicks(double inches) {
        double rev = inches / OPcircumference;
        double tick = 2000 * rev;
        return tick;
    }
}
