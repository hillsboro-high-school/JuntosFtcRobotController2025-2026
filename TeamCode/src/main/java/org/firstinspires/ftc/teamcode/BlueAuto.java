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


    double localTargetTick;

    // Calculates the circumference for the Odometry pods
    // Divides by 25.4 to change mm to inches
    double OPcircumference = 2.0*Math.PI*(24.0/25.4);

    double aprilTagX = halfeTileMat;
    double aprilTagY = 11*halfeTileMat;

    double robotX = 8*halfeTileMat;
    double robotY = 8*halfeTileMat;
    double robotTheta = 90;
    double targetTheta;
    double direction;
    double strafe;
    Vector<Double> turnCalculations = new Vector<Double>(0);

    double Kp = 2;
    double Ki = 0.3;
    double Kd = 0.2;


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

        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.RIGHT;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.UP;

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


        while(opModeIsActive()) {

            /*
            ALL POWER MUST BE NEGATIVE
            Sleep is used when testing, set to 0 for max time
             */


            List<List<Double>> coordinates = new ArrayList<List<Double>>();
            append(coordinates, 4 * halfeTileMat, 4* halfeTileMat,  1);  // data1 = x
            //append(coordinates, 4*halfeTileMat, 4 * halfeTileMat, 0);             // data2 = y
            //append(coordinates, 5 * halfeTileMat, 7 * halfeTileMat, 1);   // data3 = shoot
            //append(coordinates, halfeTileMat, 5 * halfeTileMat, 0);             // 0 means don't shoot
            //append(coordinates, 5 * halfeTileMat, 7 * halfeTileMat, 1);   // 1 means shoot | can be simplifies with camera

            for(int i=0; i<coordinates.size(); i++){
                turnCalculations = TurnCalc(robotX, robotY, robotTheta, coordinates.get(i).get(0), coordinates.get(i).get(1), 90);
                relativePower(turnCalculations.get(0), turnCalculations.get(1), turnCalculations.get(2), turnCalculations.get(3));
                pidControl(turnCalculations.get(4));
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
        double calcTheta = fieldTheta-robotTheta;


        if(fieldTheta>=0){
            direction = 1; // forward
            if (fieldTheta>=90){
                strafe = 1; // left
            }
            else{
                strafe = -1; // right
            }
        }else{
            direction = -1; // backward
            if (fieldTheta<=-90){
                strafe = 1; // left
            }
            else{
                strafe = -1; // right
            }
        }

        Vector<Double> endCalcVector = new Vector<Double>(0);
        endCalcVector.add(calcTheta);
        endCalcVector.add(fieldTheta);
        endCalcVector.add(direction);
        endCalcVector.add(strafe);
        endCalcVector.add(distError);
        return endCalcVector;
    }

    public void relativePower(double calcTheta, double fieldTheta, double direction, double strafe){
        fieldTheta = Math.toRadians(fieldTheta);
        forwardBackward(direction, fieldTheta);
        leftRight(strafe, fieldTheta);

        double maxMotor = Math.abs(Math.max(Math.max(Math.abs(FR_power), Math.abs(FL_power)), Math.max(Math.abs(BL_power), Math.abs(BR_power))));

        if (maxMotor > 1){
            FR_power /= maxMotor;
            FL_power /= maxMotor;
            BL_power /= maxMotor;
            BR_power /= maxMotor;
        }

        telemetry.addData("FR", FR_power);
        telemetry.addData("FL", FL_power);
        telemetry.addData("BR", BR_power);
        telemetry.addData("BL", BL_power);
        telemetry.update();
        sleep(5000);

    }

    public void pidControl(double distError){
        double integralSum = 0;
        double lastError = 0;
        double error;
        double derivative;
        double out;
        double p;
        double i;
        double d;

        resetLeftTicks();
        resetCenterTicks();
        ElapsedTime timer = new ElapsedTime();

        double yTicks = getLeftTicks();
        double xTicks = getCenterTicks();
        double robotDist = 0;

        while(robotDist<inchesToTicks(distError)){

            telemetry.addData("LeftTicks", getLeftTicks());
            telemetry.addData("CenterTicks", getCenterTicks());
            telemetry.update();

            error = distError - ticksToInches(getLeftTicks());
            derivative = (error-lastError) / timer.seconds();
            integralSum = integralSum + (error*timer.seconds());

            p = Kp*error;
            i = Ki*integralSum;
            d = Kd*derivative;

            out = p+i+d;
            telemetry.addData("P", p);
            telemetry.addData("I", i);
            telemetry.addData("D", d);
            telemetry.addData("PID", out);
            telemetry.update();
            setPower(out);

            lastError = error;
            robotDist = Math.sqrt(Math.pow(yTicks, 2) + Math.pow(xTicks, 2));
            timer.reset();
        }
        stopAllPower();
    }

    public void setPower(double c){

        FR_power *= c;
        FL_power *= c;
        BL_power *= c;
        BR_power *= c;

        double maxMotor = Math.abs(Math.max(Math.max(FR_power, FL_power), Math.max(BL_power, BR_power)));
        if (maxMotor > 1){
            FR_power /= maxMotor;
            FL_power /= maxMotor;
            BL_power /= maxMotor;
            BR_power /= maxMotor;
        }

        /*
        telemetry.addData("FR", FR_power);
        telemetry.addData("FL", FL_power);
        telemetry.addData("BR", BR_power);
        telemetry.addData("BL", BL_power);

         */

        FR.setPower(FR_power);
        FL.setPower(FL_power);
        BL.setPower(BL_power);
        BR.setPower(BR_power);
    }


    public void forwardBackward(double power, double theta){

        FR_power += Math.sin(theta);
        FL_power += Math.sin(theta);
        BL_power += Math.sin(theta);
        BR_power += Math.sin(theta);
    }

    public void leftRight(double power, double theta){

        FR_power += Math.cos(theta);
        FL_power -= Math.cos(theta);
        BL_power += Math.cos(theta);
        BR_power -= Math.cos(theta);
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

    public double inchesToTicks(double inches) {
        double rev = inches / OPcircumference;
        return 2000 * rev;

    }
    public double ticksToInches(double ticks) {
        double rev = ticks/2000;
        return rev*OPcircumference;
    }

}
