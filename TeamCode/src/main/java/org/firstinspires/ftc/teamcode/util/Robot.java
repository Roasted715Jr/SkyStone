package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.odometry.OdometryGlobalCoordinatePosition;

//Length is 15.75
//Width is 17.25

public class Robot {
    private static final double INCH_PER_MM = 0.03937007874;
    private static final double WHEEL_DIAMETER_INCH = 100 * INCH_PER_MM;
    private static final double WHEEL_CIRCUMFERENCE_INCH = WHEEL_DIAMETER_INCH * Math.PI;
    private static final double ODOMETER_DIAMETER_INCH = 38 * INCH_PER_MM;
    private static final double ODOMETER_CIRCUMFERENCE_INCH = ODOMETER_DIAMETER_INCH * Math.PI;
    private static final double ODOMETER_COUNTS_PER_INCH = 360 * 4 / ODOMETER_CIRCUMFERENCE_INCH; //307.699557, we calculated 306.381642
//    private static final int REV_CORE_HEX_COUNTS_PER_REVOLUTION = 288;
//    private static final int NEVEREST_40_COUNTS_PER_REVOLUTION = 1120;
    private static final int NEVEREST_20_COUNTS_PER_REVOLUTION = 537; //Is actually 537.6, but setting the motors requires an int so it will truncate to 537 anyways
    private static final int ENCODER_THRESHOLD = 50;
    private static final double POWER_INCREMENT = 0.1;

    private static final int MAIN_BOT = 0;
    private static final int MATT_TINY_BOT = 1;
    private static final int TINY_BOT = 2;
    private static final int MECANUM_PUSHBOT = 3;
    private static final int INTAKE_TEST = 4;
    private static final int ROBOT_TYPE = INTAKE_TEST;

    public static final String VUFORIA_LICENSE_KEY = "Abq1tHr/////AAABmYC8ioniS0f2gyQRx7fZlTWMwyYcrV/bnslJvcDe0AhxA/GAkYTIdNbPWjYtplipzvASUZRGR+AoGDI1dKyuCFCc4qy1eVbx8NO4nuAKzeGoncY7acvfol19suW5Zl29E+APEV0CG4GVBe4R+bZ/Xyd2E7CZ7AcrLbWM8+SJiMCDnxJa3J0ozBHMPMs6GNFyYS6YCVNMkFcLEKxDicwXqpuJddG5XenbAs8ot9UT11WRYZjpprLkSRtM1/OyigcUeb0wk2PL6lFVBMHMZbWK5HkJEmBoN5+v2fP6zouj0GPGyEh/eV8Xe71LhBz0WXKd180hUCowZVBfdsTtuYwFiBkAyRLtiQQb4/b80sAx1b6s";

    //Ctrl+Q or Ctrl+Shift+I for documentation and definition
    public RevColorSensorV3 rColor, lColor;
    public Rev2mDistanceSensor distanceSensor;
    public DcMotor frMotor, flMotor, brMotor, blMotor; //For the main bot
    public DcMotor lOdometer, hOdometer, rOdometer; //The odometer encoders are identified through the motors
    public DcMotor rightMotor, leftMotor; //For the baby bots
    private HardwareMap hardwareMap;
    private OdometryGlobalCoordinatePosition globalPositionUpdate;
    public DcMotor armMotor;
    public DcMotor rIntake, lIntake, liftMotor, extendMotor;
    public Servo armServo, clawServo, lFoundationServo, rFoundationServo;//, blockServo;
    public Servo rOdometerServo, hOdometerServo, lOdometerServo;
    private GenericOpMode runningOpMode;
//    TouchSensor rTouch, lTouch;

    public Robot(GenericOpMode runningOpMode) {
        this.runningOpMode = runningOpMode;
    }

    public void init(HardwareMap ahwMap) {
        hardwareMap = ahwMap;

        switch (ROBOT_TYPE) {
            case MAIN_BOT:
                flMotor = hardwareMap.get(DcMotor.class, "flMotor");
                hOdometer = flMotor;
                blMotor = hardwareMap.get(DcMotor.class, "blMotor");
                lOdometer = blMotor;
                frMotor = hardwareMap.get(DcMotor.class, "frMotor");
                brMotor = hardwareMap.get(DcMotor.class, "brMotor");
                rOdometer = brMotor;
                setMecanumMotorPowers(0, 0, 0);
                setMecanumZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                setMecanumMotorRunmodes(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

                globalPositionUpdate = new OdometryGlobalCoordinatePosition(lOdometer, rOdometer, hOdometer, ODOMETER_COUNTS_PER_INCH, 75);
                globalPositionUpdate.reverseRightEncoder();
                globalPositionUpdate.reverseNormalEncoder();
                lOdometer.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rOdometer.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                hOdometer.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                lOdometerServo = hardwareMap.get(Servo.class, "lOdometerServo");
                hOdometerServo = hardwareMap.get(Servo.class, "hOdometerServo");
                rOdometerServo = hardwareMap.get(Servo.class, "rOdometerServo");

                lOdometer.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                rOdometer.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                hOdometer.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

//                armServo = hardwareMap.get(Servo.class, "armServo"); //Vertical is at 0.3
                armMotor = hardwareMap.get(DcMotor.class, "armMotor");
                armMotor.setDirection(DcMotorSimple.Direction.REVERSE);
                //As long as we keep power to the motor from the phone or the battery, the encoder will constantly be tracking its position
                armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                armMotor.setPower(0);
                clawServo = hardwareMap.get(Servo.class, "clawServo");

                rFoundationServo = hardwareMap.get(Servo.class, "rFoundationServo");
                lFoundationServo = hardwareMap.get(Servo.class, "lFoundationServo");
//                blockServo = hardwareMap.get(Servo.class, "blockServo");

                rColor = hardwareMap.get(RevColorSensorV3.class, "rColor");
                lColor = hardwareMap.get(RevColorSensorV3.class, "lColor");

//                rTouch = hardwareMap.get(TouchSensor.class, "rTouch");
//                lTouch = hardwareMap.get(TouchSensor.class, "lTouch");

                distanceSensor = hardwareMap.get(Rev2mDistanceSensor.class, "distance");
                break;
            case MECANUM_PUSHBOT:
                flMotor = hardwareMap.get(DcMotor.class, "frontL");
                blMotor = hardwareMap.get(DcMotor.class, "backL");
                frMotor = hardwareMap.get(DcMotor.class, "frontR");
                brMotor = hardwareMap.get(DcMotor.class, "backR");
                frMotor.setDirection(DcMotor.Direction.REVERSE);
                brMotor.setDirection(DcMotor.Direction.REVERSE);
                setMecanumMotorPowers(0, 0, 0);
                setMecanumZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                setMecanumMotorRunmodes(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                break;
            case MATT_TINY_BOT:
                rightMotor = hardwareMap.get(DcMotor.class, "rightMotor");
                leftMotor = hardwareMap.get(DcMotor.class, "leftMotor");
                leftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
                setSimpleMotorPowers(0);
                setSimpleZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                setSimpleMotorRunmodes(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                break;
            case TINY_BOT:
                rightMotor = hardwareMap.get(DcMotor.class, "driveR");
                leftMotor = hardwareMap.get(DcMotor.class, "driveL");
                setSimpleMotorPowers(0);
                setSimpleZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                setSimpleMotorRunmodes(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                break;
            case INTAKE_TEST:
                rIntake = hardwareMap.get(DcMotor.class, "rIntake");
                lIntake = hardwareMap.get(DcMotor.class, "lIntake");
                liftMotor = hardwareMap.get(DcMotor.class, "liftMotor");
                extendMotor = hardwareMap.get(DcMotor.class, "extendMotor");
                lIntake.setDirection(DcMotorSimple.Direction.REVERSE);
                liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                break;
        }
    }

    public void retractOdometers() {
        rOdometerServo.setPosition(1);
        hOdometerServo.setPosition(1);
        lOdometerServo.setPosition(0);
    }

    public void startGlobalPositionUpdate() {
        Thread positionThread = new Thread(globalPositionUpdate);
        positionThread.start();
    }

    public void setStartSpot(double x, double y, double r) {
        globalPositionUpdate.setStartSpot(x, y ,r);
    }

    public double getX() {
        return globalPositionUpdate.returnXCoordinate();
    }

    public double getY() {
        return globalPositionUpdate.returnYCoordinate();
    }

    public double getR() {
        return globalPositionUpdate.returnOrientation();
    }

    public double getCountsPerInch() {
        return ODOMETER_COUNTS_PER_INCH;
    }

    public void setMecanumMotorPowers(double x, double y, double r) {
//        flMotor.setPower(x + y + r);
//        blMotor.setPower(-x + y + r);
//        frMotor.setPower(-x + y - r);
//        brMotor.setPower(x + y - r);

        //We reverse these two motors because they're on the right side
        frMotor.setPower(-(-x + y - r));
        brMotor.setPower(-(x + y - r));
        flMotor.setPower(x + y + r);
        blMotor.setPower(-x + y + r);
    }

    void setSimpleMotorPowers(double power) {
        rightMotor.setPower(power);
        leftMotor.setPower(power);
    }

    public void setMecanumMotorRunmodes(DcMotor.RunMode runMode) {
        flMotor.setMode(runMode);
        blMotor.setMode(runMode);
        frMotor.setMode(runMode);
        brMotor.setMode(runMode);
    }

    void setSimpleMotorRunmodes(DcMotor.RunMode runMode) {
        rightMotor.setMode(runMode);
        leftMotor.setMode(runMode);
    }

    private void setMecanumZeroPowerBehavior(DcMotor.ZeroPowerBehavior zeroPowerBehavior) {
        flMotor.setZeroPowerBehavior(zeroPowerBehavior);
        blMotor.setZeroPowerBehavior(zeroPowerBehavior);
        frMotor.setZeroPowerBehavior(zeroPowerBehavior);
        brMotor.setZeroPowerBehavior(zeroPowerBehavior);
    }

    private void setSimpleZeroPowerBehavior(DcMotor.ZeroPowerBehavior zeroPowerBehavior) {
        rightMotor.setZeroPowerBehavior(zeroPowerBehavior);
        leftMotor.setZeroPowerBehavior(zeroPowerBehavior);
    }

    public void goDistance(double xDistance, double yDistance, double rAmount, double x, double y, double r) {
//        setMecanumMotorRunmodes(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        int xCounts = (int) ((xDistance/* + (xDistance > 0 ? 1 : -1)*/) / WHEEL_CIRCUMFERENCE_INCH * NEVEREST_20_COUNTS_PER_REVOLUTION); //Was 1.06 or 1.03 or 1.05
        int yCounts = (int) (yDistance / WHEEL_CIRCUMFERENCE_INCH * NEVEREST_20_COUNTS_PER_REVOLUTION);
        int rCounts = 0;

        setMecanumTargetPositions(xCounts, yCounts, rCounts);
        setMecanumMotorRunmodes(DcMotor.RunMode.RUN_TO_POSITION);
//        setMecanumMotorRunmodes(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

//        rampMecanumMotors(x, y, r, true);

//        rampMecanumMotors(x, y, r, false);

//        setMecanumMotorPowers(x, y, r);

        double power = 0;

//        while (nearTarget(flMotor, ENCODER_THRESHOLD) || nearTarget(blMotor, ENCODER_THRESHOLD) || nearTarget(frMotor, ENCODER_THRESHOLD) || nearTarget(brMotor, ENCODER_THRESHOLD)) {
        while (flMotor.isBusy() || blMotor.isBusy() || frMotor.isBusy() || brMotor.isBusy()) {
            if (nearTarget(flMotor, ENCODER_THRESHOLD) && nearTarget(blMotor, ENCODER_THRESHOLD) && nearTarget(frMotor, ENCODER_THRESHOLD) && nearTarget(brMotor, ENCODER_THRESHOLD))
                break;

            if (power < 1) {
                setMecanumMotorPowers(power * x, power * y, power * r);
                power += POWER_INCREMENT;
            }

            runningOpMode.addTelemetry("flMotor", flMotor.getCurrentPosition() + " / " + flMotor.getTargetPosition());
            runningOpMode.addTelemetry("blMotor", blMotor.getCurrentPosition() + " / " + blMotor.getTargetPosition());
            runningOpMode.addTelemetry("frMotor", frMotor.getCurrentPosition() + " / " + frMotor.getTargetPosition());
            runningOpMode.addTelemetry("brMotor", brMotor.getCurrentPosition() + " / " + brMotor.getTargetPosition());
            runningOpMode.updateTelemetry();
        }

//        runningOpMode.addTelemetry(nearTarget(flMotor, ENCODER_THRESHOLD));
//        runningOpMode.addTelemetry(nearTarget(blMotor, ENCODER_THRESHOLD));
//        runningOpMode.addTelemetry(nearTarget(frMotor, ENCODER_THRESHOLD));
//        runningOpMode.addTelemetry(nearTarget(brMotor, ENCODER_THRESHOLD));
//        runningOpMode.updateTelemetry();
        setMecanumMotorRunmodes(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    void setMecanumTargetPositions(int x, int y, int r) {
        flMotor.setTargetPosition(x + y + r + flMotor.getCurrentPosition());
        blMotor.setTargetPosition(-x + y + r + blMotor.getCurrentPosition());
        frMotor.setTargetPosition(-(-x + y - r) + frMotor.getCurrentPosition());
        brMotor.setTargetPosition(-(x + y - r) + brMotor.getCurrentPosition());
    }

    void rampMecanumMotors(double x, double y, double r, boolean rampUp) {
//        rampMotors(flMotor, x + y + r, x + y + r > 0);
//        rampMotors(flMotor, -x + y + r, -x + y + r > 0);
//        rampMotors(flMotor, -(-x + y - r), -(-x + y - r) > 0);
//        rampMotors(flMotor, -(x + y - r), -(x + y - r) > 0);

        rampMotors(x, y, r, 0.1, 1000, rampUp);
    }

    void rampMotors(double x, double y, double r, double increment, int cycle, boolean rampUp) {
        double power = rampUp ? 0 : 1;
        double flMax = 0, blMax = 0, frMax = 0, brMax = 0;

        if (!rampUp) {
            flMax = flMotor.getPower();
            blMax = blMotor.getPower();
            frMax = frMotor.getPower();
            brMax = brMotor.getPower();
        }

        //We need to find a way to set all of these motors to ramp to their specific values, preferably not the hard way
        while (true) {
            if (rampUp) {
                // Keep stepping up until we hit the max value.
                power += increment;
                if (power >= 1) {
                    //                power = target;
                    //                rampUp = !rampUp;   // Switch ramp direction
                    setMecanumMotorPowers(x, y, r);
                    return;
                }
            } else {
                // Keep stepping down until we hit the min value.
                power -= increment;
                if (power <= 0) {
                    //                power = target;
                    //                rampUp = !rampUp;  // Switch ramp direction
                    setMecanumMotorPowers(0, 0, 0);
                    return;
                }
            }

            // Display the current value
//            runningOpMode.addTelemetry("Motor Power", "%5.2f", power);
//            runningOpMode.addTelemetry(">", "Press Stop to end test.");
//            runningOpMode.updateTelemetry();

            // Set the motor to the new power and pause;
//            motor.setPower(power);
//            flMotor.setPower(power * (x + y + r));
//            blMotor.setPower(power * (-x + y + r));
//            frMotor.setPower(power * -(-x + y - r));
//            brMotor.setPower(power * -(x + y - r));
            if (!rampUp) {
                frMotor.setPower(power * frMax);
                brMotor.setPower(power * brMax);
                flMotor.setPower(power * flMax);
                blMotor.setPower(power * blMax);
            } else
                setMecanumMotorPowers(power * x, power * y, power * r);

//            runningOpMode.addTelemetry("flMotor", flMotor.getCurrentPosition() + "/" + flMotor.getTargetPosition());
//            runningOpMode.addTelemetry("blMotor", blMotor.getCurrentPosition() + "/" + blMotor.getTargetPosition());
//            runningOpMode.addTelemetry("frMotor", frMotor.getCurrentPosition() + "/" + frMotor.getTargetPosition());
//            runningOpMode.addTelemetry("brMotor", brMotor.getCurrentPosition() + "/" + brMotor.getTargetPosition());
//            runningOpMode.updateTelemetry();

            try {
                Thread.sleep(cycle);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
    }

    public void moveHooks(boolean deploy) {
        rFoundationServo.setPosition(deploy ? 1 : 0);
        lFoundationServo.setPosition(deploy ? 0 : 1);
    }

    private boolean nearTarget(DcMotor motor, int threshold) {
        if (motor.getTargetPosition() > 0)
            return motor.getTargetPosition() - motor.getCurrentPosition() <= threshold;
        else
            return motor.getCurrentPosition() - motor.getTargetPosition() <= threshold;

//        return Math.abs(motor.getTargetPosition() - motor.getCurrentPosition()) > threshold;
    }


    public void goToPosition(double targetX, double targetY, double targetRotation, double robotPower, double rotationSpeed, double threshold, double angleThreshold) {
        targetX *= ODOMETER_COUNTS_PER_INCH;
        targetY *= ODOMETER_COUNTS_PER_INCH;
        threshold *= ODOMETER_COUNTS_PER_INCH;

        double distanceToXTarget = targetX - globalPositionUpdate.returnXCoordinate();
        double distanceToYTarget = targetY - globalPositionUpdate.returnYCoordinate();
        double distance = Math.hypot(distanceToXTarget, distanceToYTarget);
        double orientation = globalPositionUpdate.returnOrientation();

//        telemetry.addData("orientation", orientation);
//        telemetry.update();

//        Log.d("orientation", Double.toString(orientation));
//        Log.d("Check 1", Double.toString(Math.abs(targetRotation) + angleThreshold));
//        Log.d("Check 2", Double.toString(Math.abs(targetRotation) - angleThreshold));

        while (runningOpMode.opModeIsActive() && (distance > threshold ||
                //Check to see if we are over or under our desired rotation to continue the loop
                (Math.abs(targetRotation) + angleThreshold < Math.abs(orientation) || Math.abs(orientation) < Math.abs(targetRotation) - angleThreshold))) {
            distanceToXTarget = targetX - globalPositionUpdate.returnXCoordinate();
            distanceToYTarget = targetY - globalPositionUpdate.returnYCoordinate();

            //We are reversing x and y because 0 degrees is forwards rather than to the right
            double robotMovementAngle = Math.toDegrees(Math.atan2(distanceToXTarget, distanceToYTarget));

            double robotMovementXComponent = calculateX(robotMovementAngle, robotPower);
            double robotMovementYComponent = calculateY(robotMovementAngle, robotPower);
            double pivotCorrection = targetRotation - globalPositionUpdate.returnOrientation();
            double robotMovementRComponent = calculateR(pivotCorrection, angleThreshold, rotationSpeed);

//            setMecanumMotorPowers(robotMovementXComponent, robotMovementYComponent, robotMovementRComponent);
            setMecanumMotorPowers(robotMovementXComponent, robotMovementYComponent, 0);
            distance = Math.hypot(distanceToXTarget, distanceToYTarget);
            orientation = globalPositionUpdate.returnOrientation();

            runningOpMode.telemetry.addData("distanceToXTarget", distanceToXTarget / ODOMETER_COUNTS_PER_INCH);
            runningOpMode.telemetry.addData("distanceToYTarget", distanceToYTarget / ODOMETER_COUNTS_PER_INCH);
            runningOpMode.telemetry.addData("distance", distance / ODOMETER_COUNTS_PER_INCH);

            runningOpMode.telemetry.addData("robotMovementAngle", robotMovementAngle);

            runningOpMode.telemetry.addData("robotMovementXComponent", robotMovementXComponent);
            runningOpMode.telemetry.addData("robotMovementYComponent", robotMovementYComponent);
            runningOpMode.telemetry.addData("pivotCorrection", pivotCorrection);
            runningOpMode.telemetry.addData("robotMovementRComponent", robotMovementRComponent);

            runningOpMode.telemetry.addData("orientation", orientation);
            runningOpMode.telemetry.addData("Check 1", Math.abs(targetRotation) + angleThreshold < Math.abs(orientation));
            runningOpMode.telemetry.addData("Check 2", Math.abs(orientation) < Math.abs(targetRotation) - angleThreshold);

            runningOpMode.telemetry.update();
        }

        setMecanumMotorPowers(0, 0, 0);
    }

    public void approachSkyStone(double xSpeed, double ySpeed, double threshold, double targetRotation, double rotationSpeed, double angleThreshold) {
        //This should start at 0
        double targetX = 0;
//        targetY *= ODOMETER_COUNTS_PER_INCH;
        threshold *= ODOMETER_COUNTS_PER_INCH;

        boolean foundBlock = true;

        //These values should start at 0
        double distanceToXTarget = 0;
//        double distanceToYTarget = targetY - globalPositionUpdate.returnYCoordinate();
        double orientation = 0;

//        telemetry.addData("orientation", orientation);
//        telemetry.update();

//        Log.d("orientation", Double.toString(orientation));
//        Log.d("Check 1", Double.toString(Math.abs(targetRotation) + angleThreshold));
//        Log.d("Check 2", Double.toString(Math.abs(targetRotation) - angleThreshold));

        while (runningOpMode.opModeIsActive() && (foundBlock || Math.abs(distanceToXTarget) > threshold ||
                //Check to see if we are over or under our desired rotation to continue the loop
                (Math.abs(targetRotation) + angleThreshold < Math.abs(orientation) || Math.abs(orientation) < Math.abs(targetRotation) - angleThreshold))) {
            foundBlock = !foundSkyStone(lColor) && !foundSkyStone(rColor);

            distanceToXTarget = targetX - globalPositionUpdate.returnXCoordinate();
//            distanceToYTarget = targetY - globalPositionUpdate.returnYCoordinate();

            //We are reversing x and y because 0 degrees is forwards rather than to the right
//            double robotMovementAngle = Math.toDegrees(Math.atan2(distanceToXTarget, distanceToYTarget));
            double robotMovementAngle = globalPositionUpdate.returnXCoordinate() - targetX > threshold ? -90 :
                    globalPositionUpdate.returnXCoordinate() - targetX < -threshold ? 90 : 0;

            double robotMovementXComponent = calculateX(robotMovementAngle, xSpeed);
//            double robotMovementYComponent = calculateY(robotMovementAngle, ySpeed);
            double pivotCorrection = targetRotation - globalPositionUpdate.returnOrientation();
            double robotMovementRComponent = calculateR(pivotCorrection, angleThreshold, rotationSpeed);

//            setMecanumMotorPowers(robotMovementXComponent, robotMovementYComponent, robotMovementRComponent);
            setMecanumMotorPowers(robotMovementXComponent, ySpeed, 0);
//            distance = Math.hypot(distanceToXTarget, distanceToYTarget);
            orientation = globalPositionUpdate.returnOrientation();

            runningOpMode.telemetry.addData("targetX", targetX);
            runningOpMode.telemetry.addData("xPosition", globalPositionUpdate.returnXCoordinate());
            runningOpMode.telemetry.addData("distanceToXTarget", distanceToXTarget / ODOMETER_COUNTS_PER_INCH);
//            runningOpMode.telemetry.addData("distanceToYTarget", distanceToYTarget / ODOMETER_COUNTS_PER_INCH);
//            runningOpMode.telemetry.addData("distance", distance / ODOMETER_COUNTS_PER_INCH);

            runningOpMode.telemetry.addData("robotMovementAngle", robotMovementAngle);

            runningOpMode.telemetry.addData("robotMovementXComponent", robotMovementXComponent);
//            runningOpMode.telemetry.addData("robotMovementYComponent", robotMovementYComponent);
            runningOpMode.telemetry.addData("pivotCorrection", pivotCorrection);
            runningOpMode.telemetry.addData("robotMovementRComponent", robotMovementRComponent);

            runningOpMode.telemetry.addData("orientation", orientation);
            runningOpMode.telemetry.addData("Check 1", Math.abs(targetRotation) + angleThreshold < Math.abs(orientation));
            runningOpMode.telemetry.addData("Check 2", Math.abs(orientation) < Math.abs(targetRotation) - angleThreshold);

            runningOpMode.telemetry.update();
        }

        setMecanumMotorPowers(0, 0, 0);
    }

    //From Dad
    public void goToPositionNew(int targetxyr, double targetX, double targetY, double robotPower, double targetRotation, double distanceThreshold, double angleThreshold) {
        targetX *= ODOMETER_COUNTS_PER_INCH;
        targetY *= ODOMETER_COUNTS_PER_INCH;
        distanceThreshold *= ODOMETER_COUNTS_PER_INCH;

        double distanceToXTarget = targetX - globalPositionUpdate.returnXCoordinate();
        double distanceToYTarget = targetY - globalPositionUpdate.returnYCoordinate();
        double distance = Math.hypot(distanceToXTarget, distanceToYTarget);
        double pivotCorrection = targetRotation - globalPositionUpdate.returnOrientation();
        double countMe = 0;
        boolean targetFound = false;
        while (runningOpMode.opModeIsActive() && !targetFound) { //(distance > distanceThreshold) || (Math.abs(pivotCorrection) > angleThreshold) ) {

            //We are reversing x and y because 0 degrees is forwards rather than to the right
            double robotMovementAngle = Math.toDegrees(Math.atan2(distanceToXTarget, distanceToYTarget));
            double robotMovementXComponent = calculateX(robotMovementAngle, robotPower);
            double robotMovementYComponent = calculateY(robotMovementAngle, robotPower);
            double robotMovementRComponent = calculateR(pivotCorrection,angleThreshold,robotPower);
            //countMe++;

            if (targetxyr == 3) {
                setMecanumMotorPowers(0, 0, robotMovementRComponent);
            }else{
                setMecanumMotorPowers(robotMovementXComponent, robotMovementYComponent, robotMovementRComponent/4);
            }
            //setMecanumMotorPowers(robotMovementXComponent, robotMovementYComponent, robotMovementRComponent);

            distance = Math.hypot(distanceToXTarget, distanceToYTarget);
            distanceToXTarget = targetX - globalPositionUpdate.returnXCoordinate();
            distanceToYTarget = targetY - globalPositionUpdate.returnYCoordinate();
            pivotCorrection = targetRotation - globalPositionUpdate.returnOrientation();

            switch (targetxyr) {
                case 1:
                    targetFound = Math.abs(distanceToXTarget) <= distanceThreshold;
                    break;
                case 2:
                    targetFound = Math.abs(distanceToYTarget) <= distanceThreshold;
                    break;
                case 3:
                    targetFound = Math.abs(pivotCorrection) <= angleThreshold;
                    break;
                case 0:
                    targetFound = distance <= distanceThreshold && Math.abs(pivotCorrection) <= angleThreshold;
                    break;
            }

            runningOpMode.telemetry.addData("X Position", globalPositionUpdate.returnXCoordinate() / ODOMETER_COUNTS_PER_INCH);
            runningOpMode.telemetry.addData("Y Position", globalPositionUpdate.returnYCoordinate() / ODOMETER_COUNTS_PER_INCH);
            runningOpMode.telemetry.addData("Orientation (Degrees)", globalPositionUpdate.returnOrientation());
            runningOpMode.telemetry.addData("Targeting", targetxyr);
            runningOpMode.telemetry.addData("distanceToXTarget", distanceToXTarget);
            runningOpMode.telemetry.addData("distanceToYTarget", distanceToYTarget);
            runningOpMode.telemetry.addData("distance", distance);

            runningOpMode.telemetry.addData("robotMovementAngle", robotMovementAngle);

            runningOpMode.telemetry.addData("robotMovementXComponent", robotMovementXComponent);
            runningOpMode.telemetry.addData("robotMovementYComponent", robotMovementYComponent);
            runningOpMode.telemetry.addData("pivotCorrection", pivotCorrection);

            runningOpMode.telemetry.update();
        }

        setMecanumMotorPowers(0, 0, 0);
    }

    public void centerOnSkyStone(double xSpeed, double ySpeed, double targetRotation, double rotationSpeed, double angleThreshold) {
        double orientation = globalPositionUpdate.returnOrientation();
        boolean keepMoving = true;

//        telemetry.addData("orientation", orientation);
//        telemetry.update();

//        Log.d("orientation", Double.toString(orientation));
//        Log.d("Check 1", Double.toString(Math.abs(targetRotation) + angleThreshold));
//        Log.d("Check 2", Double.toString(Math.abs(targetRotation) - angleThreshold));

        while (runningOpMode.opModeIsActive() && (!foundSkyStone(lColor) || !foundSkyStone(rColor) ||
                //Check to see if we are over or under our desired rotation to continue the loop
                (Math.abs(targetRotation) + angleThreshold < Math.abs(orientation) || Math.abs(orientation) < Math.abs(targetRotation) - angleThreshold))) {
            if (foundSkyStone(lColor) && foundSkyStone(rColor))
                keepMoving = false;

            double pivotCorrection = targetRotation - globalPositionUpdate.returnOrientation();
            double robotMovementRComponent = calculateR(pivotCorrection, angleThreshold, rotationSpeed);

            setMecanumMotorPowers(keepMoving ? xSpeed : 0, keepMoving ? ySpeed : 0, robotMovementRComponent);
//            setMecanumMotorPowers(robotMovementXComponent, robotMovementYComponent, 0);
            orientation = globalPositionUpdate.returnOrientation();

            runningOpMode.telemetry.addData("pivotCorrection", pivotCorrection);
            runningOpMode.telemetry.addData("robotMovementRComponent", robotMovementRComponent);
            runningOpMode.telemetry.addData("orientation", orientation);
//            runningOpMode.telemetry.addData("Check 1", Math.abs(targetRotation) + angleThreshold < Math.abs(orientation));
//            runningOpMode.telemetry.addData("Check 2", Math.abs(orientation) < Math.abs(targetRotation) - angleThreshold);

            runningOpMode.addTelemetry("lColor Found SkyStone", foundSkyStone(lColor));
            runningOpMode.addTelemetry("rColor Found SkyStone", foundSkyStone(rColor));

            runningOpMode.telemetry.update();
        }

        setMecanumMotorPowers(0, 0, 0);
    }

    public void goToPosition(double targetX, double targetY, double targetRotation, double speed, double rotationSpeed) {
        goToY(targetY, speed);
        goToX(targetX, speed);
//        turnToR(targetRotation, rotationSpeed);
//        goToY(targetY, speed);
    }

    void goToY(double targetY, double speed) {
        targetY *= ODOMETER_COUNTS_PER_INCH;
        double distanceToYTarget = targetY - globalPositionUpdate.returnYCoordinate();

        //Make this so we can go backwards with this as well
        while (runningOpMode.opModeIsActive() && Math.abs(distanceToYTarget) > 0.25) {
            distanceToYTarget = targetY - globalPositionUpdate.returnYCoordinate();
            setMecanumMotorPowers(0, (distanceToYTarget > 0 ? 1 : -1) * speed, 0);
            runningOpMode.telemetry.addData("distanceToYTarget", distanceToYTarget / ODOMETER_COUNTS_PER_INCH);
            runningOpMode.telemetry.update();
        }

        setMecanumMotorPowers(0, 0, 0);
    }

    void goToX(double targetX, double speed) {
        targetX *= ODOMETER_COUNTS_PER_INCH;
        double distanceToXTarget = targetX - globalPositionUpdate.returnXCoordinate();

        //Make this so we can go backwards with this as well
        while (runningOpMode.opModeIsActive() && Math.abs(distanceToXTarget) > 0.25) {
            distanceToXTarget = targetX - globalPositionUpdate.returnXCoordinate();
            setMecanumMotorPowers((distanceToXTarget > 0 ? 1 : -1) * speed, 0, 0);
            runningOpMode.telemetry.addData("distanceToXTarget", distanceToXTarget / ODOMETER_COUNTS_PER_INCH);
            runningOpMode.telemetry.addData("horizontalEncoder", hOdometer.getCurrentPosition());
            runningOpMode.telemetry.update();
        }

        setMecanumMotorPowers(0, 0, 0);
    }

    private double calculateX(double desiredAngle, double speed) {
        return Math.sin(Math.toRadians(desiredAngle)) * speed;
    }

    private double calculateY(double desiredAngle, double speed) {
        return Math.cos(Math.toRadians(desiredAngle)) * speed;
    }

    private double calculateR(double desiredAngle, double threshold, double speed) {
        if (Math.abs(desiredAngle) <= threshold)
            return 0;

        return (desiredAngle > 0 ? 1 : -1) * speed / 5;
    }

    public void moveArmMotor(int pos) {
        armMotor.setTargetPosition(pos);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setPower(1);
    }

    public boolean foundSkyStone(RevColorSensorV3 color) {
//        return inRange(color.red(), 500, 2500) && inRange(color.green(), 1000, 3500) && inRange(color.blue(), 600, 2000); //With light on
        return inRange(color.red(), 0, 5) && inRange(color.green(), 0, 5) && inRange(color.blue(), 0, 5); //With light off
    }

    public int sumOfColors(RevColorSensorV3 color) {
        return color.red() + color.blue() + color.green();
    }

    private boolean inRange(double val, double min, double max) {
        return min <= val && val <= max;
    }

    public void stop() {
        //Stop the thread
        globalPositionUpdate.stop();
    }

    public void sleep(long ms) {
        try {
            Thread.sleep(ms);
        } catch (Exception e) {

        }
    }
}
