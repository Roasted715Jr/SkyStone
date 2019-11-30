package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.navigation.Position;

//Length is 15.75
//Width is 17.25

public class Hardware<T extends GenericOpMode> {
    private static final double INCH_PER_MM = 0.03937007874;
    private static final double WHEEL_DIAMETER_INCH = 100 * INCH_PER_MM;
    private static final double WHEEL_CIRCUMFERENCE_INCH = WHEEL_DIAMETER_INCH * Math.PI;
//    private static final int REV_CORE_HEX_COUNTS_PER_REVOLUTION = 288;
//    private static final int NEVEREST_40_COUNTS_PER_REVOLUTION = 1120;
    private static final int NEVEREST_20_COUNTS_PER_REVOLUTION = 537; //Is actually 537.6, but setting the motors requires an int so it will truncate to 537 anyways
//    private static final double EXTERNAL_COUNTS_PER_REVOLUTION = 8192;
//    private static final double TURN_SPEED = 0.25;
//    private static final double JOY_DEADZONE = 0.05;
//    private static final double MOTOR_MULTIPLIER = 0.5 / (Math.sqrt(2) / 2);

    private static final int MAIN_BOT = 0;
    private static final int MATT_TINY_BOT = 1;
    private static final int TINY_BOT = 2;
    private static final int MECANUM_PUSHBOT = 3;
    private static final int ROBOT_TYPE = MAIN_BOT;

    static final String VUFORIA_LICENSE_KEY = "Abq1tHr/////AAABmYC8ioniS0f2gyQRx7fZlTWMwyYcrV/bnslJvcDe0AhxA/GAkYTIdNbPWjYtplipzvASUZRGR+AoGDI1dKyuCFCc4qy1eVbx8NO4nuAKzeGoncY7acvfol19suW5Zl29E+APEV0CG4GVBe4R+bZ/Xyd2E7CZ7AcrLbWM8+SJiMCDnxJa3J0ozBHMPMs6GNFyYS6YCVNMkFcLEKxDicwXqpuJddG5XenbAs8ot9UT11WRYZjpprLkSRtM1/OyigcUeb0wk2PL6lFVBMHMZbWK5HkJEmBoN5+v2fP6zouj0GPGyEh/eV8Xe71LhBz0WXKd180hUCowZVBfdsTtuYwFiBkAyRLtiQQb4/b80sAx1b6s";

    private static int PINION_TEETH;
    private static int SPUR_TEETH;
    private static double xMult;
    private static double yMult;
    private static double rMult;

    //Ctrl+Q or Ctrl+Shift+I for documentation and definition
    RevColorSensorV3 rColor, lColor;
    Rev2mDistanceSensor distanceSensor;
    DcMotor frMotor, flMotor, brMotor, blMotor; //For the main bot
    DcMotor rightMotor, leftMotor; //For the baby bots
    private HardwareMap hardwareMap;
    Position pos;
    Servo armServo, clawServo, lFoundationServo, rFoundationServo;
    private T runningOpMode;
    TouchSensor rTouch, lTouch;

    Hardware(T runningOpMode) {
        this.runningOpMode = runningOpMode;
    }

    void init(HardwareMap ahwMap) {
        hardwareMap = ahwMap;

        switch (ROBOT_TYPE) {
            case MAIN_BOT:
                flMotor = hardwareMap.get(DcMotor.class, "flMotor");
                blMotor = hardwareMap.get(DcMotor.class, "blMotor");
                frMotor = hardwareMap.get(DcMotor.class, "frMotor");
                brMotor = hardwareMap.get(DcMotor.class, "brMotor");
                setMecanumMotorPower(0, 0, 0);
                setMecanumZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                setMecanumMotorRunmodes(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

                armServo = hardwareMap.get(Servo.class, "armServo"); //Vertical is at 0.3
                clawServo = hardwareMap.get(Servo.class, "clawServo");
                rFoundationServo = hardwareMap.get(Servo.class, "rFoundationServo");
                lFoundationServo = hardwareMap.get(Servo.class, "lFoundationServo");

                rColor = hardwareMap.get(RevColorSensorV3.class, "rColor");
                lColor = hardwareMap.get(RevColorSensorV3.class, "lColor");

//                rTouch = hardwareMap.get(TouchSensor.class, "rTouch");
//                lTouch = hardwareMap.get(TouchSensor.class, "lTouch");

                distanceSensor = hardwareMap.get(Rev2mDistanceSensor.class, "distanceSensor");

                //Init other sensors
                //3 encoders

                //Init gyro?

//                PINION_TEETH = 1;
//                SPUR_TEETH = 1;
                break;
            case MECANUM_PUSHBOT:
                flMotor = hardwareMap.get(DcMotor.class, "frontL");
                blMotor = hardwareMap.get(DcMotor.class, "backL");
                frMotor = hardwareMap.get(DcMotor.class, "frontR");
                brMotor = hardwareMap.get(DcMotor.class, "backR");
                frMotor.setDirection(DcMotor.Direction.REVERSE);
                brMotor.setDirection(DcMotor.Direction.REVERSE);
                setMecanumMotorPower(0, 0, 0);
                setMecanumZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                setMecanumMotorRunmodes(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

//                PINION_TEETH = 1;
//                SPUR_TEETH = 1;
                break;
            case MATT_TINY_BOT:
                rightMotor = hardwareMap.get(DcMotor.class, "rightMotor");
                leftMotor = hardwareMap.get(DcMotor.class, "leftMotor");
                leftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
                setSimpleMotorPowers(0);
                setSimpleZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                setSimpleMotorRunmodes(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

//                distanceSensor = hardwareMap.get(DistanceSensor.class, "distanceSensor");

                PINION_TEETH = 1;
                SPUR_TEETH = 1;
                break;
            case TINY_BOT:
                rightMotor = hardwareMap.get(DcMotor.class, "driveR");
                leftMotor = hardwareMap.get(DcMotor.class, "driveL");
                setSimpleMotorPowers(0);
                setSimpleZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                setSimpleMotorRunmodes(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

                PINION_TEETH = 1;
                SPUR_TEETH = 1;
                break;
        }
    }

    void setMecanumMotorPower(double x, double y, double r) {
        //Turn speed is half of x or y
        //x and y speed are the same
//        xMult = 0.4;
//        yMult = 0.4;
//        rMult = 0.2;

//        xMult = 1;
//        yMult = 1;
//        rMult = 1;

//        xMult = MOTOR_MULTIPLIER * 2 / 5;
//        yMult = MOTOR_MULTIPLIER * 2 / 5;
//        rMult = MOTOR_MULTIPLIER / 5;

        //When at 45 deg on left stick, multiply by 0.704 (or 1/2 / (sqrt(2) / 2)) to get optimal x and y

//        flMotor.setPower(x + y + r);
//        blMotor.setPower(-x + y + r);
//        frMotor.setPower(-x + y - r);
//        brMotor.setPower(x + y - r);

        //For the sideways configuration
        frMotor.setPower(-(-x + y - r));
        brMotor.setPower(-(x + y - r));
        flMotor.setPower(x + y + r);
        blMotor.setPower(-x + y + r);
    }

    void setSimpleMotorPowers(double power) {
        rightMotor.setPower(power);
        leftMotor.setPower(power);
    }

    void setMecanumMotorRunmodes(DcMotor.RunMode runMode) {
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

    void goDistance(double x, double y, double r) {
        setMecanumMotorRunmodes(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        int xCounts = (int) (x / WHEEL_CIRCUMFERENCE_INCH * NEVEREST_20_COUNTS_PER_REVOLUTION);
        int yCounts = (int) (y / WHEEL_CIRCUMFERENCE_INCH * NEVEREST_20_COUNTS_PER_REVOLUTION);
        int rCounts = 0;

        setMecanumTargetPositions(xCounts, yCounts, rCounts);

        setMecanumMotorRunmodes(DcMotor.RunMode.RUN_TO_POSITION);

//        rampMecanumMotors(0, 1, 0, true);

//        rampMecanumMotors(0, 0, 0, false);

        setMecanumMotorPower(0, 1, 0);

        while (flMotor.isBusy() && frMotor.isBusy()) {

        }

        setMecanumMotorPower(0, 0, 0);
        setMecanumMotorRunmodes(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    void setMecanumTargetPositions(int x, int y, int r) {
        flMotor.setTargetPosition(x + y + r);
        blMotor.setTargetPosition(-x + y + r);
        frMotor.setTargetPosition(-(-x + y - r));
        brMotor.setTargetPosition(-(x + y - r));
    }

    void rampMecanumMotors(double x, double y, double r, boolean rampUp) {
//        rampMotor(flMotor, x + y + r, x + y + r > 0);
//        rampMotor(flMotor, -x + y + r, -x + y + r > 0);
//        rampMotor(flMotor, -(-x + y - r), -(-x + y - r) > 0);
//        rampMotor(flMotor, -(x + y - r), -(x + y - r) > 0);

        rampMotor(flMotor, x, y, r, 0.1, 50, rampUp);
    }

    void rampMotor(DcMotor motor, double x, double y, double r, double increment, int cycle, boolean rampUp) {
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
                    setMecanumMotorPower(x, y, r);
                    return;
                }
            } else {
                // Keep stepping down until we hit the min value.
                power -= increment;
                if (power <= 0) {
                    //                power = target;
                    //                rampUp = !rampUp;  // Switch ramp direction
                    setMecanumMotorPower(0, 0, 0);
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
                setMecanumMotorPower(power * x, power * y, power * r);

            runningOpMode.addTelemetry("flMotor", flMotor.getCurrentPosition() + "/" + flMotor.getTargetPosition());
            runningOpMode.addTelemetry("blMotor", blMotor.getCurrentPosition() + "/" + blMotor.getTargetPosition());
            runningOpMode.addTelemetry("frMotor", frMotor.getCurrentPosition() + "/" + frMotor.getTargetPosition());
            runningOpMode.addTelemetry("brMotor", brMotor.getCurrentPosition() + "/" + brMotor.getTargetPosition());
            runningOpMode.updateTelemetry();

            try {
                Thread.sleep(cycle);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
    }

//    private Position getOdometryDistance() {
//        return new Position();
//    }

//    private double getOdometryRotation() {
//        return 0;
//    }
}
