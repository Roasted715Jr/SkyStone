package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Hardware<T extends GenericOpMode> {
    private static final double INCH_PER_MM = 0.03937007874;
    private static final double WHEEL_DIAMETER_INCH = 100.965 * INCH_PER_MM;
    private static final double WHEEL_CIRCUMFERENCE_INCH = WHEEL_DIAMETER_INCH * Math.PI;
    private static final int REV_CORE_HEX_COUNTS_PER_REVOLUTION = 288;
    private static final int NEVEREST_40_COUNTS_PER_REVOLUTION = 1120;
    //    private static final int NEVEREST_20_COUNTS_PER_REVOLUTION = 537; //Is actually 537.6, but setting the motors requires an int so it will truncate to 537 anyways
    private static final double TURN_SPEED = 0.25;

    private static final int MAIN_BOT = 0;
    private static final int MATT_TINY_BOT = 1;
    private static final int TINY_BOT = 2;
    private static final int ROBOT_TYPE = MAIN_BOT;

    static final String VUFORIA_LICENSE_KEY = "Abq1tHr/////AAABmYC8ioniS0f2gyQRx7fZlTWMwyYcrV/bnslJvcDe0AhxA/GAkYTIdNbPWjYtplipzvASUZRGR+AoGDI1dKyuCFCc4qy1eVbx8NO4nuAKzeGoncY7acvfol19suW5Zl29E+APEV0CG4GVBe4R+bZ/Xyd2E7CZ7AcrLbWM8+SJiMCDnxJa3J0ozBHMPMs6GNFyYS6YCVNMkFcLEKxDicwXqpuJddG5XenbAs8ot9UT11WRYZjpprLkSRtM1/OyigcUeb0wk2PL6lFVBMHMZbWK5HkJEmBoN5+v2fP6zouj0GPGyEh/eV8Xe71LhBz0WXKd180hUCowZVBfdsTtuYwFiBkAyRLtiQQb4/b80sAx1b6s";

    private static int PINION_TEETH;
    private static int SPUR_TEETH;

    DcMotor frMotor, flMotor, brMotor, blMotor; //For the main bot
    DcMotor rightMotor, leftMotor; //For the baby bots
    private HardwareMap hardwareMap;
    private T runningOpMode;

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
                setMecanumMotorPowers(0, 0, 0);
                setMecanumZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                setMecanumMotorRunmodes(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

                //Init other sensors

                //Init gyro?

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
            case MATT_TINY_BOT:
                rightMotor = hardwareMap.get(DcMotor.class, "rightMotor");
                leftMotor = hardwareMap.get(DcMotor.class, "leftMotor");
                leftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
                setSimpleMotorPowers(0);
                setSimpleZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                setSimpleMotorRunmodes(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

                PINION_TEETH = 1;
                SPUR_TEETH = 1;
                break;
        }
    }

    void setMecanumMotorPowers(double x, double y, double r) {
        flMotor.setPower(x * (0.4) + y * (0.4) + r * (0.2));
        blMotor.setPower(-x * (0.4) + y * (0.4) + r * (0.2));
        frMotor.setPower(-x * (0.4) + y * (0.4) - r * (0.2));
        brMotor.setPower(x * (0.4) + y * (0.4) - r * (0.2));
    }

    void setSimpleMotorPowers(double power) {
        rightMotor.setPower(power);
        leftMotor.setPower(power);
    }

    private void setMecanumMotorRunmodes(DcMotor.RunMode runMode) {
        flMotor.setMode(runMode);
        blMotor.setMode(runMode);
        frMotor.setMode(runMode);
        brMotor.setMode(runMode);
    }

    private void setSimpleMotorRunmodes(DcMotor.RunMode runMode) {
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
}
