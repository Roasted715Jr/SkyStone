package org.firstinspires.ftc.teamcode.odometry;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;

/**
 * Created by Sarthak on 10/4/2019.
 */
@TeleOp(name = "My Odometry OpMode")
//@Disabled
public class MyOdometryOpmode extends LinearOpMode {
    //Drive motors
    DcMotor right_front, right_back, left_front, left_back;
    //Odometry Wheels
    DcMotor verticalLeft, verticalRight, horizontal;

    final double COUNTS_PER_INCH = 306.381642;//90;//5;//.46; //307.699557;

    //Hardware Map Names for drive motors and odometry wheels. THIS WILL CHANGE ON EACH ROBOT, YOU NEED TO UPDATE THESE VALUES ACCORDINGLY
    String rfName = "frMotor", rbName = "brMotor", lfName = "flMotor", lbName = "blMotor";
//    String verticalLeftEncoderName = rbName, verticalRightEncoderName = lfName, horizontalEncoderName = rfName;
    String verticalLeftEncoderName = lbName, verticalRightEncoderName = rbName, horizontalEncoderName = lfName;

    OdometryGlobalCoordinatePosition globalPositionUpdate;


    private File odometerMultiplier = AppUtil.getInstance().getSettingsFile("odometerMultiplier.txt");
    private double odometerYMultiplier = Double.parseDouble(ReadWriteFile.readFile(odometerMultiplier).trim());

    //Use this to calibrate
//        ReadWriteFile.writeFile(odometerMultiplier, String.valueOf(24 * COUNTS_PER_INCH / globalPositionUpdate.returnYCoordinate()));
//        telemetry.addData("odometerMultiplier", Double.parseDouble(ReadWriteFile.readFile(odometerMultiplier).trim()));
//        telemetry.update();


    @Override
    public void runOpMode() throws InterruptedException {
        //Initialize hardware map values. PLEASE UPDATE THESE VALUES TO MATCH YOUR CONFIGURATION
        initDriveHardwareMap(rfName, rbName, lfName, lbName, verticalLeftEncoderName, verticalRightEncoderName, horizontalEncoderName);

        telemetry.addData("Status", "Init Complete");
        telemetry.update();
        waitForStart();

        //Create and start GlobalCoordinatePosition thread to constantly update the global coordinate positions
        globalPositionUpdate = new OdometryGlobalCoordinatePosition(verticalLeft, verticalRight, horizontal, COUNTS_PER_INCH, 75);
        Thread positionThread = new Thread(globalPositionUpdate);
        positionThread.start();

        globalPositionUpdate.reverseRightEncoder();
//        globalPositionUpdate.reverseLeftEncoder();
        globalPositionUpdate.reverseNormalEncoder();

//        goToPositionNew(0, 23.75, 0, 0.5, 0.2, 1.5, 3);

//        goToPositionNew(0, 24, 0.2, 0, 0.2, 0.5, 3);
//        goToPositionNew(24, 23.75, 0.5, 0, 0.2, 1.5, 3);
//        goToPositionNew(24, 24, 0.2, 0, 0.2, 0.5, 3);
//        goToPositionNew(0.75, 0.75, 0.5, 0, 0.2, 1.5, 3);
//        goToPositionNew(0, 0, 0.2, 0, 0.2, 0.5, 3);
//        goToPositionNew(0, 0, 0.2, 90, 0.5, 2, 3);

//        goToPositionNew(0, 24, 0.5, 0, 1.25, 3);
//        goToPositionNew(24, 24, 0.5, 0, 1.25, 3);
//        goToPositionNew(0, 0, 0.5, 0, 1.25, 3);

//        goToPositionNew(0, 0, 0.3, 90, 1.25, 3);

//        goToPositionNew(0, 24, 0.2, 0, 0.5, 1);

        //22.5 y
        goToPosition(-36, 0, 0, 0.25, 0);

        while(opModeIsActive()){
            //Display Global (x, y, theta) coordinates
//            telemetry.addData("X Position", globalPositionUpdate.returnXCoordinate() / COUNTS_PER_INCH);
//            telemetry.addData("Y Position", globalPositionUpdate.returnYCoordinate() / COUNTS_PER_INCH);
//            telemetry.addData("Orientation (Degrees)", globalPositionUpdate.returnOrientation());
////
//            telemetry.addData("Vertical left encoder position", verticalLeft.getCurrentPosition());
//            telemetry.addData("Vertical right encoder position", verticalRight.getCurrentPosition());
//            telemetry.addData("horizontal encoder position", horizontal.getCurrentPosition());
////
//            telemetry.addData("Thread Active", positionThread.isAlive());
//            telemetry.update();
        }

        //Stop the thread
        globalPositionUpdate.stop();
    }

    private void initDriveHardwareMap(String rfName, String rbName, String lfName, String lbName, String vlEncoderName, String vrEncoderName, String hEncoderName){
        right_front = hardwareMap.dcMotor.get(rfName);
        right_back = hardwareMap.dcMotor.get(rbName);
        left_front = hardwareMap.dcMotor.get(lfName);
        left_back = hardwareMap.dcMotor.get(lbName);

        verticalLeft = hardwareMap.dcMotor.get(vlEncoderName);
        verticalRight = hardwareMap.dcMotor.get(vrEncoderName);
        horizontal = hardwareMap.dcMotor.get(hEncoderName);

        right_front.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_back.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left_front.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left_back.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        right_front.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right_back.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        left_front.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        left_back.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        verticalLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        verticalRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        horizontal.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        verticalLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        verticalRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        horizontal.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        right_front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right_back.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        left_front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        left_back.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addData("Status", "Hardware Map Init Complete");
        telemetry.update();
    }

    public void goToPosition(double targetX, double targetY, double targetRotation, double robotPower, double rotationSpeed, double threshold, double angleThreshold) {
        targetX *= COUNTS_PER_INCH;
        targetY *= COUNTS_PER_INCH;
        threshold *= COUNTS_PER_INCH;

        double distanceToXTarget = targetX - globalPositionUpdate.returnXCoordinate();
        double distanceToYTarget = targetY - globalPositionUpdate.returnYCoordinate();
        double distance = Math.hypot(distanceToXTarget, distanceToYTarget);
        double orientation = globalPositionUpdate.returnOrientation();

//        telemetry.addData("orientation", orientation);
//        telemetry.update();

//        Log.d("orientation", Double.toString(orientation));
//        Log.d("Check 1", Double.toString(Math.abs(targetRotation) + angleThreshold));
//        Log.d("Check 2", Double.toString(Math.abs(targetRotation) - angleThreshold));

        while (opModeIsActive() && (distance > threshold ||
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

            setMecanumMotorPowers(robotMovementXComponent, robotMovementYComponent, robotMovementRComponent);
            distance = Math.hypot(distanceToXTarget, distanceToYTarget);
            orientation = globalPositionUpdate.returnOrientation();

            telemetry.addData("distanceToXTarget", distanceToXTarget / COUNTS_PER_INCH);
            telemetry.addData("distanceToYTarget", distanceToYTarget / COUNTS_PER_INCH);
            telemetry.addData("distance", distance / COUNTS_PER_INCH);
            telemetry.addData("orientation", orientation);

            telemetry.addData("robotMovementAngle", robotMovementAngle);

            telemetry.addData("robotMovementXComponent", robotMovementXComponent);
            telemetry.addData("robotMovementYComponent", robotMovementYComponent);
            telemetry.addData("pivotCorrection", pivotCorrection);
            telemetry.addData("robotMovementRComponent", robotMovementRComponent);

            telemetry.addData("orientation", orientation);
            telemetry.addData("Check 1", Math.abs(targetRotation) + angleThreshold < Math.abs(orientation));
            telemetry.addData("Check 2", Math.abs(orientation) < Math.abs(targetRotation) - angleThreshold);

            telemetry.update();
        }

        setMecanumMotorPowers(0, 0, 0);
    }

    //Move close to the block
    //Adjust x
    //Adjust orientation
    //Move closer to the block

    void goToPosition(double targetX, double targetY, double targetRotation, double speed, double rotationSpeed) {
        goToY(targetY, speed);
        goToX(targetX, speed);
//        turnToR(targetRotation, rotationSpeed);
//        goToY(targetY, speed);
    }

    void goToY(double targetY, double speed) {
//        double distanceToYTarget = targetY - globalPositionUpdate.returnYCoordinate();
////        double distanceToYTarget = (targetY - globalPositionUpdate.returnYCoordinate()) * odometerYMultiplier / COUNTS_PER_INCH;
////
//        while (opModeIsActive() && distanceToYTarget > 0.25) {
//            distanceToYTarget = targetY - globalPositionUpdate.returnYCoordinate();
////            distanceToYTarget = (targetY - globalPositionUpdate.returnYCoordinate()) * odometerYMultiplier / COUNTS_PER_INCH;
////
//            setMecanumMotorPowers(0, 0.25, 0);
////
//            telemetry.addData("y-Coordinate", globalPositionUpdate.returnYCoordinate());
//            telemetry.addData("Left Odometer", verticalLeft.getCurrentPosition());
//            telemetry.addData("Right Odometer", verticalRight.getCurrentPosition());
//            telemetry.addData("distanceToYTarget", distanceToYTarget);
//            telemetry.addData("Distance in inches", distanceToYTarget / COUNTS_PER_INCH);
//            telemetry.update();
//        }
////
//        setMecanumMotorPowers(0, 0, 0);
//
////        ReadWriteFile.writeFile(odometerMultiplier, String.valueOf(targetY / COUNTS_PER_INCH / globalPositionUpdate.returnYCoordinate()));
////        ReadWriteFile.writeFile(odometerMultiplier, String.valueOf(distanceToYTarget / (targetY / COUNTS_PER_INCH)));
////        telemetry.addData("odometerMultiplier", Double.parseDouble(ReadWriteFile.readFile(odometerMultiplier).trim()));

        targetY *= COUNTS_PER_INCH;
        double distanceToYTarget = targetY - globalPositionUpdate.returnYCoordinate();

        //Make this so we can go backwards with this as well
        while (opModeIsActive() && Math.abs(distanceToYTarget) > 0.25) {
            distanceToYTarget = targetY - globalPositionUpdate.returnYCoordinate();
            setMecanumMotorPowers(0, (distanceToYTarget > 0 ? 1 : -1) * speed, 0);
            telemetry.addData("distanceToYTarget", distanceToYTarget / COUNTS_PER_INCH);
            telemetry.update();
        }

        setMecanumMotorPowers(0, 0, 0);
    }

    void goToX(double targetX, double speed) {
        targetX *= COUNTS_PER_INCH;
        double distanceToXTarget = targetX - globalPositionUpdate.returnXCoordinate();

        //Make this so we can go backwards with this as well
        while (opModeIsActive() && Math.abs(distanceToXTarget) > 0.25) {
            distanceToXTarget = targetX - globalPositionUpdate.returnXCoordinate();
            setMecanumMotorPowers((distanceToXTarget > 0 ? 1 : -1) * speed, 0, 0);
            telemetry.addData("distanceToXTarget", distanceToXTarget / COUNTS_PER_INCH);
            telemetry.addData("horizontalEncoder", horizontal.getCurrentPosition());
            telemetry.update();
        }

        setMecanumMotorPowers(0, 0, 0);
    }

    void turnToR(double targetRotation, double rotationSpeed) {

    }

    private void setMecanumMotorPowers(double x, double y, double r) {
        //We reverse these because they're on the right side
        right_front.setPower(-(-x + y - r));
        right_back.setPower(-(x + y - r));
        left_front.setPower(x + y + r);
        left_back.setPower(-x + y + r);
    }

    /**
     * Calculate the power in the x direction
     * @param desiredAngle angle on the x axis
     * @param speed robot's speed
     * @return the x vector
     */
    private double calculateX(double desiredAngle, double speed) {
        return Math.sin(Math.toRadians(desiredAngle)) * speed;
    }

    /**
     * Calculate the power in the y direction
     * @param desiredAngle angle on the y axis
     * @param speed robot's speed
     * @return the y vector
     */
    private double calculateY(double desiredAngle, double speed) {
        return Math.cos(Math.toRadians(desiredAngle)) * speed;
    }

    private double calculateR(double desiredAngle, double threshold, double speed) {
        if (Math.abs(desiredAngle) <= threshold)
            return 0;

        return (desiredAngle > 0 ? 1 : -1) * speed / 5;
    }
}
