package org.firstinspires.ftc.teamcode.odometry;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Testing Odometry - SPD")
public class TestOdometry_SPD extends LinearOpMode{
    //Drive motors
    DcMotor right_front, right_back, left_front, left_back;
    //Odometry Wheels
    DcMotor verticalLeft, verticalRight, horizontal;

    final double COUNTS_PER_INCH = 306.381642; //307.699557;

    //Hardware Map Names for drive motors and odometry wheels. THIS WILL CHANGE ON EACH ROBOT, YOU NEED TO UPDATE THESE VALUES ACCORDINGLY
    String rfName = "frMotor", rbName = "brMotor", lfName = "flMotor", lbName = "blMotor";
    //    String verticalLeftEncoderName = rbName, verticalRightEncoderName = lfName, horizontalEncoderName = rfName;
    String verticalLeftEncoderName = lbName, verticalRightEncoderName = rbName, horizontalEncoderName = lfName;

    OdometryGlobalCoordinatePosition globalPositionUpdate;

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

//        goToPosition(0 * COUNTS_PER_INCH, 24 * COUNTS_PER_INCH, 0.5, 0, 1 * COUNTS_PER_INCH);
//        goToPosition(24 * COUNTS_PER_INCH, 24 * COUNTS_PER_INCH, 0.5, 0, 1 * COUNTS_PER_INCH);
//        goToPosition(0 * COUNTS_PER_INCH, 0 * COUNTS_PER_INCH, 0.5, 0, 1 * COUNTS_PER_INCH);

        goToPosition(3,0 * COUNTS_PER_INCH, 0 * COUNTS_PER_INCH, 0.75, 180, 9 * COUNTS_PER_INCH,25);
        goToPosition(3,0 * COUNTS_PER_INCH, 0 * COUNTS_PER_INCH, 0.25, 180, 9 * COUNTS_PER_INCH,1);
        goToPosition(2,0 * COUNTS_PER_INCH, 21 * COUNTS_PER_INCH, 0.25, 180, 9 * COUNTS_PER_INCH,.5);

        goToPosition(1,24 * COUNTS_PER_INCH,24 * COUNTS_PER_INCH,.25,180,.25 * COUNTS_PER_INCH,0.5);
        goToPosition(2,24 * COUNTS_PER_INCH,24 * COUNTS_PER_INCH,.08,180,.25 * COUNTS_PER_INCH,0.5);
        //goToPosition(0 * COUNTS_PER_INCH,24 * COUNTS_PER_INCH,.1,90,.25 * COUNTS_PER_INCH,0.25);

        while(opModeIsActive()){
            //Display Global (x, y, theta) coordinates
            telemetry.addData("X Position", globalPositionUpdate.returnXCoordinate() / COUNTS_PER_INCH);
            telemetry.addData("Y Position", globalPositionUpdate.returnYCoordinate() / COUNTS_PER_INCH);
            telemetry.addData("Orientation (Degrees)", globalPositionUpdate.returnOrientation());

            telemetry.addData("Vertical left encoder position", verticalLeft.getCurrentPosition());
            telemetry.addData("Vertical right encoder position", verticalRight.getCurrentPosition());
            telemetry.addData("horizontal encoder position", horizontal.getCurrentPosition());

            telemetry.addData("Thread Active", positionThread.isAlive());
            telemetry.update();
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

    public void goToPosition(int targetxyr, double targetX, double targetY, double robotPower, double targetRotation, double distanceThreshold, double angleThreshold) {

        double distanceToXTarget = targetX - globalPositionUpdate.returnXCoordinate();
        double distanceToYTarget = targetY - globalPositionUpdate.returnYCoordinate();
        double distance = Math.hypot(distanceToXTarget, distanceToYTarget);
        double pivotCorrection = targetRotation - globalPositionUpdate.returnOrientation();
        double countMe = 0;
        boolean targetfound = false;
        while (opModeIsActive() && (targetfound == false)) { //(distance > distanceThreshold) || (Math.abs(pivotCorrection) > angleThreshold) ) {

            //We are reversing x and y because 0 degrees is forwards rather than to the right
            double robotMovementAngle = Math.toDegrees(Math.atan2(distanceToXTarget, distanceToYTarget));
            double robotMovementXComponent = calculateX(robotMovementAngle, robotPower);
            double robotMovementYComponent = calculateY(robotMovementAngle, robotPower);
            double robotMovementRComponent = CalculateR(pivotCorrection,angleThreshold,robotPower);
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
                    if (Math.abs(distanceToXTarget) > distanceThreshold) {
                        targetfound = false;
                    } else {
                        targetfound = true;
                    }
                    break;
                case 2:
                    if (Math.abs(distanceToYTarget) > distanceThreshold) {
                        targetfound = false;
                    } else {
                        targetfound = true;
                    }
                    break;
                case 3:
                    if (Math.abs(pivotCorrection) > angleThreshold) {
                        targetfound = false;
                    } else {
                        targetfound = true;
                    }
                    break;
                case 0:
                    if ((distance > distanceThreshold) || (Math.abs(pivotCorrection) > angleThreshold)) {
                        targetfound = false;
                    } else {
                        targetfound = true;
                    }
                    break;
            }

            telemetry.addData("X Position", globalPositionUpdate.returnXCoordinate() / COUNTS_PER_INCH);
            telemetry.addData("Y Position", globalPositionUpdate.returnYCoordinate() / COUNTS_PER_INCH);
            telemetry.addData("Orientation (Degrees)", globalPositionUpdate.returnOrientation());
            telemetry.addData("Targeting", targetxyr);
            telemetry.addData("distanceToXTarget", distanceToXTarget);
            telemetry.addData("distanceToYTarget", distanceToYTarget);
            telemetry.addData("distance", distance);

            telemetry.addData("robotMovementAngle", robotMovementAngle);

            telemetry.addData("robotMovementXComponent", robotMovementXComponent);
            telemetry.addData("robotMovementYComponent", robotMovementYComponent);
            telemetry.addData("pivotCorrection", pivotCorrection);

            telemetry.update();
        }

        setMecanumMotorPowers(0, 0, 0);
    }

    public void setMecanumMotorPowers(double x, double y, double r) {
        //We reverse these because they're on the right side
        if ((x + r > 1) || (y + r > 1)){
            double sumXYMax = 1 - r;
            x = x * sumXYMax;
            y = y * sumXYMax;
        }
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

    private double CalculateR(double desiredAngle, double angleThreshold, double speed) {
        double motorMovement;
        if(Math.abs(desiredAngle) > angleThreshold){
            if(desiredAngle > 0) {
                motorMovement = 1;
            }else{
                motorMovement = -1;
            }
        }else{
            motorMovement = 0;
        }
        return motorMovement * (speed/2);
    }

}
