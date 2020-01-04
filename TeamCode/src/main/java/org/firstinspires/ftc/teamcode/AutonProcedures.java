package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;

import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;

public class AutonProcedures<T extends GenericOpMode> {
    private Robot robot;
    private HardwareMap hardwareMap;
    private T runningOpMode;
    boolean running = true; //Just for testing purposes

    //These align with their respective coordinate quadrants
    private static final int START_BLUE_FOUNDATION = 1;
    private static final int START_BLUE_SKYSTONE = 2;
    private static final int START_RED_SKYSTONE = 3;
    private static final int START_RED_FOUNDATION = 4;
    private int startSpot = 0;

    //1 is closest to the wall
    private int blockPos = 0;

    //Stuff for Vuforia
//    private static final boolean PHONE_IS_PORTRAIT = false;

    // Since ImageTarget trackables use mm to specifiy their dimensions, we must use mm for all the physical dimension.
    // We will define some constants and conversions here
    private static final float mmPerInch        = 25.4f;
//    private static final float mmTargetHeight   = (6) * mmPerInch;          // the height of the center of the target image above the floor

    // Constant for Stone Target
//    private static final float stoneZ = 2.00f * mmPerInch;

    // Constants for the center support targets
//    private static final float bridgeZ = 6.42f * mmPerInch;
//    private static final float bridgeY = 23 * mmPerInch;
//    private static final float bridgeX = 5.18f * mmPerInch;
//    private static final float bridgeRotY = 59;                                 // Units are degrees
//    private static final float bridgeRotZ = 180;

    // Constants for perimeter targets
//    private static final float halfField = 72 * mmPerInch;
//    private static final float quadField  = 36 * mmPerInch;

//     Class Members
    private OpenGLMatrix lastLocation;
    private VuforiaLocalizer vuforia;
    private boolean targetVisible = false;
//    private float phoneXRotate    = 0;
//    private float phoneYRotate    = 0;
//    private float phoneZRotate    = 0;

//    VuforiaTrackables targetsSkyStone;
//    VuforiaTrackable stoneTarget;
//    VuforiaTrackable blueRearBridge;
//    VuforiaTrackable redRearBridge;
//    VuforiaTrackable redFrontBridge;
//    VuforiaTrackable blueFrontBridge;
//    VuforiaTrackable red1;
//    VuforiaTrackable red2;
//    VuforiaTrackable front1;
//    VuforiaTrackable front2;
//    VuforiaTrackable blue1;
//    VuforiaTrackable blue2;
//    VuforiaTrackable rear1;
//    VuforiaTrackable rear2;

//    VuforiaTrackable target;

    // Next, translate the camera lens to where it is on the robot.
    // In this example, it is centered (left to right), but forward of the middle of the robot, and above ground level.
//    final float CAMERA_FORWARD_DISPLACEMENT  = 4.0f * mmPerInch;   // eg: Camera is 4 Inches in front of robot center
//    final float CAMERA_VERTICAL_DISPLACEMENT = 8.0f * mmPerInch;   // eg: Camera is 8 Inches above ground
//    final float CAMERA_LEFT_DISPLACEMENT     = 0;     // eg: Camera is ON the robot's center line

    // For convenience, gather together all the trackable objects in one easily-iterable collection */
    List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();

    private VectorF translation;
    private Orientation rotation;

    void init(Robot robot, HardwareMap hardwareMap, T runningOpMode) {
        this.robot = robot;
        this.hardwareMap = hardwareMap;
        this.runningOpMode = runningOpMode;

        robot.init(hardwareMap);

//        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
//        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
//        //For if we don't need to see what is in the camera frame
//        // VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
//        parameters.vuforiaLicenseKey = Robot.VUFORIA_LICENSE_KEY;
//        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
//        vuforia = ClassFactory.getInstance().createVuforia(parameters);
//
//        targetsSkyStone = this.vuforia.loadTrackablesFromAsset("Skystone");
//
//        stoneTarget = targetsSkyStone.get(0);
//        stoneTarget.setName("Stone Target");
//        blueRearBridge = targetsSkyStone.get(1);
//        blueRearBridge.setName("Blue Rear Bridge");
//        redRearBridge = targetsSkyStone.get(2);
//        redRearBridge.setName("Red Rear Bridge");
//        redFrontBridge = targetsSkyStone.get(3);
//        redFrontBridge.setName("Red Front Bridge");
//        blueFrontBridge = targetsSkyStone.get(4);
//        blueFrontBridge.setName("Blue Front Bridge");
//        red1 = targetsSkyStone.get(5);
//        red1.setName("Red Perimeter 1");
//        red2 = targetsSkyStone.get(6);
//        red2.setName("Red Perimeter 2");
//        front1 = targetsSkyStone.get(7);
//        front1.setName("Front Perimeter 1");
//        front2 = targetsSkyStone.get(8);
//        front2.setName("Front Perimeter 2");
//        blue1 = targetsSkyStone.get(9);
//        blue1.setName("Blue Perimeter 1");
//        blue2 = targetsSkyStone.get(10);
//        blue2.setName("Blue Perimeter 2");
//        rear1 = targetsSkyStone.get(11);
//        rear1.setName("Rear Perimeter 1");
//        rear2 = targetsSkyStone.get(12);
//        rear2.setName("Rear Perimeter 2");
//
//        allTrackables.addAll(targetsSkyStone);
//
//        // Set the position of the Stone Target.  Since it's not fixed in position, assume it's at the field origin.
//        // Rotated it to to face forward, and raised it to sit on the ground correctly.
//        // This can be used for generic target-centric approach algorithms
//        stoneTarget.setLocation(OpenGLMatrix
//                .translation(0, 0, stoneZ)
//                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));
//
//        //Set the position of the bridge support targets with relation to origin (center of field)
//        blueFrontBridge.setLocation(OpenGLMatrix
//                .translation(-bridgeX, bridgeY, bridgeZ)
//                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, bridgeRotY, bridgeRotZ)));
//
//        blueRearBridge.setLocation(OpenGLMatrix
//                .translation(-bridgeX, bridgeY, bridgeZ)
//                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, -bridgeRotY, bridgeRotZ)));
//
//        redFrontBridge.setLocation(OpenGLMatrix
//                .translation(-bridgeX, -bridgeY, bridgeZ)
//                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, -bridgeRotY, 0)));
//
//        redRearBridge.setLocation(OpenGLMatrix
//                .translation(bridgeX, -bridgeY, bridgeZ)
//                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, bridgeRotY, 0)));
//
//        //Set the position of the perimeter targets with relation to origin (center of field)
//        red1.setLocation(OpenGLMatrix
//                .translation(quadField, -halfField, mmTargetHeight)
//                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180)));
//
//        red2.setLocation(OpenGLMatrix
//                .translation(-quadField, -halfField, mmTargetHeight)
//                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180)));
//
//        front1.setLocation(OpenGLMatrix
//                .translation(-halfField, -quadField, mmTargetHeight)
//                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0 , 90)));
//
//        front2.setLocation(OpenGLMatrix
//                .translation(-halfField, quadField, mmTargetHeight)
//                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 90)));
//
//        blue1.setLocation(OpenGLMatrix
//                .translation(-quadField, halfField, mmTargetHeight)
//                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0)));
//
//        blue2.setLocation(OpenGLMatrix
//                .translation(quadField, halfField, mmTargetHeight)
//                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0)));
//
//        rear1.setLocation(OpenGLMatrix
//                .translation(halfField, quadField, mmTargetHeight)
//                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0 , -90)));
//
//        rear2.setLocation(OpenGLMatrix
//                .translation(halfField, -quadField, mmTargetHeight)
//                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));
//
//        // We need to rotate the camera around it's long axis to bring the correct camera forward.
//        if (parameters.cameraDirection == VuforiaLocalizer.CameraDirection.BACK) {
//            phoneYRotate = -90;
//        } else {
//            phoneYRotate = 90;
//        }
//
//        // Rotate the phone vertical about the X axis if it's in portrait mode
//        if (PHONE_IS_PORTRAIT) {
//            phoneXRotate = 90 ;
//        }
//
//        OpenGLMatrix robotFromCamera = OpenGLMatrix
//                .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
//                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES, phoneYRotate, phoneZRotate, phoneXRotate));
//
//        /**  Let all the trackable listeners know where the phone is.  */
//        for (VuforiaTrackable trackable : allTrackables) {
//            ((VuforiaTrackableDefaultListener) trackable.getListener()).setPhoneInformation(robotFromCamera, parameters.cameraDirection);
//        }
    }

    void startWithStartSpot(int startSpot) {
        this.startSpot = startSpot;
        start();
    }

    void start() {
//        targetsSkyStone.activate();

//        while (target == null)
//            target = updatePosition();

//        runningOpMode.addTelemetry("This is the first step");
//        runningOpMode.updateTelemetry();

        //Find our starting spot
//        startSpot = getStartSpot();
//        startSpot = 2;

        //Interpret our starting spot
//        VectorF skyStonePos;
        switch (startSpot) {
            case START_BLUE_FOUNDATION:
                runFoundationSide(false);
                break;
            case START_BLUE_SKYSTONE:
//                skyStonePos = getSkyStonePosition();

//                runningOpMode.addTelemetry("Moving to SkyStone");
//                runningOpMode.updateTelemetry();
//                robot.goDistance(0, 54, 0, 0, 1, 0);
                runSkystoneSide(false);
//                robot.goDistance(some amount left, maybe backwards, no turning, x, y, r);
//                robot.goDistance(reverse of what we just did but more, maybe forwards, no turning, x, y, r);
//                approachSkyStone();
                break;
            case START_RED_SKYSTONE:
//                skyStonePos = getSkyStonePosition();
                runSkystoneSide(true);
                break;
            case START_RED_FOUNDATION:
                runFoundationSide(true);
                break;
        }

//        Move to marked Skystone and pick it up
//        Move backwards, turn towards the audience, and move backwards past the bridge

//        targetsSkyStone.deactivate();
        runningOpMode.addTelemetry("Done");
        runningOpMode.updateTelemetry();
    }

    private VuforiaTrackable updatePosition() {
        VuforiaTrackable target = null;

        // check all the trackable targets to see which one (if any) is visible.
        targetVisible = false;
        for (VuforiaTrackable trackable : allTrackables) {
            if (((VuforiaTrackableDefaultListener)trackable.getListener()).isVisible()) {
                runningOpMode.addTelemetry("Visible Target", trackable.getName());
                targetVisible = true;
                target = trackable;

                // getUpdatedRobotLocation() will return null if no new information is available since
                // the last time that call was made, or if the trackable is not currently visible.
                OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener)trackable.getListener()).getUpdatedRobotLocation();
                if (robotLocationTransform != null) {
                    lastLocation = robotLocationTransform;
                }
                break;
            }
        }

        // Provide feedback as to where the robot is located (if we know).
        if (targetVisible) {
            // express position (translation) of robot in inches.
            translation = lastLocation.getTranslation();
            runningOpMode.addTelemetry("Pos (in)", "{X, Y, Z} = %.1f, %.1f, %.1f",
                    translation.get(0) / mmPerInch, translation.get(1) / mmPerInch, translation.get(2) / mmPerInch);

            // express the rotation of the robot in degrees.
            rotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);
            runningOpMode.addTelemetry("Rot (deg)", "{Roll, Pitch, Heading} = %.0f, %.0f, %.0f", rotation.firstAngle, rotation.secondAngle, rotation.thirdAngle);
        }
        else {
            runningOpMode.addTelemetry("Visible Target", "none");
        }

//        runningOpMode.updateTelemetry();

        return target;
    }

    private int getStartSpot() {
        //Wait for the target to become identified
        while (updatePosition() == null) {
            runningOpMode.addTelemetry("Finding Starting Spot");
            runningOpMode.updateTelemetry();
        }

        int pos = 0;
        float x = translation.get(0);
        float y = translation.get(1);

        if (y > 0) {
            if (x > 0)
                pos = 1;
            else
                pos = 2;
        } else {
            if (x > 0)
                pos = 4;
            else
                pos = 3;
        }

        return pos;
    }

    private void runFoundationSide(boolean isRed) {

    }

    private void runSkystoneSide(boolean isRed) {
        //Get the Skystone
        approachSkyStone();

        if (foundSkyStone(robot.rColor))
            blockPos = isRed ? 3 : 1;
        else if (foundSkyStone(robot.lColor))
            blockPos = 2;
        else
            blockPos = isRed ? 1 : 3;

        runningOpMode.addTelemetry("Block Position", blockPos);
        runningOpMode.updateTelemetry();

        //Go to the SkyStone and grab it
        centerOnSkyStone();
        robot.blockServo.setPosition(1);
        try {
            Thread.sleep(3000);
        } catch (Exception e) {

        }

        //Try 5 ft for now
        deliverSkyStone(isRed, 60);
        
        //Move an extra 2 ft over
        robot.goDistance(-84, 0, 0, -1, 0 ,0);
        approachSkyStone();
        centerOnSkyStone();
        //Move past the previous
        deliverSkyStone(isRed, 90);
    }

    private void approachSkyStone() {
        robot.setMecanumMotorPowers(0, 0.25, 0);
        while (robot.distanceSensor.getDistance(DistanceUnit.INCH) > 3) {
            runningOpMode.addTelemetry("Distance", robot.distanceSensor.getDistance(DistanceUnit.INCH));
            runningOpMode.updateTelemetry();
        }
        robot.setMecanumMotorPowers(0, 0, 0);
    }

    private void centerOnSkyStone() {
        while (!(foundSkyStone(robot.rColor) && foundSkyStone(robot.lColor))) {
            //Run this inside the loop so that when we go for the second SkyStone we don't move if we're already lined up
            robot.setMecanumMotorPowers(foundSkyStone(robot.rColor) ? 0.25 : -0.25, 0, 0);

            runningOpMode.addTelemetry("Centering on block position " + blockPos);
            runningOpMode.updateTelemetry();
        }

        robot.setMecanumMotorPowers(0, 0, 0);
    }

    private void deliverSkyStone(boolean isRed, int distance) {
        robot.goDistance(0, -5, 0, 0, -1, 0);

        int dir = isRed ? 1 : -1;
        robot.goDistance(distance * dir, 0, 0, dir, 0 ,0);

        robot.goDistance(0, 5, 0, 0, 1, 0);
        robot.blockServo.setPosition(0);
        try {
            Thread.sleep(3000);
        } catch (Exception e) {

        }
        robot.goDistance(0, -5, 0, 0, -1, 0);
    }

    private boolean foundSkyStone(RevColorSensorV3 color) {
        return inRange(color.red(), 500, 2500) && inRange(color.green(), 1000, 3500) && inRange(color.blue(), 600, 2000);
    }

    void simpleAuton(boolean isRight, boolean isFar) {
        robot.setMecanumMotorRunmodes(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.goDistance(0, isFar ? 22 : 4, 0, 0, 1, 0);
        robot.goDistance(isRight ? -36 : 36, 0, 0, isRight ? -1 : 1, 0, 0);

        robot.setMecanumMotorPowers(0, 0, 0);
    }

    private boolean inRange(double val, double min, double max) {
        return min < val && val < max;
    }
}
