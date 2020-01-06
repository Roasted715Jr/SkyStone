package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.util.GenericOpMode;
import org.firstinspires.ftc.teamcode.util.Robot;

@TeleOp(name = "Distance Sensor Test", group = GenericOpMode.GROUP_TESTINNG)
//@Disabled
public class DistanceSensorTest extends GenericOpMode {
    Robot<DistanceSensorTest> robot = new Robot<>(this);

    double distance;
    long start, finish;

    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);

        waitForStart();

        while (opModeIsActive()) {
            start = System.currentTimeMillis();

            //Goes from 0.25 in to 3 in for V3
            distance = robot.distanceSensor.getDistance(DistanceUnit.INCH);

            finish = System.currentTimeMillis();

            telemetry.addData("Distance", distance);
            //No more than 10 ms for V3
            //No more than 45 ms for 2M
            telemetry.addData("Time", finish - start);

            telemetry.update();
        }
    }
}
