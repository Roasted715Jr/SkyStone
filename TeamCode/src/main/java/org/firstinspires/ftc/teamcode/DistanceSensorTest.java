package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "Distance Sensor Test", group = "Android Studio")
//@Disabled
public class DistanceSensorTest extends GenericOpMode {
    Hardware<DistanceSensorTest> robot = new Hardware<>(this);

    double distance;
    long start, finish;

    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);

        waitForStart();

        while (opModeIsActive()) {
            start = System.currentTimeMillis();

            //Goes from 0.25 in to 3 in for V3
            distance = robot.distanceSensor.getDistance(DistanceUnit.METER);

            finish = System.currentTimeMillis();

            telemetry.addData("Distance", distance);
            //No more than 10 ms for V3
            //No more than 45 ms for 2M
            telemetry.addData("Time", finish - start);

            telemetry.update();
        }
    }
}
