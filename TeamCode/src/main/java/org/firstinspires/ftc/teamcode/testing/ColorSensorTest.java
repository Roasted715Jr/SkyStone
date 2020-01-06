package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.util.GenericOpMode;
import org.firstinspires.ftc.teamcode.util.Robot;

@TeleOp(name = "Color Sensor Test", group = GenericOpMode.GROUP_TESTINNG)
public class ColorSensorTest extends GenericOpMode {
    Robot<ColorSensorTest> robot = new Robot<>(this);

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);

        boolean foundSkyStone = false;

        waitForStart();

        while (opModeIsActive()) {
            foundSkyStone = false;

            telemetry.addData("Red", robot.rColor.red());
            telemetry.addData("Green", robot.rColor.green());
            telemetry.addData("Blue", robot.rColor.blue());

            //500, 1000, 600 marked

            //5000 7500 1000 yellow

//            if (inRange(robot.rColor.red() + robot.rColor.green() + robot.rColor.blue(), 1000, 10000))
            if (inRange(robot.rColor.red(), 500, 2500) && inRange(robot.rColor.green(), 1000, 3500) && inRange(robot.rColor.blue(), 600, 2000))
                foundSkyStone = true;

            telemetry.addData("Found Skystone", foundSkyStone);
            telemetry.update();
        }
    }

    private boolean inRange(double val, double min, double max) {
        return min < val && val < max;
    }
}
