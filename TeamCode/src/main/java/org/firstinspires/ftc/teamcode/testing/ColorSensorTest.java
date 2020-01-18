package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.util.GenericOpMode;
import org.firstinspires.ftc.teamcode.util.Robot;

@TeleOp(name = "Color Sensor Test", group = GenericOpMode.GROUP_TESTINNG)
//@Disabled
public class ColorSensorTest extends GenericOpMode {
    private Robot robot = new Robot(this);

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);

        boolean foundSkyStone = false;

        waitForStart();

        while (opModeIsActive()) {
            foundSkyStone = false;

            telemetry.addData("Left Red", robot.lColor.red());
            telemetry.addData("Left Green", robot.lColor.green());
            telemetry.addData("Left Blue", robot.lColor.blue());

            telemetry.addData("Right Red", robot.rColor.red());
            telemetry.addData("Right Green", robot.rColor.green());
            telemetry.addData("Right Blue", robot.rColor.blue());

            //500, 1000, 600 marked

            //5000 7500 1000 yellow

//            if (inRange(robot.rColor.red() + robot.rColor.green() + robot.rColor.blue(), 1000, 10000))
//            if (inRange(robot.rColor.red(), 500, 2500) && inRange(robot.rColor.green(), 1000, 3500) && inRange(robot.rColor.blue(), 600, 2000))
//                foundSkyStone = true;
//
//            telemetry.addData("Found Skystone", foundSkyStone);
            telemetry.update();
        }
    }

    private boolean inRange(double val, double min, double max) {
        return min < val && val < max;
    }
}
