package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "ColorSensorTest", group = "SkyStone")
public class ColorSensorTest extends GenericOpMode {
    Hardware<ColorSensorTest> robot = new Hardware<>(this);

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



            if (inRange(robot.rColor.red() + robot.rColor.green() + robot.rColor.blue(), 1000, 10000))
                foundSkyStone = true;

            telemetry.addData("Found Skystone", foundSkyStone);
            telemetry.update();
        }
    }

    private boolean inRange(double val, double min, double max) {
        return min < val && val < max;
    }
}
