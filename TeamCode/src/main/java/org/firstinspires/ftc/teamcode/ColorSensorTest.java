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

            if (robot.rColor.red() < 50 && robot.rColor.green() < 50)
                foundSkyStone = true;

            telemetry.addData("Found Skystone", foundSkyStone);
            telemetry.update();
        }
    }
}
