package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "TouchSensorTest", group = "SkyStone")
public class TouchSensorTest extends GenericOpMode {
    Hardware<TouchSensorTest> robot = new Hardware<>(this);

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);

        waitForStart();

        while (opModeIsActive()) {

        }
    }
}