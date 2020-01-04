package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "TouchSensorTest", group = GenericOpMode.GROUP_SKYSTONE)
@Disabled
public class TouchSensorTest extends GenericOpMode {
    Robot<TouchSensorTest> robot = new Robot<>(this);

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);

        waitForStart();

        while (opModeIsActive()) {

        }
    }
}
