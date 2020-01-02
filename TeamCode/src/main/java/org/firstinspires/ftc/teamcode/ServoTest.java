package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "ServoTest", group = GenericOpMode.GROUP_SKYSTONE)
@Disabled
public class ServoTest extends GenericOpMode {
    Hardware<ServoTest> robot = new Hardware<>(this);

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);

        waitForStart();

        robot.clawServo.setPosition(0.25);
        Thread.sleep(2000);
        robot.clawServo.setPosition(1);

        while (opModeIsActive()) {

        }
    }
}
