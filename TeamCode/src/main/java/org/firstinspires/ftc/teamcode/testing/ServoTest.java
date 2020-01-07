package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.util.GenericOpMode;
import org.firstinspires.ftc.teamcode.util.Robot;

@TeleOp(name = "ServoTest", group = GenericOpMode.GROUP_SKYSTONE)
@Disabled
public class ServoTest extends GenericOpMode {
    private Robot robot = new Robot(this);

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
