package org.firstinspires.ftc.teamcode.skystone;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.util.GenericOpMode;
import org.firstinspires.ftc.teamcode.util.Robot;

@Autonomous(name = "Right Close", group = GenericOpMode.GROUP_SKYSTONE)
public class AutonRightClose extends GenericOpMode {
    private Robot robot = new Robot(this);
    private AutonProcedures autonProcedures = new AutonProcedures();

    @Override
    public void runOpMode() throws InterruptedException {
        autonProcedures.init(robot, hardwareMap, this);

        waitForStart();

        autonProcedures.simpleAuton(true, false, 27000);

        while (opModeIsActive()) {

        }

        robot.stop();
    }
}
