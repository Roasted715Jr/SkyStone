package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Left Close", group = GenericOpMode.GROUP_SKYSTONE)
public class AutonLeftClose extends GenericOpMode {
    Robot<AutonLeftClose> robot = new Robot<>(this);
    AutonProcedures<AutonLeftClose> autonProcedures = new AutonProcedures<>();

    @Override
    public void runOpMode() throws InterruptedException {
        autonProcedures.init(robot, hardwareMap, this);

        waitForStart();

        autonProcedures.simpleAuton(false, false);

        while (opModeIsActive()) {
            telemetry.update();
        }
    }
}
