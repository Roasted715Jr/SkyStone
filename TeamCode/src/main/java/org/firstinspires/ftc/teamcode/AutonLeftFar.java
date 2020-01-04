package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Left Far", group = GenericOpMode.GROUP_SKYSTONE)
public class AutonLeftFar extends GenericOpMode {
    Robot<AutonLeftFar> robot = new Robot<>(this);
    AutonProcedures<AutonLeftFar> autonProcedures = new AutonProcedures<>();

    @Override
    public void runOpMode() throws InterruptedException {
        autonProcedures.init(robot, hardwareMap, this);

        waitForStart();

        autonProcedures.simpleAuton(false, true);

        while (opModeIsActive()) {
            telemetry.update();
        }
    }
}
