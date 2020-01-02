package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Left Far", group = GenericOpMode.GROUP_SKYSTONE)
public class AutonLeftFar extends GenericOpMode {
    Hardware<AutonLeftFar> robot = new Hardware<>(this);
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
