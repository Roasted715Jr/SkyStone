package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Right Far", group = GenericOpMode.GROUP_SKYSTONE)
public class AutonRightFar extends GenericOpMode {
    Hardware<AutonRightFar> robot = new Hardware<>(this);
    AutonProcedures<AutonRightFar> autonProcedures = new AutonProcedures<>();

    @Override
    public void runOpMode() throws InterruptedException {
        autonProcedures.init(robot, hardwareMap, this);

        waitForStart();

        autonProcedures.simpleAuton(true, true);

        while (opModeIsActive()) {

        }
    }
}
