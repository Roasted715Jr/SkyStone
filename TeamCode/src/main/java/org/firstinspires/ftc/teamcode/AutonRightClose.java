package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Right Close", group = GenericOpMode.GROUP_SKYSTONE)
public class AutonRightClose extends GenericOpMode {
    Hardware<AutonRightClose> robot = new Hardware<>(this);
    AutonProcedures<AutonRightClose> autonProcedures = new AutonProcedures<>();

    @Override
    public void runOpMode() throws InterruptedException {
        autonProcedures.init(robot, hardwareMap, this);

        waitForStart();

        autonProcedures.simpleAuton(true, false);

        while (opModeIsActive()) {

        }
    }
}
