package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Right Close", group = GenericOpMode.GROUP_SKYSTONE)
public class AutonRightClose extends GenericOpMode {
    Robot<AutonRightClose> robot = new Robot<>(this);
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
