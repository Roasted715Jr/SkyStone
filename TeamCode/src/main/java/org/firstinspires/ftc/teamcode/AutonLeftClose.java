package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "Left Close", group = "SkyStone")
public class AutonLeftClose extends GenericOpMode {
    Hardware<AutonLeftClose> robot = new Hardware<>(this);
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
