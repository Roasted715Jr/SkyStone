package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "Left Far", group = "SkyStone")
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
