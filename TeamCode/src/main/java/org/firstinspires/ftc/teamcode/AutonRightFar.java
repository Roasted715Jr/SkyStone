package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "Right Far", group = "SkyStone")
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
