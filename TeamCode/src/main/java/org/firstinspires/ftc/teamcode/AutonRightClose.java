package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "Right Close", group = "SkyStone")
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
