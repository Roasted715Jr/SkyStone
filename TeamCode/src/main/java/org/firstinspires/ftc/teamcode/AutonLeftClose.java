package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "Left Close", group = "SkyStone")
public class AutonLeftClose extends GenericOpMode {
    Hardware<AutonLeftClose> robot = new Hardware<>(this);

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);

        waitForStart();

        robot.setMecanumMotorRunmodes(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.goDistance(0, 4, 0, 0, 1, 0);
        robot.goDistance(36, 0, 0, 1, 0, 0);

        robot.setMecanumMotorPowers(0, 0, 0);

        while (opModeIsActive()) {
            telemetry.update();
        }
    }
}
