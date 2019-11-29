package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "Movement Test", group = "SkyStone")
//@Disabled
public class MovementTest extends GenericOpMode {
    Hardware<MovementTest> robot = new Hardware<>(this);

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);

        waitForStart();

        robot.moveDistance(0, 24);

        while (opModeIsActive()) {
            telemetry.addData("Motor Value", robot.frMotor.getCurrentPosition());
            telemetry.update();
        }
    }
}
