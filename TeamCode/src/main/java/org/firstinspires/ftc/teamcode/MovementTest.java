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

        robot.rampMotor(robot.frMotor, 1, true);

//        robot.moveDistance(0, 5, 0);

        while (opModeIsActive()) {

        }
    }
}
