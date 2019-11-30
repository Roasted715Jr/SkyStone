package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Autonomous(name = "Movement Test", group = "Testing")
@Disabled
public class MovementTest extends GenericOpMode {
    Hardware<MovementTest> robot = new Hardware<>(this);

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);

        waitForStart();

//        robot.rampMecanumMotors(1, 1, 0, true);

//        robot.rampMotor(robot.frMotor, 1, true);

//        robot.goDistance(0, 500, 0);

//        robot.rampMecanumMotors(0, 1, 0, true);
//        robot.rampMecanumMotors(0, 0, 0, false);
//        robot.rampMecanumMotors(0, -1, 0, true);
//        robot.rampMecanumMotors(0, 0, 0, false);

        robot.goDistance(0, 54, 0);
//        robot.goDistance(0, -48, 0);

        while (opModeIsActive()) {

        }
    }
}
