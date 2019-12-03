package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "Movement Test", group = "Testing")
@Disabled
public class MovementTest extends GenericOpMode {
    Hardware<MovementTest> robot = new Hardware<>(this);

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);

        waitForStart();

//        robot.rampMecanumMotors(1, 1, 0, true);

//        robot.rampMotors(robot.frMotor, 1, true);

//        robot.goDistance(0, 500, 0);

//        robot.rampMecanumMotors(0, 1, 0, true);
//        robot.rampMecanumMotors(0, 0, 0, false);
//        robot.rampMecanumMotors(0, -1, 0, true);
//        robot.rampMecanumMotors(0, 0, 0, false);

//        robot.goDistance(0, 48, 0, 0, 1, 0);
//        robot.goDistance(0, 56, 0);
        robot.setMecanumMotorRunmodes(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        robot.goDistance(0, 12, 0, 0, 1, 0);

        //Tested with a 50 count stop in advance to reaching the target position
        //Expected 12, went ~10.5 (~10 OFF)
        //Expected 24, went 22 (~40 over)
        //Expected 36, went 35 (~50 over)
        //Expected 48, went 47 (~40 over)
        robot.goDistance(-48, 0, 0, 1, 0, 0);
        robot.setMecanumMotorPowers(0, 0, 0);

        while (opModeIsActive()) {
            addTelemetry("flMotor", robot.flMotor.getCurrentPosition() + " / " + robot.flMotor.getTargetPosition());
            addTelemetry("blMotor", robot.blMotor.getCurrentPosition() + " / " + robot.blMotor.getTargetPosition());
            addTelemetry("frMotor", robot.frMotor.getCurrentPosition() + " / " + robot.frMotor.getTargetPosition());
            addTelemetry("brMotor", robot.brMotor.getCurrentPosition() + " / " + robot.brMotor.getTargetPosition());
            updateTelemetry();
        }
    }
}
