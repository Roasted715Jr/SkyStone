package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.util.GenericOpMode;
import org.firstinspires.ftc.teamcode.util.Robot;

@TeleOp(name = "Intake Test", group = GenericOpMode.GROUP_TESTINNG)
public class IntakeTest extends GenericOpMode {
    Robot robot = new Robot(this);

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);

        waitForStart();

        while (opModeIsActive()) {
            double y = -gamepad1.left_stick_y;
            robot.rIntake.setPower(y);
            robot.lIntake.setPower(y);

            telemetry.addData("Joystick val", y);
            telemetry.update();
        }
    }
}
