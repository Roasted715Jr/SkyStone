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
            if (gamepad1.right_trigger > 0.25) {
                robot.rIntake.setPower(1);
                robot.lIntake.setPower(1);
            } else if (gamepad1.left_trigger > 0.25) {
                robot.rIntake.setPower(-1);
                robot.lIntake.setPower(-1);
            } else {
                robot.rIntake.setPower(0);
                robot.lIntake.setPower(0);
            }

            double lY = -gamepad1.left_stick_y;
            double rY = -gamepad1.right_stick_y;
            robot.liftMotor.setPower(lY);
            robot.extendMotor.setPower(rY);

            telemetry.addData("Left Y", lY);
            telemetry.addData("Right Y", rY);
            telemetry.update();
        }
    }
}
