package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.util.GenericOpMode;
import org.firstinspires.ftc.teamcode.util.Robot;

@TeleOp(name = "Intake Test", group = GenericOpMode.GROUP_TESTINNG)
public class IntakeTest extends GenericOpMode {
    private Robot robot = new Robot(this);

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

            boolean atLowLimit = robot.liftMotor.getCurrentPosition() < 0;
            boolean atHighLimit = robot.liftMotor.getCurrentPosition() > 4100;

            if (atLowLimit && lY > 0)
                robot.liftMotor.setPower(lY * 1);
            else if (atHighLimit && lY < 0)
                robot.liftMotor.setPower(lY * 1);
            else if (!atLowLimit && !atHighLimit)
                robot.liftMotor.setPower(lY * 1);
            else
                robot.liftMotor.setPower(0);

            robot.extendMotor.setPower(rY);

            telemetry.addData("Left Y", lY);
            telemetry.addData("atLowLimit", atLowLimit);
            telemetry.addData("atHighLimit", atHighLimit);
            telemetry.addData("Right Y", rY);
            telemetry.addData("liftMotor Pos", robot.liftMotor.getCurrentPosition());
            telemetry.update();
        }
    }
}
