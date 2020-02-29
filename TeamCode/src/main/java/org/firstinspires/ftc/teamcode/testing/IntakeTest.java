package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.util.GenericOpMode;
import org.firstinspires.ftc.teamcode.util.Robot;

@TeleOp(name = "Intake Test", group = GenericOpMode.GROUP_TESTINNG)
public class IntakeTest extends GenericOpMode {
    private static final double SPEED_MAX = 0.75;
    private static final double SPEED_MIN = 0.25;

    private Robot robot = new Robot(this);

    @Override
    public void runOpMode() throws InterruptedException {
        boolean aPressed = false, multiplierToggle = false;
        double x, y, r;
        double speedMultiplier;

        robot.init(hardwareMap);

        waitForStart();

        while (opModeIsActive()) {
            //Controller #1
            x = gamepad1.left_stick_x;
            y = -gamepad1.left_stick_y;
            r = gamepad1.right_stick_x;

            if (gamepad1.a) {
                if (!aPressed)
                    multiplierToggle = !multiplierToggle;
                aPressed = true;
            } else
                aPressed = false;

            if (gamepad1.x)
                robot.moveHooks(true);
            else if (gamepad1.b)
                robot.moveHooks(false);

            speedMultiplier = multiplierToggle ? SPEED_MIN : SPEED_MAX;

            robot.setMecanumMotorPowers(x * speedMultiplier, y * speedMultiplier, r * speedMultiplier);

            //Controller #2
            if (gamepad2.right_trigger > 0.25) {
                robot.rIntake.setPower(1);
                robot.lIntake.setPower(1);
            } else if (gamepad2.left_trigger > 0.25) {
                robot.rIntake.setPower(-1);
                robot.lIntake.setPower(-1);
            } else {
                robot.rIntake.setPower(0);
                robot.lIntake.setPower(0);
            }

            double lY = -gamepad2.left_stick_y;
            double rY = -gamepad2.right_stick_y;

            boolean atLowLimit = robot.liftMotor.getCurrentPosition() <= 0;
            boolean atHighLimit = robot.liftMotor.getCurrentPosition() >= 4300;

            if (atLowLimit && lY > 0)
                robot.liftMotor.setPower(lY);
            else if (atHighLimit && lY < 0)
                robot.liftMotor.setPower(lY);
            else if (!atLowLimit && !atHighLimit)
                robot.liftMotor.setPower(lY);
            else
                robot.liftMotor.setPower(0);

            boolean atFarLimit = robot.extendMotor.getCurrentPosition() > 1750;
            boolean atCloseLimit = robot.extendMotor.getCurrentPosition() < 50;

            if (gamepad2.right_bumper)
                robot.grabberServo.setPosition(1);
            else if (gamepad2.left_bumper)
                robot.grabberServo.setPosition(0);

            if (robot.grabberServo.getPosition() > 0.75) {
                if (atCloseLimit && rY > 0)
                    robot.extendMotor.setPower(rY);
                else if (atFarLimit && rY < 0)
                    robot.extendMotor.setPower(rY);
                else if (!atCloseLimit && !atFarLimit)
                    robot.extendMotor.setPower(rY);
                else
                    robot.extendMotor.setPower(0);
            }

            telemetry.addData("speedMultiplier", speedMultiplier);
            telemetry.addData("x", "%.5f", x);
            telemetry.addData("y", "%.5f", y);
            telemetry.addData("r", "%.5f", r);
            telemetry.addData("Left Y", lY);
//            telemetry.addData("atLowLimit", atLowLimit);
//            telemetry.addData("atHighLimit", atHighLimit);
            telemetry.addData("Right Y", rY);
            telemetry.addData("liftMotor Pos", robot.liftMotor.getCurrentPosition());
            telemetry.addData("extendMotor Pos", robot.extendMotor.getCurrentPosition());
            telemetry.addData("grabberServo Pos", robot.grabberServo.getPosition());
            telemetry.update();
        }
    }
}
