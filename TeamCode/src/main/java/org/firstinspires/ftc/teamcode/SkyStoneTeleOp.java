package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "SkyStone TeleOp", group = "SkyStone")
public class SkyStoneTeleOp extends GenericOpMode {
    private Hardware<SkyStoneTeleOp> robot = new Hardware<>(this);

    public void runOpMode() throws InterruptedException {
        boolean aPressed = false, multiplierToggle = false;
        double speedMultiplier = 1;

        robot.init(hardwareMap);

        double x, y, r;

        waitForStart();

        while (opModeIsActive()) {
            x = gamepad1.left_stick_x;
            y = -gamepad1.left_stick_y;
            r = gamepad1.right_stick_x;

            if (gamepad1.a) {
                if (!aPressed)
                    multiplierToggle = !multiplierToggle;
                aPressed = true;
            } else
                aPressed = false;

            speedMultiplier = multiplierToggle ? 0.5 : 1;

            robot.setMecanumMotorPowers(x * speedMultiplier, y * speedMultiplier, r * speedMultiplier);

            if (gamepad2.right_trigger > 0.5)
                robot.clawServo.setPosition(0.95);
            else if (gamepad2.right_bumper)
                robot.clawServo.setPosition(0.25);

            if (gamepad2.left_trigger > 0.5)
                robot.armServo.setPosition(robot.armServo.getPosition() + 0.003);
            else if (gamepad2.left_bumper)
                robot.armServo.setPosition(robot.armServo.getPosition() - 0.003);

            if (gamepad2.a)
                robot.moveHooks(true);
            else if (gamepad2.b)
                robot.moveHooks(false);

            telemetry.addData("speedMultiplier", speedMultiplier);
            telemetry.addData("x", "%.5f", x);
            telemetry.addData("y", "%.5f", y);
            telemetry.addData("r", "%.5f", r);
            telemetry.update();
        }
    }
}
