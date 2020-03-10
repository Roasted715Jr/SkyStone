package org.firstinspires.ftc.teamcode.skystone;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.util.GenericOpMode;
import org.firstinspires.ftc.teamcode.util.Robot;

@TeleOp(name = "KidMode", group = GenericOpMode.GROUP_SKYSTONE)
public class KidMode extends GenericOpMode {
    private Robot robot = new Robot(this);

    private final double MOTOR_MULT = 0.5;

    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);

        double x, y, r;

        waitForStart();

        while (opModeIsActive()) {
            x = gamepad1.left_stick_x;
            y = -gamepad1.left_stick_y;
            r = gamepad1.right_stick_x;

            robot.setMecanumPower(x * MOTOR_MULT, y * MOTOR_MULT, r * MOTOR_MULT);

            telemetry.addData("x", "%.5f", x);
            telemetry.addData("y", "%.5f", y);
            telemetry.addData("r", "%.5f", r);
            telemetry.update();
        }
    }
}
