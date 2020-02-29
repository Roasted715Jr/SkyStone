package org.firstinspires.ftc.teamcode.babyBot;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.util.GenericOpMode;
import org.firstinspires.ftc.teamcode.util.Robot;

@TeleOp(name = "Baby Bot Tank Drive", group = "Baby Bot")
public class TankDrive extends GenericOpMode {
    Robot robot = new Robot(this);

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);

        waitForStart();

        double l, r;

        while (opModeIsActive()) {
            r = -gamepad1.right_stick_y;
            l = -gamepad1.left_stick_y;

            robot.rightMotor.setPower(r);
            robot.leftMotor.setPower(l);

            telemetry.addData("r", r);
            telemetry.addData("l", l);
            telemetry.update();
        }
    }
}
