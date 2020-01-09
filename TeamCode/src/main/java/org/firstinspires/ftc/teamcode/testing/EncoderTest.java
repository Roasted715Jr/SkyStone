package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.util.GenericOpMode;
import org.firstinspires.ftc.teamcode.util.Robot;

@TeleOp(name = "Encoder Test", group = GenericOpMode.GROUP_TESTINNG)
@Disabled
public class EncoderTest extends GenericOpMode {
    private Robot robot = new Robot(this);

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("fl", robot.flMotor.getCurrentPosition());
            telemetry.addData("bl", robot.blMotor.getCurrentPosition());
            telemetry.addData("fr", robot.frMotor.getCurrentPosition());
            telemetry.addData("br", robot.brMotor.getCurrentPosition());
            telemetry.update();
        }
    }
}
