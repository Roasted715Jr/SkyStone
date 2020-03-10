package org.firstinspires.ftc.teamcode.skystone;

import android.media.MediaPlayer;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.R;
import org.firstinspires.ftc.teamcode.util.GenericOpMode;
import org.firstinspires.ftc.teamcode.util.Robot;

@Autonomous(name = "Left Close", group = GenericOpMode.GROUP_SKYSTONE)
public class AutonLeftClose extends GenericOpMode {
    private Robot robot = new Robot(this);
    private AutonProcedures autonProcedures = new AutonProcedures();

    @Override
    public void runOpMode() throws InterruptedException {
        autonProcedures.init(robot, hardwareMap, this);
        MediaPlayer mediaPlayer = MediaPlayer.create(hardwareMap.appContext, R.raw.charge);

        waitForStart();

        mediaPlayer.start();

//        autonProcedures.simpleAuton(false, false, 22000);
        autonProcedures.simpleAuton(false, false, 25000);

        while (opModeIsActive()) {
            telemetry.update();
        }

        mediaPlayer.stop();

        robot.stop();
    }
}
