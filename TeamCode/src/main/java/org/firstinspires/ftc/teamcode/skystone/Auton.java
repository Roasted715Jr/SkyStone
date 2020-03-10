package org.firstinspires.ftc.teamcode.skystone;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.R;
import org.firstinspires.ftc.teamcode.util.GenericOpMode;
import org.firstinspires.ftc.teamcode.util.Robot;

import android.media.MediaPlayer;

@Autonomous(name = "Autonomous", group = "SkyStone")
@Disabled
public class Auton extends GenericOpMode {
    private Robot robot = new Robot(this);
    private AutonProcedures autonProcedures = new AutonProcedures();

    @Override
    public void runOpMode() throws InterruptedException {
        autonProcedures.init(robot, hardwareMap, this, true); //This calls init in the Robot class
        MediaPlayer mediaPlayer = MediaPlayer.create(hardwareMap.appContext, R.raw.charge);

        waitForStart();

        mediaPlayer.start();

//        autonProcedures.startWithStartSpot(2);
        autonProcedures.start();

        while (opModeIsActive()) {

        }

        mediaPlayer.stop();

        robot.stop();
    }
}
