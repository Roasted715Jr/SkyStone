package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import static java.lang.Thread.interrupted;

@Autonomous(name = "Autonomous", group = "Android Studio")
public class Auton extends GenericOpMode {
    Hardware<Auton> robot = new Hardware<>(this);
    AutonProcedures<Auton> autonProcedures = new AutonProcedures<>();

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        autonProcedures.init(robot, hardwareMap, this);

        Thread thread = new Thread(new Runnable() {
            @Override
            public void run() {
                while (!interrupted())
                    autonProcedures.start();
            }
        });

        waitForStart();

        thread.start();

        while (opModeIsActive()) {
            if (isStopRequested()) {
                autonProcedures.running = false;
//                thread.interrupt();
            }

            telemetry.update();
        }
    }
}
