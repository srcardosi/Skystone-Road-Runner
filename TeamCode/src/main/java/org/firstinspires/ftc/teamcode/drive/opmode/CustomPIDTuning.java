package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveBase;
import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveREV;

/**
 * Created by Sean Cardosi on 2020-01-11.
 */
@TeleOp(name = "Custom PID Tuner")
public class CustomPIDTuning extends LinearOpMode {

    SampleMecanumDriveBase drive;
    @Override
    public void runOpMode() {
        drive = new SampleMecanumDriveREV(hardwareMap);
        drive.setPoseEstimate(new Pose2d(0, 0, 0));


        waitForStart();


        telemetry.addData("Forward", " 24 inches");
        telemetry.update();
        drive.followTrajectorySync(drive.trajectoryBuilder()
                .forward(24)
                .build()
        );

        sleep(2000);

        telemetry.addData("Backwards", " 24 inches");
        telemetry.update();
        drive.followTrajectorySync(drive.trajectoryBuilder()
                .back(24)
                .build()
        );

        sleep(2000);

        telemetry.addData("Left", " 24 inches");
        telemetry.update();
        drive.followTrajectorySync(drive.trajectoryBuilder()
                .strafeLeft(24)
                .build()
        );

        sleep(2000);

        telemetry.addData("Right", " 24 inches");
        telemetry.update();
        drive.followTrajectorySync(drive.trajectoryBuilder()
                .strafeRight(24)
                .build()
        );

        sleep(2000);

        telemetry.addData("Turning", "90 Degrees");
        telemetry.update();

        drive.turnSync(90);
    }
}
