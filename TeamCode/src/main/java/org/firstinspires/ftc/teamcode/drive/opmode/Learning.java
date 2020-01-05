//package org.firstinspires.ftc.teamcode.drive.opmode;
//
//import com.acmerobotics.roadrunner.geometry.Pose2d;
//import com.acmerobotics.roadrunner.geometry.Vector2d;
//import com.acmerobotics.roadrunner.path.Path;
//import com.acmerobotics.roadrunner.path.PathBuilder;
//import com.acmerobotics.roadrunner.path.heading.ConstantInterpolator;
//import com.acmerobotics.roadrunner.path.heading.LinearInterpolator;
//import com.acmerobotics.roadrunner.trajectory.Trajectory;
//import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
//import com.acmerobotics.roadrunner.trajectory.TrajectoryGenerator;
//import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints;
//import com.acmerobotics.roadrunner.trajectory.constraints.MecanumConstraints;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//
//import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveBase;
//import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveREV;
//import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveREVOptimized;
//
///**
// * This is the field layout in the Road Runner coordinate system.
// *
// *                  (+x)
// *                    |
// *                    |
// *                    | f (foundation)
// *                    |
// *                    |
// * (+y)-------------(0,0)-------------(-y)
// *                    |
// *                    | s (stones)
// *                    | s
// *                    | s
// *                    |
// *                  (-x)
// *
// * Created by Sean Cardosi on 2020-01-02.
// */
//public class Learning extends LinearOpMode {
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//        SampleMecanumDriveREV drive = new SampleMecanumDriveREV(hardwareMap);
//        drive.setPoseEstimate(new Pose2d(0,0,0));//Define start position
//
//        waitForStart();
//
//        Trajectory trajectory = drive
//                .trajectoryBuilder()
//                .lineTo(new Vector2d(-40,40), new LinearInterpolator(Math.toRadians(90), Math.toRadians(180)))
//                .build();
//
//        if (isStopRequested()) return;
//
//        drive.followTrajectorySync(trajectory);
//
//    }
//}
