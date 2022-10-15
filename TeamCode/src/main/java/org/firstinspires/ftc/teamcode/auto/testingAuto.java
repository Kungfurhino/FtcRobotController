package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import ii.SampleMecanumDrive;

@Autonomous
public class testingAuto extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        //drive.setPoseEstimate(new Pose2d(-37.2,-71.2, Math.toRadians(93)));
        Trajectory ducksRoute = drive.trajectoryBuilder(new Pose2d(0,0, Math.toRadians(0)))
                .splineTo(
                        new Vector2d(30, 30), 0,
                        SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .build();


        waitForStart();

        if(isStopRequested()) return;
        //Need to be separate trajectories or else non continous since matching the velocities with the paths is impossible
        drive.followTrajectory(ducksRoute);
    }
}
