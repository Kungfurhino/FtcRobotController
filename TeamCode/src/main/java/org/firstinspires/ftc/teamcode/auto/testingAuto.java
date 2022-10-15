package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;

import ii.SampleMecanumDrive;

@Autonomous
public class testingAuto extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Trajectory straight = drive.trajectoryBuilder(new Pose2d())
                .forward(30)
                .build();
        Trajectory strafe = drive.trajectoryBuilder(new Pose2d(30, 0, 0))
                .back(30)
                .build();
        Trajectory spline = drive.trajectoryBuilder(strafe.end())
                .splineTo(new Vector2d(40, 40), Math.toRadians(0))
                .build();


        waitForStart();

        if(isStopRequested()) return;

        //Need to be separate trajectories or else non continous since matching the velocities with the paths is impossible
        drive.followTrajectory(straight);
        telemetry.addData("current pose: ",  "\nx: " + drive.getPoseEstimate().getX() + "\ny: " + drive.getPoseEstimate().getY() + "\nheading: " + drive.getPoseEstimate().getHeading());
        telemetry.update();
        //telemetry.addData("current pose: ",  "\nx: " + drive.getPoseEstimate().getX() + "\ny: " + drive.getPoseEstimate().getY() + "\nheading: " + drive.getPoseEstimate().getHeading());
        drive.followTrajectory(strafe);
        drive.followTrajectory(spline);
        telemetry.addData("current pose: ",  "\nx: " + drive.getPoseEstimate().getX() + "\ny: " + drive.getPoseEstimate().getY() + "\nheading: " + drive.getPoseEstimate().getHeading());
    }
}
