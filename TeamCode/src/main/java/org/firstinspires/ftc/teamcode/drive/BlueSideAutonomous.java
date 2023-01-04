package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;

@Autonomous
public class BlueSideAutonomous extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        drive.setPoseEstimate(new Pose2d(35,58, Math.toRadians(90)));
        waitForStart();

        if (isStopRequested()) return;

        drive.followTrajectorySequence(drive.trajectorySequenceBuilder(new Pose2d(35, 58, Math.toRadians(90)))
                        .back(59)
                        .turn(Math.toRadians(-75))
                        .build()
                );
        drive.setMotorPowers(-1,-1,-1,-1);
        sleep(100);


        while (!isStopRequested() && opModeIsActive()) ;
    }
}

