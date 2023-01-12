package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.detection.DetectionClass;
import org.firstinspires.ftc.teamcode.detection.DetectionTest;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;

@Autonomous
public class OneCycleAuto extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        drive.config.leftVerticalSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        drive.config.leftVerticalSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        drive.config.rightVerticalSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        drive.config.rightVerticalSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        drive.config.intakeDrawerSlideRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        drive.config.intakeDrawerSlideRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        drive.config.intakeDrawerSlideLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        drive.config.intakeDrawerSlideLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        drive.leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        drive.leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        drive.rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        drive.rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        drive.rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        drive.rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        DetectionClass detector;
        DetectionTest.SkystoneDeterminationPipeline.SkystonePosition pos = null;
        long time;
        detector = new DetectionClass(hardwareMap, this);
        detector.init();

        drive.setPoseEstimate(new Pose2d(35,58, Math.toRadians(90)));
        drive.config.rightPivot.setPosition(0.22);
        drive.config.leftPivot.setPosition(0.78);
        waitForStart();

        if (isStopRequested()) return;

        pos = detector.Detect();
        time = System.currentTimeMillis();
        while(pos == null){
            pos = detector.Detect();
            if(System.currentTimeMillis() - time > 1500){
                pos = DetectionTest.SkystoneDeterminationPipeline.SkystonePosition.ONE;
                telemetry.addData("Defaulted", "to ONE");
                break;
            }
        }
        telemetry.addData("position", pos.toString());

        drive.goBackwardOdometers(-80000, (float)0.6, telemetry);
        drive.setMotorPowers(0.5,0.5,0.5,0.5);
        sleep(200);
        drive.setMotorPowers(0,0,0,0);
        sleep(200);
        drive.rotateRightWithGyro((float)0.5, -76);

        drive.setMotorPowers(0.6,-0.6,0.6,-0.6);
        sleep(320);
        //drive.setMotorPowers(0.4,0.4,0.4,0.4);
        //sleep(200);
        drive.setMotorPowers(0,0,0,0);
        getoff(drive);
        scorecone(drive);
        pullverticalslides(drive);

        drive.config.rightPivot.setPosition(0.27);//Bring arm up to prep for parking
        drive.config.leftPivot.setPosition(0.73);

        Pose2d newPose = drive.getPoseEstimate();
        drive.followTrajectorySequence(drive.trajectorySequenceBuilder(newPose)
                .turn(Math.toRadians(-16.6))
                .build()
        );
        drive.setMotorPowers(-0.6,0.6,-0.6,0.6);
        sleep(450);

        if(pos == DetectionTest.SkystoneDeterminationPipeline.SkystonePosition.ONE){
            drive.setMotorPowers(0.6,0.6,0.6,0.6); //strafe left
            sleep(900);
            drive.setMotorPowers(0,0,0,0);
        }else if(pos == DetectionTest.SkystoneDeterminationPipeline.SkystonePosition.THREE){
            drive.setMotorPowers(-0.6,-0.6,-0.6,-0.6); //strafe right
            sleep(760);
            drive.setMotorPowers(0,0,0,0);
        }
        else {
            stop();
        }

    }

    public void pullverticalslides(SampleMecanumDrive drive) { //pull vertical slides in
        while(drive.config.leftVerticalSlide.getCurrentPosition() >= 0){
            drive.config.leftVerticalSlide.setPower(-1); //pull in
            drive.config.rightVerticalSlide.setPower(-1);
        }
        drive.config.leftVerticalSlide.setPower(0);
        drive.config.rightVerticalSlide.setPower(0);
    }

    public void getoff(SampleMecanumDrive drive) //get the lift off of preloaded cone
    {
        drive.config.claw.setPosition(0.6); //open claw
        sleep(150);
        drive.config.rightPivot.setPosition(0.5); //mid position
        drive.config.leftPivot.setPosition(0.5);
        sleep(500);
    }

    public void scorecone(SampleMecanumDrive drive) //lift vertical slides up
    {
        while(drive.config.leftVerticalSlide.getCurrentPosition() <= 2600){//-2350
            drive.config.leftVerticalSlide.setPower(1);
            drive.config.rightVerticalSlide.setPower(1);
        }
    }

}

