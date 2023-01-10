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
public class PreLoadAuto extends LinearOpMode {

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

        DetectionClass detector;
        DetectionTest.SkystoneDeterminationPipeline.SkystonePosition pos = null;
        long time;
        detector = new DetectionClass(hardwareMap, this);
        detector.init();

        drive.setPoseEstimate(new Pose2d(35,58, Math.toRadians(90)));
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

        drive.followTrajectorySequence(drive.trajectorySequenceBuilder(new Pose2d(35, 58, Math.toRadians(90)))
                .back(54.5)
                .turn(Math.toRadians(-76))
                .build()
        );
        drive.setMotorPowers(0.6,-0.6,0.6,-0.6);
        sleep(320);
        drive.setMotorPowers(0,0,0,0);
        sleep(200);
        drive.setMotorPowers(0,0,0,0);
        getoff(drive);
        scorecone(drive);
        pullverticalslides(drive);

        drive.config.rightPivot.setPosition(0.27);//Bring arm up to prep for parking
        drive.config.leftPivot.setPosition(0.73);

        Pose2d newPose = drive.getPoseEstimate();
        drive.followTrajectorySequence(drive.trajectorySequenceBuilder(newPose)
                .turn(Math.toRadians(76))
                .build()
        );
        drive.setMotorPowers(0.6,-0.6,0.6,-0.6);
        sleep(250);
        drive.setMotorPowers(0.6,0.6,0.6,0.6);
        sleep(200);

        if(pos == DetectionTest.SkystoneDeterminationPipeline.SkystonePosition.THREE){
            drive.setMotorPowers(0.6,-0.6,0.6,-0.6); //strafe left
            sleep(600);
            drive.setMotorPowers(0,0,0,0);
        }else if(pos == DetectionTest.SkystoneDeterminationPipeline.SkystonePosition.ONE){
            drive.setMotorPowers(-0.6,0.6,-0.6,0.6); //strafe right
            sleep(600);
            drive.setMotorPowers(0,0,0,0);
        }
        else {
            stop();
        }

    }

    public void pullinslides(SampleMecanumDrive drive){
        drive.config.rightPivot.setPosition(0.5); //put pivots in neutral position
        drive.config.leftPivot.setPosition(0.5);
        while(drive.config.intakeDrawerSlideLeft.getCurrentPosition() >= 0 && drive.config.intakeDrawerSlideRight.getCurrentPosition() >= 0) {
            drive.config.intakeDrawerSlideLeft.setPower(-1); //pull in slides
            drive.config.intakeDrawerSlideRight.setPower(-1);
        }
        drive.config.intakeDrawerSlideLeft.setPower(0);
        drive.config.intakeDrawerSlideRight.setPower(0);
    }
    public void dropcone(SampleMecanumDrive drive) throws InterruptedException { //drop cone on the platform
        drive.config.rightPivot.setPosition(0.25);
        drive.config.leftPivot.setPosition(0.75);
        sleep(1200);
        drive.config.claw.setPosition(0.6); //open the claw
        sleep(500);
        drive.config.rightPivot.setPosition(0.84); //put pivots in forward position
        drive.config.leftPivot.setPosition(0.16);
        sleep(500);
    }

    public void pullverticalslides(SampleMecanumDrive drive) { //pull vertical slides in
        while(drive.config.leftVerticalSlide.getCurrentPosition() >= 2500){
            drive.config.leftVerticalSlide.setPower(-0.5); //pull in
            drive.config.rightVerticalSlide.setPower(-0.5);
        }
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
        while(drive.config.leftVerticalSlide.getCurrentPosition() <= 2300){//-2350
            drive.config.leftVerticalSlide.setPower(1);
            drive.config.rightVerticalSlide.setPower(1);
        }
    }
    public void getcone(int iter, SampleMecanumDrive drive){ //extend slides to get cone
        double angle;
        int ticks = 2300;
        if(iter==0) {
            angle =.37;
            ticks = 2300;
        } else if (iter==1){
            angle = .33;
            ticks = 2250;
        } else if(iter == 2){
            angle = .33;
            ticks = 2200;
        } else if (iter == 3){
            angle =.3;
            ticks = 2150;
        } else if (iter == 4){
            angle = .28;
            ticks = 2100;
        }
        else {
            angle = .5;
        }
        drive.config.rightPivot.setPosition(1-angle); //set pivots on forward positions
        drive.config.leftPivot.setPosition(angle);
        drive.config.claw.setPosition(0.4); //open claw
        while(drive.config.intakeDrawerSlideLeft.getCurrentPosition() <= ticks && drive.config.intakeDrawerSlideRight.getCurrentPosition() <= ticks)
        {
            drive.config.intakeDrawerSlideLeft.setPower(1); //extend drawer slides out
            drive.config.intakeDrawerSlideRight.setPower(1);
            telemetry.addData("ticks",  drive.config.intakeDrawerSlideLeft.getCurrentPosition() + ", " + drive.config.intakeDrawerSlideRight.getCurrentPosition());
            telemetry.update();
        }
        drive.config.intakeDrawerSlideLeft.setPower(0); //stop extending
        drive.config.intakeDrawerSlideRight.setPower(0);
        drive.config.claw.setPosition(0.85); //close the claw to grab cone
        sleep(500);
        drive.config.rightPivot.setPosition(0.5);
        drive.config.leftPivot.setPosition(0.5);
        sleep(1000);
    }


}

