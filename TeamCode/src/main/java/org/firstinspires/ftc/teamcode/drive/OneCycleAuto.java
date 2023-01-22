package org.firstinspires.ftc.teamcode.drive;

import android.os.Build;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Recorder.JoystickRecorder;
import org.firstinspires.ftc.teamcode.detection.DetectionClass;
import org.firstinspires.ftc.teamcode.detection.DetectionTest;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;

import androidx.annotation.RequiresApi;

@Autonomous
public class OneCycleAuto extends LinearOpMode {

    @RequiresApi(api = Build.VERSION_CODES.O)
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

        JoystickRecorder slowBack = new JoystickRecorder("SlowBack.txt", telemetry);
        slowBack.loadPlayback();

        DetectionClass detector;
        DetectionTest.SkystoneDeterminationPipeline.SkystonePosition pos = null;
        long time;
        detector = new DetectionClass(hardwareMap, this);
        detector.init();

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

        slowBack.playSamplesWithTicks(drive, 101000);
        drive.setMotorPowers(-0.3, -0.2,-0.2, -0.3);
        sleep(450);
        drive.rotateRightWithGyro((float)0.25, -70);//62
        drive.rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        drive.rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        strafeWithDistance(100, drive);
        drive.setMotorPowers(0,0,0,0);
        backwardWithDistance(85, drive);
        drive.setMotorPowers(0,0,0,0);
        sleep(500);

        getoff(drive);
        scorecone(drive);
        sleep(200);
        pullverticalslides(drive);


        /*
        whileLoopScore(0, drive);
        whileLoopScore(1, drive);
        whileLoopScore(2, drive);
        whileLoopScore(3, drive);
        whileLoopScore(4, drive);
        scorecone(drive);
        pullverticalslides(drive);
        sleep(500);

         */

        drive.rotateRightWithGyro((float) 0.5, -81);//rotate back to perpendicular
        drive.rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        drive.rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        while(drive.rightRear.getCurrentPosition() < 10000){//strafe left
            drive.setMotorPowers(-0.4, 0.4, -0.4, 0.4);
            telemetry.addData("strafe: ", drive.rightRear.getCurrentPosition());
            telemetry.update();
        }
        if(pos == DetectionTest.SkystoneDeterminationPipeline.SkystonePosition.ONE){
            drive.setMotorPowers(0.6,0.6,0.6,0.6);
            sleep(1000);
            drive.setMotorPowers(0,0,0,0);
            drive.armUp();
            sleep(1000);
        }else if(pos == DetectionTest.SkystoneDeterminationPipeline.SkystonePosition.THREE){
            drive.setMotorPowers(-0.6,-0.6,-0.6,-0.6);
            sleep(760);
            drive.setMotorPowers(0,0,0,0);
            drive.armUp();
            sleep(1000);
        }
        else {
            drive.armUp();
            sleep(1000);
            stop();
        }
    }

    public void pullverticalslides(SampleMecanumDrive drive) { //pull vertical slides in
        while(drive.config.leftVerticalSlide.getCurrentPosition() >= 0){
            drive.config.leftVerticalSlide.setPower(-0.7); //pull in
            drive.config.rightVerticalSlide.setPower(-0.7);
        }
        drive.config.leftVerticalSlide.setPower(0);
        drive.config.rightVerticalSlide.setPower(0);
    }

    public void getoff(SampleMecanumDrive drive) //get the lift off of preloaded cone
    {
        drive.config.rightPivot.setPosition(0.5); //mid position
        drive.config.leftPivot.setPosition(0.5);
        sleep(500);
    }

    public void scorecone(SampleMecanumDrive drive) //lift vertical slides up
    {
        while(drive.config.leftVerticalSlide.getCurrentPosition() <= 2600){//-2350
            drive.config.leftVerticalSlide.setPower(0.65);
            drive.config.rightVerticalSlide.setPower(0.65);
        }
    }

    public void whileLoopScore(int iter, SampleMecanumDrive drive) { //extend slides to get cone
        double angleLeft = 0;
        double angleRight = 0;
        boolean horizontal = false;
        boolean horizontalFinal = false;
        boolean vertical = false;
        boolean height = true;

        int ticks = 2300;
        if (iter == 0) {
            angleLeft = 0.4;
            angleRight = 0.6;
            ticks = 2020;
        } else if (iter == 1) {
            angleLeft = 0.30;
            angleRight = 0.650;
            ticks = 1900;
        } else if (iter == 2) {
            angleLeft = 0.3;
            angleRight = 0.7;
            ticks = 1830;
        } else if (iter == 3) {
            angleLeft = .247;
            angleRight = 0.6988;
            ticks = 1800;
        } else {
            angleLeft = .238;
            angleRight = 0.709;
            ticks = 1770;
        }
        //pivot the claw

        drive.config.rightPivot.setPosition(angleRight); //set pivots on forward positions
        drive.config.leftPivot.setPosition(angleLeft);
        sleep(700);
        drive.config.claw.setPosition(0.6); //open claw



        //horizontal
        drive.config.intakeDrawerSlideLeft.setPower(0.4); //extend drawer slides out
        drive.config.intakeDrawerSlideRight.setPower(0.4);

        //vert
        drive.config.leftVerticalSlide.setPower(0.7);
        drive.config.rightVerticalSlide.setPower(0.7);
        while (true) {
            if ((horizontal && vertical)) {
                drive.config.leftVerticalSlide.setPower(0);
                drive.config.rightVerticalSlide.setPower(0);
                drive.config.intakeDrawerSlideLeft.setPower(0); //extend drawer slides out
                drive.config.intakeDrawerSlideRight.setPower(0);
                break;
            }
            if(gamepad2.left_bumper){
                break;
            }
            if (drive.config.intakeDrawerSlideLeft.getCurrentPosition() >= ticks || drive.config.intakeDrawerSlideRight.getCurrentPosition() >= ticks) {
                horizontalFinal = true;
                drive.config.intakeDrawerSlideLeft.setPower(0); //stop extending
                drive.config.intakeDrawerSlideRight.setPower(0);
                drive.config.claw.setPosition(0.89); //close the claw to grab cone
                sleep(700);
                drive.config.rightPivot.setPosition(0.5);
                drive.config.leftPivot.setPosition(0.5);
                sleep(300);
                drive.config.intakeDrawerSlideLeft.setPower(-0.4); //start retracting
                drive.config.intakeDrawerSlideRight.setPower(-0.4);
            }
            if (horizontalFinal && drive.config.intakeDrawerSlideLeft.getCurrentPosition() <= 0){
                drive.config.rightPivot.setPosition(0.276);
                drive.config.leftPivot.setPosition(0.666);
                sleep(400);
                drive.config.claw.setPosition(0.75);
                sleep(500);
                drive.config.rightPivot.setPosition(0.5);
                drive.config.leftPivot.setPosition(0.5);
                sleep(400);
                horizontal = true;
            }
            if (drive.config.leftVerticalSlide.getCurrentPosition() >= 2650 && height) {
                height = false;
                drive.config.leftVerticalSlide.setPower(-0.7);
                drive.config.rightVerticalSlide.setPower(-0.7);
            }
            if(!height && drive.config.leftVerticalSlide.getCurrentPosition() <= 0)
            {
                drive.config.leftVerticalSlide.setPower(0); //pull in
                drive.config.rightVerticalSlide.setPower(0);
                vertical = true;
            }
            telemetry.addData("boolean vars", vertical + ", " + horizontal + ", " + height);
            telemetry.addData("horizontal slides", "left slide" + drive.config.intakeDrawerSlideLeft.getCurrentPosition() + ", Right slide" + drive.config.intakeDrawerSlideRight.getCurrentPosition());
            telemetry.update();
        }
    }

    public void strafeWithDistance(int millimeters, SampleMecanumDrive drive){
        while(drive.config.distanceSensor.getDistance(DistanceUnit.MM) > millimeters){
            drive.setMotorPowers(0.2, -0.2, 0.2, -0.2);
        }
    }

    public void backwardWithDistance(int milimeters, SampleMecanumDrive drive){
        if(drive.config.distanceSensor.getDistance(DistanceUnit.MM) > milimeters){
            while(drive.config.distanceSensor.getDistance(DistanceUnit.MM) > milimeters){
                drive.setMotorPowers(-0.2, -0.2, -0.2, -0.2);
            }
        }else if(drive.config.distanceSensor.getDistance(DistanceUnit.MM) < milimeters){
            while(drive.config.distanceSensor.getDistance(DistanceUnit.MM) < milimeters){
                drive.setMotorPowers(0.2, 0.2, 0.2, 0.2);
            }
        }
    }
}

