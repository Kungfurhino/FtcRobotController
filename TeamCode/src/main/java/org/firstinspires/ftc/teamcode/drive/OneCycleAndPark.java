package org.firstinspires.ftc.teamcode.drive;

import android.os.Build;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Recorder.JoystickRecorder;
import org.firstinspires.ftc.teamcode.detection.DetectionClass;
import org.firstinspires.ftc.teamcode.detection.DetectionTest;

import androidx.annotation.RequiresApi;

@Autonomous
public class OneCycleAndPark extends LinearOpMode {

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

        drive.config.alignmentTool.setPosition(0.5);
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

        slowBack.playSamplesWithTicks(drive, 100000);
        drive.rotateRightWithGyro((float)0.3, -70);
        drive.setMotorPowers(-0.2, -0.2,-0.2, -0.2);
        sleep(450);
        drive.strafeWithDistance(200);
        drive.setMotorPowers(0,0,0,0);
        drive.backwardWithDistance(82);
        drive.setMotorPowers(0,0,0,0);
        drive.rotateWithBackSensor(500);
        drive.strafeWithDistance(100);
        drive.setMotorPowers(0.3, -0.3, 0.3, -0.3 );
        sleep(100);
        drive.stop();
        drive.backwardWithDistance(60);
        drive.setMotorPowers(0.2,0.2,-0.2,-0.2);
        sleep(60);
        drive.setMotorPowers(0,0,0,0);
        drive.config.leftPivot.setPosition(0.5);
        drive.config.rightPivot.setPosition(0.5);
        scorecone(drive);
        pullverticalslides(drive);

        drive.rotateRightWithGyro((float) 0.5, -81);//rotate back to perpendicular
        drive.rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        drive.rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        while(drive.rightRear.getCurrentPosition() < 10000){//TODO - change the amount of ticks
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
        drive.armUp();
        sleep(1000);
        stop();
        drive.config.clawSensor.enableLed(false);
    }

    public void pullverticalslides(SampleMecanumDrive drive) { //pull vertical slides in
        sleep(300);
        while(drive.config.leftVerticalSlide.getCurrentPosition() >= 0){
            drive.config.leftVerticalSlide.setPower(-1); //pull in
            drive.config.rightVerticalSlide.setPower(-1);
        }
        drive.config.leftVerticalSlide.setPower(0);
        drive.config.rightVerticalSlide.setPower(0);
    }

    public void scorecone(SampleMecanumDrive drive) //lift vertical slides up
    {
        while(drive.config.leftVerticalSlide.getCurrentPosition() <= 2300){//-2350
            drive.config.leftVerticalSlide.setPower(1);
            drive.config.rightVerticalSlide.setPower(1);
        }
        drive.config.leftVerticalSlide.setPower(0);
        drive.config.rightVerticalSlide.setPower(0);
    }


}

