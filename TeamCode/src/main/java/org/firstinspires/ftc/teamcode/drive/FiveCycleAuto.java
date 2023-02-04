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
public class FiveCycleAuto extends LinearOpMode {

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

        JoystickRecorder slowBack = new JoystickRecorder("SlowBackNew.txt", telemetry);
        slowBack.loadPlayback();
        drive.config.clawSensor.enableLed(false);
        DetectionClass detector;
        DetectionTest.SkystoneDeterminationPipeline.SkystonePosition pos = null;
        long time;
        detector = new DetectionClass(hardwareMap, this);
        detector.init();

        //drive.config.alignmentTool.setPosition(0.5);
        drive.config.clawSensor.enableLed(true);
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

        slowBack.playSamplesWithTicks(drive, 74000);//75000
        drive.rotateRightWithGyro((float)0.3, -70);//62
        drive.setMotorPowers(-0.2, -0.2,-0.2, -0.2);
        sleep(450);
        drive.setMotorPowers(0,0,0,0);
        sleep(200);
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
        firstCycle(drive);
        sensorGrabCone(drive);
        drive.rotateRightWithGyroColor((float)0.2, (float)-90);
        //drive.backwardWithDistance(60);
        drive.config.intakeDrawerSlideLeft.setPower(0.4);
        drive.config.intakeDrawerSlideRight.setPower(0.4);
        sleep(100);
        drive.config.intakeDrawerSlideLeft.setPower(0);
        drive.config.intakeDrawerSlideRight.setPower(0);
        drive.closeClaw();
        sleep(500);
        drive.config.intakeDrawerSlideLeft.setPower(0.05);
        drive.config.intakeDrawerSlideRight.setPower(0.05);
        drive.config.leftPivot.setPosition(0.5);
        drive.config.rightPivot.setPosition(0.5);
        sleep(600);
        while(drive.config.intakeDrawerSlideLeft.getCurrentPosition() > 0){
            drive.config.intakeDrawerSlideLeft.setPower(-1);
            drive.config.intakeDrawerSlideRight.setPower(-1);
        }
        drive.config.intakeDrawerSlideLeft.setPower(0);
        drive.config.intakeDrawerSlideRight.setPower(0);
        drive.armUp();
        sleep(500);
        drive.slightOpen();
        sleep(300);
        drive.config.leftPivot.setPosition(0.5);
        drive.config.rightPivot.setPosition(0.5);

        if(whileLoopScore(1, drive) == 1){
            stop();
        }
        if(whileLoopScore(2, drive) == 1){
            stop();
        }
        if(whileLoopScore(3, drive) == 1){
            stop();
        }
        if(whileLoopScore(4, drive) == 1){
            stop();
        }
        getoff(drive);
        scorecone(drive);
        sleep(300);
        pullverticalslides(drive);
        sleep(500);

        drive.rotateRightWithGyro((float) 0.5, -81);//rotate back to perpendicular
        drive.rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        drive.rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        while(drive.rightRear.getCurrentPosition() < 12000){//strafe left
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
        drive.config.clawSensor.enableLed(false);
    }

    public void pullverticalslides(SampleMecanumDrive drive) { //pull vertical slides in
        sleep(100);
        while(drive.config.leftVerticalSlide.getCurrentPosition() >= 0){
            drive.config.leftVerticalSlide.setPower(-1); //pull in
            drive.config.rightVerticalSlide.setPower(-1);
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
        while(drive.config.leftVerticalSlide.getCurrentPosition() <= 2300){//-2350
            drive.config.leftVerticalSlide.setPower(1);
            drive.config.rightVerticalSlide.setPower(1);
        }
        drive.config.leftVerticalSlide.setPower(0);
        drive.config.rightVerticalSlide.setPower(0);
    }


    public void sensorGrabCone(SampleMecanumDrive drive){
        drive.rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        drive.rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        while(drive.config.clawSensor.red() <= 60 && drive.config.clawSensor.blue() <= 60 && drive.rightRear.getCurrentPosition() >= -1800){
            drive.setMotorPowers(0.3, -0.3, 0.3, -0.3);
        }
        drive.stop();
    }

    public void firstCycle(SampleMecanumDrive drive){
        boolean scored = false;
        boolean extension = false;
        boolean vert = false;
        drive.config.leftPivot.setPosition(0.7);
        drive.config.rightPivot.setPosition(0.3);
        sleep(700);
        drive.config.intakeDrawerSlideLeft.setPower(0.8);
        drive.config.intakeDrawerSlideRight.setPower(0.8);
        drive.config.claw.setPosition(0);
        sleep(300);
        drive.config.leftVerticalSlide.setPower(1);
        drive.config.rightVerticalSlide.setPower(1);
        while(true){
            if(scored && extension){
                break;
            }
            if(drive.config.leftVerticalSlide.getCurrentPosition() >= 2300){
                drive.config.leftVerticalSlide.setPower(0);
                drive.config.rightVerticalSlide.setPower(0);
                sleep(100);
                drive.config.leftVerticalSlide.setPower(-1);
                drive.config.rightVerticalSlide.setPower(-1);
                vert = true;
            }
            if(drive.config.clawSensor.getDistance(DistanceUnit.CM) <= 1.9 || drive.config.intakeDrawerSlideLeft.getCurrentPosition() >= 1075 || drive.config.intakeDrawerSlideRight.getCurrentPosition() >= 1075){
                drive.config.intakeDrawerSlideLeft.setPower(0);
                drive.config.intakeDrawerSlideRight.setPower(0);
                extension = true;
            }
            if(vert && drive.config.leftVerticalSlide.getCurrentPosition() <= 0){
                drive.config.leftVerticalSlide.setPower(0);
                drive.config.rightVerticalSlide.setPower(0);
                scored = true;
            }
        }
    }

    public int whileLoopScore(int iter, SampleMecanumDrive drive) { //extend slides to get cone
        double angleLeft = 0;
        double angleRight = 0;
        boolean horizontal = false;
        boolean horizontalFinal = false;
        boolean vertical = false;
        boolean height = true;
        boolean peak = false;
        boolean grab = true;
        boolean upPos = false;
        boolean fastHorizontal = false;
        boolean slowed = true;
        long currentTime = 0;
        long currentTimeVert = 0;
        long currentTimeDeposit;
        int i = 0;

        int ticks = 960;
        if (iter == 0) {
            angleLeft = 0.7;
            angleRight = 0.3;
            ticks = 980;
        } else if (iter == 1) {
            angleLeft = 0.73;
            angleRight = 0.27;
            ticks = 1050; //1050
        } else if (iter == 2) {
            angleLeft = 0.73;
            angleRight = 0.27;
            ticks = 1055; //1055
        } else if (iter == 3) {
            angleLeft = .75;
            angleRight = 0.25;
            ticks = 1050; //1050
        } else {
            angleLeft = .77;
            angleRight = 0.23;
            ticks = 1020; //1020
        }
        //pivot the claw

        drive.config.rightPivot.setPosition(angleRight); //set pivots on forward positions
        drive.config.leftPivot.setPosition(angleLeft);
        sleep(700);
        drive.openClaw();

        //horizontal
        drive.config.intakeDrawerSlideLeft.setPower(0.7); //extend drawer slides out
        drive.config.intakeDrawerSlideRight.setPower(0.7);

        //vert
        drive.config.leftVerticalSlide.setPower(1);
        drive.config.rightVerticalSlide.setPower(1);
        while (true) {
            if ((horizontal && vertical)) {//Stop method once all slides are back in their starting position
                drive.config.leftVerticalSlide.setPower(0);
                drive.config.rightVerticalSlide.setPower(0);
                drive.config.intakeDrawerSlideLeft.setPower(0);
                drive.config.intakeDrawerSlideRight.setPower(0);
                break;
            }
            if(Float.parseFloat(drive.getPitchAngle()) > 5 || Float.parseFloat(drive.getRollAngle()) > 5){
                return 1;
            }
            if(gamepad2.left_bumper){
                return 1;
            }
            if (drive.config.leftVerticalSlide.getCurrentPosition() >= 2270 && height && !peak) {//Pull down vertical sides
                drive.config.leftVerticalSlide.setPower(0);
                drive.config.rightVerticalSlide.setPower(0);
                currentTimeVert = System.currentTimeMillis();
                peak = true;
            }
            if(System.currentTimeMillis() - currentTimeVert >= 300 && peak){//Replaces Sleep methods
                drive.config.leftVerticalSlide.setPower(-1);
                drive.config.rightVerticalSlide.setPower(-1);
                height = false;
                peak = false;
            }
            if(!height && drive.config.leftVerticalSlide.getCurrentPosition() <= 0) {//Stop vertical slides once they are done extending
                drive.config.leftVerticalSlide.setPower(0);
                drive.config.rightVerticalSlide.setPower(0);
                vertical = true;
            }
            if(grab && (drive.config.intakeDrawerSlideLeft.getCurrentPosition() >= ticks - 50 || drive.config.intakeDrawerSlideRight.getCurrentPosition() >= ticks - 50)){
                drive.config.intakeDrawerSlideLeft.setPower(0); //stop extending
                drive.config.intakeDrawerSlideRight.setPower(0);
                grab = false;
                upPos = true;
            }
            if(!horizontalFinal && (upPos && drive.config.leftVerticalSlide.getCurrentPosition() <= 1000) && (drive.config.intakeDrawerSlideLeft.getCurrentPosition() >= ticks - 50 || drive.config.intakeDrawerSlideRight.getCurrentPosition() >= ticks - 50)) {//Extend horizontal slides until it reaches the cone
                drive.config.intakeDrawerSlideRight.setPower(0.4);
                drive.config.intakeDrawerSlideRight.setPower(0.4);
                sleep(300);
                drive.config.intakeDrawerSlideLeft.setPower(0); //stop extending
                drive.config.intakeDrawerSlideRight.setPower(0);
                drive.closeClaw(); //close the claw to grab cone
                sleep(300);
                drive.config.intakeDrawerSlideLeft.setPower(0.05);
                drive.config.intakeDrawerSlideRight.setPower(0.05);
                drive.config.rightPivot.setPosition(0.5);
                drive.config.leftPivot.setPosition(0.5);
                sleep(250);
                drive.config.intakeDrawerSlideLeft.setPower(-1); //start retracting
                drive.config.intakeDrawerSlideRight.setPower(-1);
                horizontalFinal = true;
            }
            if((slowed && horizontalFinal) && drive.config.intakeDrawerSlideLeft.getCurrentPosition() <= 100){
                drive.config.intakeDrawerSlideLeft.setPower(-0.4); //start retracting
                drive.config.intakeDrawerSlideRight.setPower(-0.4);
                fastHorizontal = true;
                slowed = false;
            }
            if(fastHorizontal && drive.config.intakeDrawerSlideLeft.getCurrentPosition() <= 0){
                if(vertical){
                    drive.armUp();
                    sleep(500);
                    drive.slightOpen();
                    sleep(250);
                    horizontal = true;
                }
            }

            telemetry.addData("grab", grab );
            telemetry.addData("statement 1: ", drive.config.intakeDrawerSlideLeft.getCurrentPosition() >= ticks || drive.config.intakeDrawerSlideRight.getCurrentPosition() >= ticks);
            telemetry.addData("horizontal slides", "left slide" + drive.config.intakeDrawerSlideLeft.getCurrentPosition() + ", Right slide" + drive.config.intakeDrawerSlideRight.getCurrentPosition());
            telemetry.update();
        }
        return 0;
    }

}

