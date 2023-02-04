package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Autonomous
public class AutonomousTesting extends LinearOpMode {

    SampleMecanumDrive robot;
    int ticks = 2300;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new SampleMecanumDrive(hardwareMap);

        robot.config.intakeDrawerSlideRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.config.intakeDrawerSlideRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.config.intakeDrawerSlideLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.config.intakeDrawerSlideLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

        getcone(4, robot);

    }

    public void getcone(int iter, SampleMecanumDrive drive){ //extend slides to get cone
        double angle;
        if(iter==0) {
            angle =.39;
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
            drive.config.intakeDrawerSlideLeft.setPower(0.4); //extend drawer slides out
            drive.config.intakeDrawerSlideRight.setPower(0.4);
            telemetry.addData("ticks",  drive.config.intakeDrawerSlideLeft.getCurrentPosition() + ", " + drive.config.intakeDrawerSlideRight.getCurrentPosition());
            telemetry.update();
        }
        drive.config.intakeDrawerSlideLeft.setPower(0); //stop extending
        drive.config.intakeDrawerSlideRight.setPower(0);
        drive.config.claw.setPosition(0.85); //close the claw to grab cone
        sleep(1000);
        drive.config.rightPivot.setPosition(0.5);
        drive.config.leftPivot.setPosition(0.5);
        sleep(1000);
        while(drive.config.intakeDrawerSlideLeft.getCurrentPosition() >= 0 && drive.config.intakeDrawerSlideRight.getCurrentPosition() >= 0)
        {
            drive.config.intakeDrawerSlideLeft.setPower(-0.4); //extend drawer slides out
            drive.config.intakeDrawerSlideRight.setPower(-0.4);
            telemetry.addData("ticks",  drive.config.intakeDrawerSlideLeft.getCurrentPosition() + ", " + drive.config.intakeDrawerSlideRight.getCurrentPosition());
            telemetry.update();
        }
    }
}
