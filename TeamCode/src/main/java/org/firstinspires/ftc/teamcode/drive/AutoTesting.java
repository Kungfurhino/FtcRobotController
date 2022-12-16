package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.detection.DetectionClass;
import org.firstinspires.ftc.teamcode.detection.DetectionTest;

@Autonomous
public class AutoTesting extends LinearOpMode {

    DetectionTest.SkystoneDeterminationPipeline.SkystonePosition position;
    private static SampleMecanumDrive robot;
    DetectionClass detector;
    private long time;

    //Left front = -47500
    // right rear for going left 43139, going right -43139

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new SampleMecanumDrive(hardwareMap);
        robot.leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        detector = new DetectionClass(hardwareMap, this);
        detector.init();

        waitForStart();
        position = detector.Detect();
        time = System.currentTimeMillis();
        while(position == null){
            position = detector.Detect();
            if(System.currentTimeMillis() - time > 1500){
                position = DetectionTest.SkystoneDeterminationPipeline.SkystonePosition.ONE;
                telemetry.addData("Defaulted", "to ONE");
                break;
            }
        }
        telemetry.addData("Detection", position);
        telemetry.update();
        sleep(2000);
        while(robot.leftFront.getCurrentPosition() > -40000){
            robot.setMotorPowers(0.5, 0.5, 0.5, 0.5);
            telemetry.addData("LeftFront", robot.leftFront.getCurrentPosition());
            telemetry.update();
        }
        if(position == DetectionTest.SkystoneDeterminationPipeline.SkystonePosition.THREE){
            while(robot.rightRear.getCurrentPosition() > -41000){ //strafing right
                robot.setMotorPowers(0.5, -0.5, 0.5, -0.5);
                telemetry.addData("Right Rear", robot.rightRear.getCurrentPosition());
            }
        }else if(position == DetectionTest.SkystoneDeterminationPipeline.SkystonePosition.ONE){
            while(robot.rightRear.getCurrentPosition() < 44000){ //strafing left
                robot.setMotorPowers(-0.5, 0.5, -0.5, 0.5);
                telemetry.addData("Right Rear", robot.rightRear.getCurrentPosition());
            }
        }
    }
}
