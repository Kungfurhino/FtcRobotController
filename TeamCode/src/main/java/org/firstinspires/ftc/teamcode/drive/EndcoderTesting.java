package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class EndcoderTesting extends LinearOpMode {

    enum State{
        IDLE,
        SLOW_SERVO_DOWN,
        SlOW_SERVO_UP,
        PULL_IN_SLIDES,
        DROP_CONE
    }

    private int verticalLiftTicks;
    private int drawerSlidesTicks;
    private int initialDrawerTicks = 0;

    State currentState = State.IDLE;

    private long currentTime;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive robot = new SampleMecanumDrive(hardwareMap);
        robot.leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.config.intakeDrawerSlideRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.config.intakeDrawerSlideRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.config.leftVerticalSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.config.leftVerticalSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();
        while(opModeIsActive() && !isStopRequested()) {
            switch (currentState) {
                case SlOW_SERVO_UP:
                    double progress = System.currentTimeMillis() - currentTime;
                    double angleRight = map(progress, 0, 7000, 0.9, 0.2);
                    double angleLeft = map(progress, 0, 7000, 0.1, 0.8);
                    robot.config.rightPivot.setPosition(angleRight);
                    robot.config.leftPivot.setPosition(angleLeft);
                    if (progress >= 7000) {
                        currentState = State.IDLE;
                    }
                    break;
                case SLOW_SERVO_DOWN:
                    double timeProgress = System.currentTimeMillis() - currentTime;
                    double right = map(timeProgress, 0, 7000, 0.25, 0.9);
                    double left = map(timeProgress, 0, 7000, 0.75, 0.1);
                    robot.config.rightPivot.setPosition(right);
                    robot.config.leftPivot.setPosition(left);
                    if (timeProgress >= 7000) {
                        currentState = State.IDLE;
                    }
                    break;
                case PULL_IN_SLIDES:
                    robot.config.rightPivot.setPosition(0.5);
                    robot.config.leftPivot.setPosition(0.5);
                    robot.config.leftVerticalSlide.setPower(1);
                    robot.config.rightVerticalSlide.setPower(1);
                    if(robot.config.leftVerticalSlide.getCurrentPosition() <= initialDrawerTicks){
                        initialDrawerTicks = robot.config.leftVerticalSlide.getCurrentPosition();
                        currentState = State.IDLE;
                    }
                    break;
                case DROP_CONE:
                    robot.config.rightPivot.setPosition(0.2);
                    robot.config.leftPivot.setPosition(0.8);
                    robot.config.claw.setPosition(0.6);
                    robot.config.rightPivot.setPosition(0.9);
                    robot.config.leftPivot.setPosition(0.1);
                    currentState = State.IDLE;
                    break;
                case IDLE:
                    currentTime = System.currentTimeMillis();
                    drawerSlidesTicks = robot.config.intakeDrawerSlideLeft.getCurrentPosition();
                    verticalLiftTicks = robot.config.leftVerticalSlide.getCurrentPosition();
                    break;
            }
            if (-gamepad2.left_stick_y != 0) {
                robot.config.rightVerticalSlide.setPower(-gamepad2.left_stick_y);
                robot.config.leftVerticalSlide.setPower(-gamepad2.left_stick_y);
            } else if (gamepad1.left_bumper) {
                robot.config.intakeDrawerSlideLeft.setPower(1);
                robot.config.intakeDrawerSlideRight.setPower(1);
            } else if (gamepad1.right_bumper){
                robot.config.intakeDrawerSlideLeft.setPower(-1);
                robot.config.intakeDrawerSlideRight.setPower(-1);
            }else if (gamepad2.dpad_left) {
                robot.config.rightPivot.setPosition(0.2);
                robot.config.leftPivot.setPosition(0.8);
            } else if (gamepad2.dpad_right) {
                robot.config.rightPivot.setPosition(0.9);
                robot.config.leftPivot.setPosition(0.1);
            } else if (gamepad2.b) {
                robot.config.claw.setPosition(0.6);
            } else if (gamepad2.x) {
                robot.config.claw.setPosition(0.85);
            } else if (gamepad2.right_bumper) {
                robot.config.claw.setPosition(0.7);
            } else if (gamepad2.dpad_down) {
                currentState = State.PULL_IN_SLIDES;
            } else if(gamepad2.dpad_up){
                currentState = State.DROP_CONE;
            } else{
                robot.zeroAllMotors();
                robot.setWeightedDrivePower(
                        new Pose2d(
                                -gamepad1.left_stick_y,
                                -gamepad1.left_stick_x,
                                -gamepad1.right_stick_x
                        )
                );
                telemetry.addData("rightRear", robot.rightRear.getCurrentPosition() + "\n");
                telemetry.addData("leftFront", robot.leftFront.getCurrentPosition() + "\n");
                telemetry.addData("leftRear", robot.leftRear.getCurrentPosition() + "\n");
                telemetry.addData("Right Vertical Slide", robot.config.rightVerticalSlide.getCurrentPosition() + "\n");
                telemetry.addData("Left Vertical Slide", robot.config.leftVerticalSlide.getCurrentPosition() + "\n");
                telemetry.addData("Intake Drawer slides", robot.config.intakeDrawerSlideRight.getCurrentPosition() + "\n");
                telemetry.update();
                robot.update();
            }
        }
    }

    public double map(double x, double in_min, double in_max, double out_min, double out_max){
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    }
}
