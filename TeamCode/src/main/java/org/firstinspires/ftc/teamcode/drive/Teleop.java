package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class Teleop extends LinearOpMode {

    //2600 ticks for high goal
    enum State{
        IDLE,
        PULL_IN_SLIDES,
        PULL_IN_SLOW,
        RAISE_VERT,
        PULL_IN_VERT,
        DROP_CONE
    }

    private int verticalLiftTicks;
    private int initialDrawerTicks = 0;
    private int alignNum = 0;
    private int initialVerticalTicks = 0;
    private double leftServo;
    private double rightServo;
    private boolean auto = false;

    State currentState = State.IDLE;

    private long currentTime;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive robot = new SampleMecanumDrive(hardwareMap);
        robot.leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.config.intakeDrawerSlideRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.config.intakeDrawerSlideRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.config.intakeDrawerSlideLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.config.intakeDrawerSlideLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.config.leftVerticalSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.config.leftVerticalSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();
        while(opModeIsActive() && !isStopRequested()) {
            switch (currentState) {
                case PULL_IN_SLIDES:
                    robot.config.rightPivot.setPosition(0.5);
                    robot.config.leftPivot.setPosition(0.5);
                    robot.config.intakeDrawerSlideRight.setPower(-0.7);
                    robot.config.intakeDrawerSlideLeft.setPower(-0.7);
                    if(robot.config.intakeDrawerSlideLeft.getCurrentPosition() <= 100){
                        robot.config.intakeDrawerSlideLeft.setPower(-0.1);
                        robot.config.intakeDrawerSlideRight.setPower(-0.1);
                        currentState = State.PULL_IN_SLOW;
                    }
                    break;
                case PULL_IN_SLOW:
                    if(robot.config.intakeDrawerSlideLeft.getCurrentPosition() <= 0){
                        robot.config.rightPivot.setPosition(0.22);
                        robot.config.leftPivot.setPosition(0.78);
                        currentTime = System.currentTimeMillis();
                        currentState = State.DROP_CONE;
                    }
                    break;
                case DROP_CONE:
                    if(System.currentTimeMillis() - currentTime >= 700){
                        robot.config.claw.setPosition(0.6);
                        if(System.currentTimeMillis() - currentTime >= 1150 ){
                            robot.config.rightPivot.setPosition(0.75);
                            robot.config.leftPivot.setPosition(0.25);
                            if(auto){
                                if(System.currentTimeMillis() - currentTime >= 1900){
                                    currentState = State.RAISE_VERT;
                                    auto = false;
                                }
                            }else{
                                currentState = State.IDLE;
                            }
                        }
                    }
                    break;
                case RAISE_VERT:
                    robot.config.leftVerticalSlide.setPower(1);
                    robot.config.rightVerticalSlide.setPower(1);
                    if(robot.config.leftVerticalSlide.getCurrentPosition() >= 2350){//-2350
                        robot.config.leftVerticalSlide.setPower(-1);
                        robot.config.rightVerticalSlide.setPower(-1);
                        currentState = State.PULL_IN_VERT;
                    }
                    break;
                case PULL_IN_VERT:
                    if(robot.config.leftVerticalSlide.getCurrentPosition() <= 0){
                        initialVerticalTicks = robot.config.leftVerticalSlide.getCurrentPosition();
                        robot.config.leftVerticalSlide.setPower(0);
                        robot.config.rightVerticalSlide.setPower(0);
                        currentState = State.IDLE;
                    }
                    break;
                case IDLE:
                    currentTime = System.currentTimeMillis();
                    verticalLiftTicks = robot.config.leftVerticalSlide.getCurrentPosition();
                    rightServo = robot.config.rightPivot.getPosition();
                    leftServo = robot.config.leftPivot.getPosition();
                    break;
            }
            if (-gamepad2.left_stick_y != 0) {
                robot.config.intakeDrawerSlideLeft.setPower(-gamepad2.left_stick_y);
                robot.config.intakeDrawerSlideRight.setPower(-gamepad2.left_stick_y);
            }else if(currentState != State.PULL_IN_SLIDES){
                robot.config.intakeDrawerSlideLeft.setPower(0);
                robot.config.intakeDrawerSlideRight.setPower(0);
            }
            if (gamepad1.left_bumper) {
                robot.config.leftVerticalSlide.setPower(0.7);
                robot.config.rightVerticalSlide.setPower(0.7);
            }else if (gamepad1.right_bumper){
                robot.config.leftVerticalSlide.setPower(-0.7);
                robot.config.rightVerticalSlide.setPower(-0.7);
            }else if(currentState != State.RAISE_VERT && currentState != State.PULL_IN_VERT){
                robot.config.leftVerticalSlide.setPower(0);
                robot.config.rightVerticalSlide.setPower(0);
            }
            if (gamepad2.dpad_left) { //up
                robot.armUp();
            }
            if (gamepad2.dpad_right) { // down
                robot.armDown();
            }
            if (gamepad2.b) {
                robot.config.claw.setPosition(0.6);
            }
            if (gamepad2.x) {
                robot.config.claw.setPosition(0.85);
            }
            if (gamepad2.right_bumper) {
                robot.config.claw.setPosition(0.7);
            }
            if (gamepad2.dpad_up) {
                currentState = State.PULL_IN_SLIDES;
            }
            if(gamepad2.a){
                auto = true;
            }
            if(gamepad2.y){
                robot.config.rightPivot.setPosition(0.5);
                robot.config.leftPivot.setPosition(0.5);
            }
            if(gamepad2.left_trigger != 0){
                rightServo += 0.01;
                leftServo -= 0.01;
                robot.config.rightPivot.setPosition(rightServo); //set pivots on forward positions
                robot.config.leftPivot.setPosition(leftServo);
            }
            if(gamepad2.right_trigger != 0){
                rightServo -= 0.01;
                leftServo += 0.01;
                robot.config.rightPivot.setPosition(rightServo); //set pivots on forward positions
                robot.config.leftPivot.setPosition(leftServo);
            }
            if(gamepad1.left_trigger != 0){
                currentState = State.RAISE_VERT;
            }
            if(gamepad1.a){
                robot.config.alignmentTool.setPosition(0.5);
            }
            if(gamepad1.y){
                robot.config.alignmentTool.setPosition(0.85);
            }
            if(gamepad2.left_bumper){
                currentState = State.IDLE;
            }
            robot.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x,
                            -gamepad1.right_stick_x
                    )
            );
        }
    }
}
