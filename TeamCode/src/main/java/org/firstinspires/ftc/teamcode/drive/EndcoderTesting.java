package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class EndcoderTesting extends LinearOpMode {

    private double pivotConstant = 0.0;
    private double leftConstant = 0.0;
    private double rightConstant = 0.0;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive robot = new SampleMecanumDrive(hardwareMap);
        robot.config.clawLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.config.clawLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

        for(;;) {
            if (isStopRequested()){
                break;
            } else if (-gamepad2.left_stick_y != 0) {
                robot.config.rightVerticalSlide.setPower(-gamepad2.left_stick_y);
                robot.config.leftVerticalSlide.setPower(gamepad2.left_stick_y);
            } else if (-gamepad2.right_stick_y != 0) {
                robot.config.intakeDrawerSlide.setPower(-gamepad2.right_stick_y);//going up goes back in
            } else if (gamepad2.dpad_left) {
                robot.config.rightRoller.setPower(-1);
                robot.config.leftRoller.setPower(1);
            } else if (gamepad2.dpad_right) {
                robot.config.rightRoller.setPower(1);
                robot.config.leftRoller.setPower(-1);
            } else if (gamepad2.y) {
                robot.config.clawPivot.setPosition(0.4);
            } else if (gamepad2.a) {
                robot.config.clawPivot.setPosition(1);
            } else if (gamepad2.b) {
                robot.config.rightClaw.setPosition(0.5);
            } else if (gamepad2.x) {
                robot.config.rightClaw.setPosition(0.1);
            } else if(gamepad1.a){
                robot.config.clawLift.setPower(0.5);
            } else if(gamepad2.right_bumper){
                robot.config.rightClaw.setPosition(0.3);
            } else if(gamepad1.left_bumper){
                robot.config.clawLift.setPower(0.5);
            } else if(gamepad1.right_bumper){
                robot.config.clawLift.setPower(-0.5);
            } else if(gamepad2.left_trigger != 0){
                robot.config.horizontalSlide.setPower(-1);
            } else if(gamepad2.right_trigger != 0){
                robot.config.horizontalSlide.setPower(1);
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
                telemetry.addData("Intake Drawer slides", robot.config.intakeDrawerSlide.getCurrentPosition() + "\n");
                telemetry.addData("Claw Pivot", robot.config.clawLift.getCurrentPosition() + "\n");
                telemetry.update();
                robot.update();
            }
        }
    }
}
