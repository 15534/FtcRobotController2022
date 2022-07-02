package org.firstinspires.ftc.teamcode.teleop;

import static org.firstinspires.ftc.teamcode.Constants.*;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.drive.Drive;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.gamepad.ButtonReader;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.PoseStorage;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.SampleTankDrive;
import org.firstinspires.ftc.teamcode.subsystems.*;


@TeleOp(name = "TeleOp")
@Config
public class MainTeleOp extends LinearOpMode {
    DriveTrain driveTrain = new DriveTrain(hardwareMap);
    Capper capper = new Capper(hardwareMap);
    Outtake outtake = new Outtake(hardwareMap);
    Intake intake = new Intake(hardwareMap);
    Carousel carousel = new Carousel(hardwareMap);

    public static boolean isMec = true;
    ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {

        driveTrain.init();
        capper.init();
        intake.init();

        waitForStart();

        while (!isStopRequested()) {
            double elapsed = runtime.seconds() - time;

            driveTrain.getMecDrive().update();
            driveTrain.getTankDrive().update();

            Pose2d poseEstimate;
            if (isMec) {
                poseEstimate = driveTrain.getMecDrive().getPoseEstimate();
            } else {
                poseEstimate = driveTrain.getTankDrive().getPoseEstimate();
            }
            PoseStorage.currentPose = poseEstimate;

            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.addData("ie lower limit:", intakeExtensionLowerLimit);
            telemetry.addData("ie upper limit:", intakeExtensionUpperLimit);

            redCarousel.readValue();
            blueCarousel.readValue();

            capperDown.readValue();
            capperUp.readValue();

            Vector2d translation = new Vector2d(-gamepad1.left_stick_y*scaler, -gamepad1.left_stick_x*scaler);
            double rotation = -ROTATION_MULTIPLIER * gamepad1.right_stick_x * scaler;

            // Gamepad 1

            // slow translation with dpad
            if (gamepad1.dpad_up) {
                translation = new Vector2d(DPAD_SPEED, 0);
            } else if (gamepad1.dpad_down) {
                translation = new Vector2d(-DPAD_SPEED, 0);
            } else if (gamepad1.dpad_left) {
                translation = new Vector2d(0, DPAD_SPEED);
            } else if (gamepad1.dpad_right) {
                translation = new Vector2d(0, -DPAD_SPEED);
            }

            // making speed slower when left trigger is pressed
            // tank mode should only have value != 0 in x component
            if (gamepad1.left_trigger > 0) {
                if (translation.getX() != 0) {
                    translation = new Vector2d(translation.getX() * 1/2, 0);
                } else {
                    translation = new Vector2d(0, translation.getY() * 1/2);
                }
            }

            //telemetry.addData("left trigger: ", gamepad1.left_trigger);

            // slow rotation with bumpers
            if (gamepad1.left_bumper) {
                rotation = BUMPER_ROTATION_SPEED;
            } else if (gamepad1.right_bumper) {
                rotation = -BUMPER_ROTATION_SPEED;
            }

            if (isMec) {
                mecDrive.setWeightedDrivePower(new Pose2d(translation, rotation));
            } else {
                // y in tank should always be 0!
                if (gamepad1.left_trigger > 0) {
                    if (translation.getX() != 0) {
                        translation = new Vector2d(translation.getX() * 1/2, 0);
                    } else {
                        translation = new Vector2d(translation.getY() * 1/2, 0);
                    }
                    rotation *= 1/2;
                } else {
                    translation = new Vector2d(-gamepad1.left_stick_y*scaler, 0.0);
                }
                tankDrive.setWeightedDrivePower(new Pose2d(translation, rotation));
            }

            // capper
            if (gamepad2.a) {
                // capper down
                if (capperServo.getPosition() + capStep < capDown) {
                    capper.down();
                }
            } else if (gamepad2.y) {
                // capper up
                if (capperServo.getPosition() - capStep > capInit) {
                    cap.up();
                }
            }

            if (gamepad2.right_bumper) {
                // cap servo pos increases slowly
                if (capperServo.getPosition() + capStep < capDown) {
                    capper.downSlow();
                }
            } else if (gamepad2.left_bumper) {
                // cap servo pos decreases slowly
                if (capperServo.getPosition() - capStep > capInit) {
                    capper.upSlow();
                }
            }

            boolean noCarousel = true;
            if (gamepad2.x || gamepad2.b) {
                driveTrain.duckTension();
            }

            if (gamepad2.x) {
                noCarousel = false;
                carousel.SpinX(runtime, lastX);
            } else {
                lastX = runtime.seconds();
            }

            if (gamepad2.b) {
                noCarousel = false;
                carousel.SpinB(runtime, lastX);
            } else {
                lastB = runtime.seconds();
            }

            if (noCarousel) { carousel.getCarouselSpinner().setPosition(0); }

            //telemetry.addData("red carousel down: ", redCarousel.isDown());

            // surgical tubing
            double intakeSurgicalPower = intake.intakeSurgicalPower(gamepad2.right_trigger, gamepad2.left_trigger);
            intake.getIntakeSurgical.setPower(intakeSurgicalPower);

            //telemetry.addData("gamepad stick position: ", gamepad2.right_stick_y);
            //telemetry.update();

            // intake servo
            //range for stick is -1 at top and 1 at bottom
            // intake comes down
            double intakePositionValue = intake.intakePosition(gamepad2.right_stick_y, isMec);
            intakePosition.setPosition(intakePositionValue);

            // intake extension motor
            if (gamepad2.left_stick_y < -0.02) {
                double joystickPosition = gamepad2.left_stick_y;
                intake.extend(joystickPosition);
            }

            if (gamepad2.left_stick_y >= 0) { // what code makes left_stick_y > 0.02 go in then?
                intake.retract();
            }

            if (intakeExtension.getCurrentPosition() >= intakeExtensionLowerLimit && intakeExtension.getCurrentPosition() <= intakeExtensionLowerLimit + 20) {
                if (gamepad2.left_stick_y > -0.02 && gamepad2.left_stick_y < 0.02) {
                    intake.holdIn();
                }
            }

            // outtake

            if (gamepad2.dpad_up) {
                int outtakeLevel = 3;
                outtake.outtakeRaise(outtakeLevel);
            }

            if (gamepad2.dpad_left) {
                int outtakeLevel = 1;
                outtake.outtakeRaise(outtakeLevel);
            }

            if (gamepad2.dpad_right) {
                int outtakeLevel = 2; // can also be 3
                outtake.outtakeRelease(outtakeLevel);
            }

            if (gamepad2.dpad_down) {
                outtake.outtakeDown();
            }

            if(gamepad1.right_trigger>0.6 && pressed){
                if (isMec) {
                    // switcing from mec to tank
                    if (intakePosition.getPosition() > mecDown - 0.01 && intakePosition.getPosition() < mecDown + 0.01) {
                        intake.tankDown();
                    }
                    driveTrain.mecToTank();

                    isMec=false;

                }else{
                    // switching from tank to mec

                    driveTrain.tankToMec();

                    sleep(200); // delay 200 milliseconds so no intake slamming upon switch

                    if(intakePosition.getPosition() > tankDown - 0.01 && intakePosition.getPosition() < tankDown + 0.01){
                        intake.mecDown();
                    }

                    isMec=true;

                }
                pressed=false;
            }

            if(gamepad1.right_trigger<0.6){
                pressed=true;
            }

            if (gamepad1.a&&a) {
                scaler=Math.max(0.2, scaler-0.2);
                a=false;
            }
            if(!gamepad1.a){
                a=true;
            }
            if (gamepad1.b&&b) {
                scaler=Math.min(1, scaler+0.2);
                b=false;
            }
            if(!gamepad1.b){
                b=true;
            }

            if (gamepad1.x&&x) {
                scaler=0.6;

            }
            if(!gamepad1.x){
                x=true;
            }
            if (gamepad1.y&&y) {
                scaler=1;

            }
            if(!gamepad1.y){
                y=true;
            }

            telemetry.update();

        }

    }
}
