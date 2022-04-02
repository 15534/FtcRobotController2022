package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.ServoConstants.*;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.PoseStorage;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.SampleTankDrive;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@Config
@Autonomous(name = "RedAutoV2")
public class RedAutoV2 extends LinearOpMode {
    OpenCvWebcam webcam;
    CVPipeline pipeline;

    DcMotorEx intakeExtension, carousel, fleft, fright, bleft, bright, outtake, intakeSurgical;

    public static int intakeExtensionLowerLimit, intakeExtensionUpperLimit;

    public SampleMecanumDrive mecanumDrive;
    public SampleTankDrive tankDrive;

    double time = 0.0;
    int position = 3;
    ElapsedTime runtime = new ElapsedTime();
    State currentState = State.IDLE;
    LinearOpMode op;

    Pose2d startingPosition = new Pose2d(-59, -56, Math.toRadians(67.61986495));

    Trajectory parkWarehouse, goToShippingHubFromCarousel2,  goToShippingHubFromCarousel,
            goToTeamCubeFromShippingHub2, goToTeamCubeFromShippingHub1,
            goToAllianceFreightFromShippingHub2, goToShippingHub,
            intakeFreight, driveBack;

    Servo fr, br, fl, bl, outtakeServo, intakePosition;

//    public static double outtakePower = 0.5;
//    public static double outtakeServoClosePosition = 0.2;
//    public static double outtakeServoOpenPosition = 0.7;
//    public static int outtakeDownPosition = 0;

    public static boolean isMec = false;

    enum State {
        GO_TO_CAROUSEL,
        KNOCK_OFF_DUCK,
        GO_TO_SHIPPING_HUB,
        GO_TO_SHIPPING_HUB_TURN,
        GO_TO_ALLIANCE_FREIGHT,
        SCORE_FREIGHT_IN_SHIPPING_HUB,
        SPIN,
        IDLE, GO_TO_SHIPPING_HUB_2,
        SHIPPING_HUB_ALL1, PARK_IN_WAREHOUSE, GO_TO_ALLIANCE_FREIGHT_2, MOVE_OUTTAKE_UP, GO_TO_SHIPPING_HUB_PARK, GO_TO_SHIPPING_HUB_PARK_2, INTAKE_FREIGHT, RESET_FOR_TELEOP, SHIPPING_HUB_ALL2
    }

    void next(State s) {
        time = runtime.seconds();
        currentState = s;
    }

    int turnsLeft = 3;

    public void buildTrajectories() {
//        goToCarouselFromStarting = tankDrive.trajectoryBuilder(startingPosition)
//                .back(4)
//                .build();

        goToShippingHubFromCarousel = tankDrive.trajectoryBuilder(startingPosition)
                //.lineToSplineHeading(shippingHubPose)
                //.lineTo(new Vector2d(shippingHubPose.getX(),shippingHubPose.getY()))
                .forward(27.586228)
                .build();

        goToShippingHubFromCarousel2 = tankDrive.trajectoryBuilder(goToShippingHubFromCarousel.end())
                //.lineToSplineHeading(shippingHubPose)
                //.lineTo(new Vector2d(shippingHubPose.getX(),shippingHubPose.getY()))
                //.lineTo(new Vector2d(shippingHubPose.getX(), shippingHubPose.getY()))
                .back(16)
                .build();

        goToTeamCubeFromShippingHub1 = tankDrive.trajectoryBuilder(goToShippingHubFromCarousel2.end())
                .forward(6)
                .build();

        goToTeamCubeFromShippingHub2 = tankDrive.trajectoryBuilder(goToTeamCubeFromShippingHub1.end())
                .forward(22)
                .build();

        goToShippingHub = tankDrive.trajectoryBuilder(goToTeamCubeFromShippingHub2.end())
                .forward(74)
                .build();

        intakeFreight = tankDrive.trajectoryBuilder(goToShippingHub.end())
                .forward(8)
                .build();

        driveBack = tankDrive.trajectoryBuilder(intakeFreight.end())
                .back(8)
                .build();
    }

    public void switchFromMecToTank() {
        fl.setPosition(0);
        bl.setPosition(1);
        fr.setPosition(1);
        br.setPosition(0);
        isMec = false;
    }

    public void switchFromTankToMec() {
        fl.setPosition(flMec);
        fr.setPosition(frMec);
        br.setPosition(brMec);
        bl.setPosition(blMec);
        isMec = true;
    }

    public void runOpMode() throws InterruptedException {


        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam"), cameraMonitorViewId);
        pipeline = new CVPipeline(true);
        webcam.setPipeline(pipeline);
        webcam.setMillisecondsPermissionTimeout(2500);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {}
        });

        mecanumDrive = new SampleMecanumDrive(hardwareMap);
        mecanumDrive.setPoseEstimate(startingPosition);
        tankDrive  = new SampleTankDrive(hardwareMap);

        carousel = hardwareMap.get(DcMotorEx.class,"caro");
        fleft = hardwareMap.get(DcMotorEx.class, "front_left");
        fright = hardwareMap.get(DcMotorEx.class, "front_right");
        bleft = hardwareMap.get(DcMotorEx.class, "rear_left");
        bright = hardwareMap.get(DcMotorEx.class, "rear_right");

        outtake = hardwareMap.get(DcMotorEx.class, "intake");
        outtake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        outtake.setDirection(DcMotor.Direction.REVERSE);
        outtake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        intakePosition = hardwareMap.get(Servo.class, "intakeLift");
        fl = hardwareMap.get(Servo.class, "frontleft");
        // limits: 0.98 & 0
        br = hardwareMap.get(Servo.class, "backright");
        // limits: 0.98 & 0
        fr = hardwareMap.get(Servo.class, "frontright");
        // limits: 0.0175 &
        bl = hardwareMap.get(Servo.class, "backleft");
        // limits: 0.034 & 1
        // tank is whole numbers
        //PoseStorage.currentPose = startingPosition;
//        fl.setPosition(0.984);
//        fr.setPosition(0.1);
//        br.setPosition(0.955);
//        bl.setPosition(0.02);

        fl.setPosition(0);
        bl.setPosition(1);
        fr.setPosition(1);
        br.setPosition(0);

        intakeExtension = hardwareMap.get(DcMotorEx.class, "intakeExtension");
        intakeExtension.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        intakeExtension.setDirection(DcMotor.Direction.REVERSE);

        intakeSurgical = hardwareMap.get(DcMotorEx.class, "intakeSurgical");
        intakeSurgical.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        intakeSurgical.setDirection(DcMotor.Direction.FORWARD);
        intakeSurgical.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        outtakeServo = hardwareMap.get(Servo.class, "outtake servo");
        outtakeServo.setDirection(Servo.Direction.FORWARD);

        buildTrajectories();

        runtime.reset();
        intakePosition.setPosition(intakeUp);
        outtakeServo.setPosition(outtakeServoLowerLimit);

//        intakeExtension.setTargetPosition(-45);
//        intakeExtension.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        intakeExtension.setPower(-0.05);

        intakeExtensionLowerLimit = intakeExtension.getCurrentPosition();
        intakeExtensionUpperLimit = intakeExtensionLowerLimit + 270;

//        OPEN CV
//        while (!opModeIsActive() && !isStopRequested()) {
//            telemetry.addData("avg1 - red", pipeline.getAvg1());
//            telemetry.addData("avg2 - blue", pipeline.getAvg2());
//            telemetry.addData("avg3 - green", pipeline.getAvg3());
//
//            position = pipeline.getPosition();
//            telemetry.addData("position", position);
//            mecanumDrive.setPoseEstimate(startingPosition);
//            tankDrive.setPoseEstimate(startingPosition);
//        }

        mecanumDrive.setPoseEstimate(startingPosition);
        tankDrive.setPoseEstimate(startingPosition);

        waitForStart();
        mecanumDrive.turnAsync(Math.toRadians(10));
        next(State.KNOCK_OFF_DUCK);
//        next(State.GO_TO_SHIPPING_HUB_2);
//        next(State.INTAKE_ALLIANCE);

        while(opModeIsActive()) {
            telemetry.addLine("running");

            telemetry.addData("outtake servo pos: ", outtakeServoLowerLimit);

            double elapsed = runtime.seconds() - time;
            switch (currentState) {
                case KNOCK_OFF_DUCK:
                    if (elapsed < 2.5) {
                        // should spin for 2.5 seconds
                        carousel.setPower(-0.45);
                        fleft.setPower(-0.1);
                        fright.setPower(-0.1);
                        bleft.setPower(-0.1);
                        bright.setPower(-0.1);
                    } else {
                        carousel.setPower(0.0);
                        fleft.setPower(0.0);
                        fright.setPower(0.0);
                        bleft.setPower(0.0);
                        bright.setPower(0.0);
                    }
                    if (!mecanumDrive.isBusy() && elapsed >= 3.5) {
                        tankDrive.followTrajectoryAsync(goToShippingHubFromCarousel);
                        next(State.GO_TO_SHIPPING_HUB);
                    }
                    break;
                case GO_TO_SHIPPING_HUB:
                    // path to go to shipping hub
                    if (!tankDrive.isBusy()) {
                        next(State.GO_TO_SHIPPING_HUB_TURN);
                    }
                    break;
                case GO_TO_SHIPPING_HUB_TURN:
                    if (!tankDrive.isBusy()) {
                        switchFromTankToMec();
                        sleep(200);
                        mecanumDrive.turn(Math.toRadians(128));
                        switchFromMecToTank();
                        sleep(200);
                        next(State.GO_TO_SHIPPING_HUB_2);
                    }
                    break;
                case GO_TO_SHIPPING_HUB_2:
                    if (!tankDrive.isBusy()) {
                        tankDrive.followTrajectoryAsync(goToShippingHubFromCarousel2);
                        next(State.MOVE_OUTTAKE_UP);
                    }
                    break;
                case MOVE_OUTTAKE_UP:
                    if (!tankDrive.isBusy()) {
                        intakeExtension.setTargetPosition(100);
                        intakeExtension.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        intakeExtension.setPower(0.5);
                        if (position == 1) {
                            outtake.setTargetPosition(outtakeFirstLevelPosition);
                        } else if (position == 2) {
                            outtake.setTargetPosition(outtakeSecondLevelPosition);
                        } else {
                            //outtake.setTargetPosition(outtakeThirdLevelPosition);
                            outtake.setTargetPosition(-330);
                        }

                        outtake.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        outtake.setPower(outtakePower);
                        next(State.SCORE_FREIGHT_IN_SHIPPING_HUB);
                    }
                    break;
                case SCORE_FREIGHT_IN_SHIPPING_HUB:
                    if (!tankDrive.isBusy()) {
                        if (elapsed < 0.2) {
                            switchFromTankToMec();
                            sleep(200);
                        } else if (elapsed < 0.7) {
                            //sleep(500);
                            if (position == 2 || position == 3) {
                                outtakeServo.setPosition(outtakeServoLimitUpperMidHub);
                            } else {
                                outtakeServo.setPosition(outtakeServoLimitLowerHub);
                            }
                            telemetry.addData("outtake servo position: ", outtakeServo.getPosition());
                        }else if (elapsed < 1) {
                            //switchFromMecToTank();
//                            tankDrive.followTrajectory(tankDrive.trajectoryBuilder(PoseStorage.currentPose).back(1).build());
                        } else if (elapsed < 1.8) {
                            outtakeServo.setPosition(outtakeServoLowerLimit);
                            outtake.setTargetPosition(outtakeFirstLevelPosition);
                            outtake.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                            outtake.setPower(outtakePower);
                        } else {
                            tankDrive.followTrajectoryAsync(goToTeamCubeFromShippingHub1);
                            next(State.GO_TO_ALLIANCE_FREIGHT);
                        }
                    }
                    break;
//                case PARK_IN_WAREHOUSE:
//                    if (!tankDrive.isBusy()) {
//                        tankDrive.followTrajectoryAsync(parkWarehouse);
//                        next(State.IDLE); // CODE ENDS HERE
//                    }
//                    break;
                case GO_TO_ALLIANCE_FREIGHT:
                    if (!tankDrive.isBusy()) {
                        switchFromTankToMec();
                        sleep(200);
                        mecanumDrive.turn(Math.toRadians(128));
                        switchFromMecToTank();
                        sleep(200);
                        next(State.GO_TO_ALLIANCE_FREIGHT_2);
                    }
                    break;
                case GO_TO_ALLIANCE_FREIGHT_2:
                    if (!tankDrive.isBusy()) {
//                        switchFromTankToMec();
//                        sleep(200);;
//                        mecanumDrive.turn(Math.toRadians(114));
//                        switchFromMecToTank();
//                        sleep(200);;
                        tankDrive.followTrajectoryAsync(goToTeamCubeFromShippingHub2);
                        next(State.GO_TO_SHIPPING_HUB_PARK);
                    }
                    break;
                case GO_TO_SHIPPING_HUB_PARK:
                    if (!tankDrive.isBusy()) {
                        switchFromTankToMec();
                        sleep(200);
                        mecanumDrive.turn(Math.toRadians(36));
                        switchFromMecToTank();
                        sleep(200);
                        next(State.GO_TO_SHIPPING_HUB_PARK_2);
                    }
                    break;
                case GO_TO_SHIPPING_HUB_PARK_2:
                    if (!tankDrive.isBusy()) {
                        tankDrive.followTrajectoryAsync(goToShippingHub);
                        next(State.SPIN);
                    }
                    break;
                case SPIN:
                    if (!tankDrive.isBusy()) {
                        intakePosition.setPosition(intakeTankDown);
                        switchFromTankToMec();
                        sleep(200);
                        mecanumDrive.turn(Math.toRadians(-45));
                        switchFromMecToTank();
                        sleep(200);
                        next(State.INTAKE_FREIGHT);
                    }
                    break;
                case INTAKE_FREIGHT:
                    if (!tankDrive.isBusy()) {
                        tankDrive.followTrajectoryAsync(intakeFreight);
                        if (elapsed < 2.5) {
                            intakeSurgical.setPower(1);
                        } else {
                            intakeSurgical.setPower(0);
                            intakeExtension.setTargetPosition(intakeExtensionLowerLimit);
                            intakeExtension.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                            intakeExtension.setPower(-0.05);
                            next(State.RESET_FOR_TELEOP);
                        }
                    }
                    break;
                case RESET_FOR_TELEOP:
                    if (!tankDrive.isBusy()) {
                        tankDrive.followTrajectoryAsync(driveBack);
                        next(State.IDLE);
                    }
                    break;
            }
            // Read pose
            Pose2d poseEstimate;
            if (isMec) {
                poseEstimate = mecanumDrive.getPoseEstimate();
            } else {
                poseEstimate = tankDrive.getPoseEstimate();
            }

            PoseStorage.currentPose = poseEstimate;
            tankDrive.update();
            mecanumDrive.update();

            // Print pose to telemetry
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.addData("state", currentState);
            telemetry.addData("MODE", isMec);
            telemetry.addData("fr motor: ", fright.getPower());
            telemetry.addData("br motor: ", bright.getPower());
            telemetry.addData("bl motor: ", bleft.getPower());
            telemetry.addData("fl motor: ", fleft.getPower());


            //telemetry.addData("elapsed", elapsed);
//            telemetry.addData("push", push);
            telemetry.update();
        }
    }
}
