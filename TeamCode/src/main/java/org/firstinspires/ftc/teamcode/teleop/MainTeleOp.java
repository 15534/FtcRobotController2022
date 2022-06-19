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


//    // defining motors and servos
//    DcMotorEx intakeSurgical, intakeExtension, outtake, motorExLeft, carousel,
//            fleft, fright, bleft, bright;
//    Servo intakePosition, outtakeServo, fr, br, fl, bl, capperServo;
//
//    // intake constants
//    double mecDown = 0.095;
//    double intakeUp = 0.57;
//    double tankDown = 0.140;
//
//    // outtake constants
//    double outtakeDelay = 0.2;
//    public static int outtakeThirdLevelPosition = -360;
//    public static int outtakeFirstLevelPosition = -120;
//    public static int outtakeDownPosition = 0;
//    public static double outtakePower = 0.5;
//
//    // public static double outtakeServoClosePosition = 0.2;
//    // public static double outtakeServoOpenPosition = 0.7;
//
//    public static int intakeExtensionLowerLimit, intakeExtensionUpperLimit;
//    //public static = 270;
//    //public static double intakePower = 0.6;
//
//    public static double leftStickPos = 1;
//
//    public static double DPAD_SPEED = 0.18;
//    public static double BUMPER_ROTATION_SPEED = .7;
//    public static double ROTATION_MULTIPLIER = .7;
//
//    public static double TICKS_PER_REV = 537.6;

    public static boolean isMec = true;
    ElapsedTime runtime = new ElapsedTime();
//    public double timer = -1;

    @Override
    public void runOpMode() throws InterruptedException {
//        double lastvaluefront = -1;
//        double lastvaluelast = -1;
//        int startpoint = 0;
//        double power = 0.0;
//        double scale = 0.0;
//        double lastX = runtime.seconds();
//        double lastB = runtime.seconds();
//
//        boolean extended = false;
//
//        SampleMecanumDrive mecDrive = new SampleMecanumDrive(hardwareMap);
//        mecDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
//        SampleTankDrive tankDrive = new SampleTankDrive(hardwareMap);
//        tankDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
//        // defining gamepads
//        GamepadEx gp1 = new GamepadEx(gamepad1);
//        GamepadEx gp2 = new GamepadEx(gamepad2);
//
//        ButtonReader redCarousel = new ButtonReader(gp2, GamepadKeys.Button.B);
//        ButtonReader blueCarousel = new ButtonReader(gp2, GamepadKeys.Button.X);
//
//        ButtonReader capperDown = new ButtonReader(gp2, GamepadKeys.Button.A);
//        ButtonReader capperUp = new ButtonReader(gp2, GamepadKeys.Button.Y);
//
//        fl = hardwareMap.get(Servo.class, "frontleft"); // lower: 0.984, upper: 0
//        fr = hardwareMap.get(Servo.class, "frontright"); // lower: .1, upper: 1
//        br = hardwareMap.get(Servo.class, "backright"); // lower: 0.954, upper: 0
//        bl = hardwareMap.get(Servo.class, "backleft"); // lower: 0.02, upper: 1
//
//        capperServo = hardwareMap.get(Servo.class, "capperServo"); // registered in driver station
//
//        // drivetrain motor init
//        fleft = hardwareMap.get(DcMotorEx.class, "front_left");
//        fright = hardwareMap.get(DcMotorEx.class, "front_right");
//        bleft = hardwareMap.get(DcMotorEx.class, "rear_left");
//        bright = hardwareMap.get(DcMotorEx.class, "rear_right");
//
//        motorExLeft = (DcMotorEx)hardwareMap.get(DcMotor.class, "intake");
//        outtakeServo = hardwareMap.get(Servo.class, "outtake servo");
//        outtakeServo.setDirection(Servo.Direction.FORWARD);
//
//        intakeSurgical = hardwareMap.get(DcMotorEx.class, "intakeSurgical");
//        intakeSurgical.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
//        intakeSurgical.setDirection(DcMotor.Direction.FORWARD);
//        intakeSurgical.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//
//        carousel = hardwareMap.get(DcMotorEx.class,"caro");
//
//        outtake = hardwareMap.get(DcMotorEx.class, "intake");
//        outtake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
//        outtake.setDirection(DcMotor.Direction.REVERSE);
//        outtake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//
//        intakeExtension = hardwareMap.get(DcMotorEx.class, "intakeExtension");
//        intakeExtension.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
//        intakeExtension.setDirection(DcMotor.Direction.REVERSE);
//        intakeExtension.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//
//        //PIDCoefficients coeffs = new PIDCoefficients(kP, kI, kD);
//        //PIDFController controller = new PIDFController(coeffs, 0, 0, 0, (x, v) -> kG);
//
//        intakePosition = hardwareMap.get(Servo.class, "intakeLift");
//
//        FtcDashboard dashboard = FtcDashboard.getInstance();
//        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
//
//        // init robot
//        intakeExtension.setTargetPosition(-60);
//        intakeExtension.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        intakeExtension.setPower(-0.5);
//
//        intakeExtensionLowerLimit = intakeExtension.getCurrentPosition();
//        intakeExtensionUpperLimit = intakeExtensionLowerLimit + 270;
//
//        intakePosition.setPosition(intakeUp);
//        motorExLeft.setTargetPosition(outtakeDownPosition);
//        outtakeServo.setPosition(outtakeServoLowerLimit);

//        double scaler = 0.6;

        driveTrain.init();
        capper.init();
        intake.init();

//        boolean a = true;
//        boolean b = true;
//        boolean x = true;
//        boolean y = true;
//        boolean pressed = true;
//
//        boolean toggleOff = false;

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
//            telemetry.addData("scaler", scaler);
//            telemetry.addData("intake servo position: ", intakePosition.getPosition());
//            telemetry.addData("outtakePosition: ", outtake.getCurrentPosition());
            //telemetry.addData("upper limit: ", intakeExtensionUpperLimit);
            //telemetry.addData("lower limit: ", intakeExtensionLowerLimit);

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
                    capperServo.setPosition(capperServo.getPosition() + capStep);
                }
            } else if (gamepad2.y) {
                // capper up
                if (capperServo.getPosition() - capStep > capInit) {
                    capperServo.setPosition(capperServo.getPosition() - capStep);
                }
            }

            if (gamepad2.right_bumper) {
                // cap servo pos increases slowly
                if (capperServo.getPosition() + capStep < capDown) {
                    capperServo.setPosition(capperServo.getPosition() + capStepSlow);
                }
            } else if (gamepad2.left_bumper) {
                // cap servo pos decreases slowly
                if (capperServo.getPosition() - capStep > capInit) {
                    capperServo.setPosition(capperServo.getPosition() - capStepSlow);
                }
            }

            // carousel
//            if (gamepad2.x) {
//                if (carousel.getCurrentPosition() <= TICKS_PER_REV * 2 / 3 * 6) {
//                    carousel.setPower(0.6);
//                } else {
//                    carousel.setPower(1);
//                }
//            }
//            if (gamepad2.b) {
//                if (carousel.getCurrentPosition() <= -(TICKS_PER_REV * 2 / 3 * 6)) {
//                    carousel.setPower(-0.6);
//                } else {
//                    carousel.setPower(-1);
//                }
//            }

            boolean noCarousel = true;
            if (gamepad2.x || gamepad2.b) {
                driveTrain.duckTension();
            }

            if (gamepad2.x) {
                noCarousel = false;
                if (runtime.seconds() - lastX < 1.25) {
                    carousel.setPower(1);
                } else {
                    carousel.setPower(0.5);
                }
            } else {
                lastX = runtime.seconds();
            }

            if (gamepad2.b) {
                noCarousel = false;
                if (runtime.seconds() - lastB < 1.25) {
                    carousel.setPower(-0.5);
                } else {
                    carousel.setPower(-1);
                }
            } else {
                lastB = runtime.seconds();
            }

            if (noCarousel) carousel.setPower(0);

            //telemetry.addData("red carousel down: ", redCarousel.isDown());

            // surgical tubing
            if (gamepad2.right_trigger < 0.5 && gamepad2.left_trigger < 0.5) {
                intakeSurgical.setPower(0);
            }

            if (gamepad2.right_trigger > 0.5) {
                intakeSurgical.setPower(gamepad2.right_trigger);
            }

            if (gamepad2.left_trigger > 0.5) {
                intakeSurgical.setPower(-1 * gamepad2.left_trigger);
            }

            //telemetry.addData("gamepad stick position: ", gamepad2.right_stick_y);
            //telemetry.update();

            // intake servo
            //range for stick is -1 at top and 1 at bottom
            // intake comes down
            if (gamepad2.right_stick_y > 0.1) {
                intakePosition.setPosition(intakeUp); // tank mode down
            }

            if (gamepad2.right_stick_y < -0.1) {
                if(isMec) {
                    intakePosition.setPosition(mecDown);
                }else{
                    intakePosition.setPosition(tankDown);// general servo up position
                }
            }

            //double function = 0.1;
            //if(pos>0){
            //  pos+=intakeExtension.getPower()*20;
            //function = Math.max(4.9*Math.sqrt(pos/300.0)*Math.exp(-1.0*pos/50.0),0.1);
            //}


            // intake extension motor
            if (gamepad2.left_stick_y < -0.02) {
                double joystickPosition = gamepad2.left_stick_y;
//                telemetry.addData("curr pos: ", intakeExtension.getCurrentPosition());
//                telemetry.addData("statement: ", intakeExtension.getCurrentPosition() > intakeExtensionLowerLimit);

                if (intakeExtension.getCurrentPosition() >= intakeExtensionLowerLimit) {
                    //telemetry.addData("pos to move: ", (intakeExtensionLowerLimit + joystickPosition * 270 * (-1)));
//                    telemetry.addData("statement: ", gamepad2.left_stick_y < -0.1);
//                    telemetry.addData("pos: ", (int) (joystickPosition * 270 * (-1)));
//
                    //telemetry.addData("test", 0);
                    intakeExtension.setTargetPosition((int) (intakeExtensionLowerLimit + joystickPosition * 270 * (-1)));
                    intakeExtension.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    intakeExtension.setPower(0.7);
                }
                //extended = true;
            }

            if (gamepad2.left_stick_y >= 0) { // what code makes left_stick_y > 0.02 go in then?
                intakeExtension.setTargetPosition(intakeExtensionLowerLimit);
                intakeExtension.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                intakeExtension.setPower(-0.5);
                //extended = false;
            }

            if (intakeExtension.getCurrentPosition() >= intakeExtensionLowerLimit && intakeExtension.getCurrentPosition() <= intakeExtensionLowerLimit + 20) {
                if (gamepad2.left_stick_y > -0.02 && gamepad2.left_stick_y < 0.02) {
                    //telemetry.addLine("pulling back");
                    intakeExtension.setTargetPosition(intakeExtensionLowerLimit);
                    intakeExtension.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    intakeExtension.setPower(-0.001);
                }
            }

            //telemetry.addData("joystick: ", gamepad2.left_stick_y);
            //telemetry.addData("intake extension pos: ", intakeExtension.getCurrentPosition());
//          telemetry.addData("extended: ", extended);

            // outtake

            if (gamepad2.dpad_up) {
                motorExLeft.setTargetPosition(outtakeThirdLevelPosition);
                motorExLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motorExLeft.setPower(outtakePower);
            }

            if (gamepad2.dpad_left) {
                motorExLeft.setTargetPosition(outtakeFirstLevelPosition);
                motorExLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motorExLeft.setPower(outtakePower);
            }

            if (gamepad2.dpad_right) {
//                if(intakeExtension.getCurrentPosition()-intakeExtensionLowerLimit<100){
//                    intakeExtension.setPower(0.2);
//                }

                outtakeServo.setPosition(outtakeServoLimitUpperMidHub);
            }

            if (gamepad2.dpad_down) {
                motorExLeft.setTargetPosition(outtakeDownPosition);
                motorExLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motorExLeft.setPower(outtakePower*0.6);

                outtakeServo.setPosition(outtakeServoLowerLimit);
            }
            //telemetry.addData("intake Position: ", intakePosition.getPosition());

            if(gamepad1.right_trigger>0.6&&pressed){
                if(isMec){
                    // switcing from mec to tank
                    if(intakePosition.getPosition() > mecDown - 0.01 && intakePosition.getPosition() < mecDown + 0.01){
                        intakePosition.setPosition(tankDown);
                    }
                    driveTrain.mecToTank();

                    isMec=false;

                }else{
                    // switching from tank to mec

                    driveTrain.tankToMec();

                    sleep(200); // delay 200 milliseconds so no intake slamming upon switch

                    if(intakePosition.getPosition() > tankDown - 0.01 && intakePosition.getPosition() < tankDown + 0.01){
                        intakePosition.setPosition(mecDown);
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

        // dpad left - tier 1
        // dpad right - open outtake servo that contains freight
        // dpad down - return to position 0 and close outtake servo as well

//
//            if (gamepad2.left_stick_y > 0.0) {
//                if (intakeExtension.getCurrentPosition() < intakeExtensionUpperLimit
//                    && intakeExtension.getCurrentPosition() > intakeExtensionLowerLimit) {
//                    intakeExtension.setPower(gamepad2.left_stick_y);
//                }
//            }

        //telemetry.addData("power: ", power);
        //telemetry.addData("scale", scale);

    }
}
