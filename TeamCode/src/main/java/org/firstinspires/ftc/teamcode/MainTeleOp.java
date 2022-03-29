package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.ServoConstants.*;
import static org.firstinspires.ftc.teamcode.ServoConstants.outtakeFirstLevelPosition;
import static org.firstinspires.ftc.teamcode.ServoConstants.outtakePower;
import static org.firstinspires.ftc.teamcode.ServoConstants.outtakeThirdLevelPosition;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
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

@TeleOp(name = "TeleOp")
@Config
public class MainTeleOp extends LinearOpMode {

    // defining motors and servos
    DcMotorEx intakeSurgical, intakeExtension, outtake, motorExLeft, carousel,
            fleft, fright, bleft, bright;
    Servo intakePosition, outtakeServo, fr, br, fl, bl, capperServo;

    // intake constants
    double mecDown = 0.1;
    double intakeUp = 0.58;
    double tankDown = 0.148;

    // outtake constants
    double outtakeDelay = 0.2;
    public static int outtakeThirdLevelPosition = -360;
    public static int outtakeFirstLevelPosition = -120;
    public static int outtakeDownPosition = 0;
    public static double outtakePower = 0.5;

    // public static double outtakeServoClosePosition = 0.2;
    // public static double outtakeServoOpenPosition = 0.7;

    public static int intakeExtensionLowerLimit = -70;
    public static int intakeExtensionUpperLimit = 270;
    public static double intakePower = 0.6;

    public static double leftStickPos = 1;

    public static double DPAD_SPEED = 0.35;
    public static double BUMPER_ROTATION_SPEED = .7;
    public static double ROTATION_MULTIPLIER = .7;

    public static double TICKS_PER_REV = 537.6;

    public static boolean isMec = true;
    ElapsedTime runtime = new ElapsedTime();
    public double timer = -1;
    @Override
    public void runOpMode() throws InterruptedException {
        double lastvaluefront = -1;
        double lastvaluelast = -1;
        int startpoint = 0;
        double power = 0.0;
        double scale = 0.0;
        double lastX = runtime.seconds();
        double lastB = runtime.seconds();

        boolean extended = false;

        SampleMecanumDrive mecDrive = new SampleMecanumDrive(hardwareMap);
        mecDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        SampleTankDrive tankDrive = new SampleTankDrive(hardwareMap);
        tankDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // defining gamepads
        GamepadEx gp1 = new GamepadEx(gamepad1);
        GamepadEx gp2 = new GamepadEx(gamepad2);

        ButtonReader redCarousel = new ButtonReader(gp2, GamepadKeys.Button.B);
        ButtonReader blueCarousel = new ButtonReader(gp2, GamepadKeys.Button.X);

        ButtonReader capperDown = new ButtonReader(gp2, GamepadKeys.Button.A);
        ButtonReader capperUp = new ButtonReader(gp2, GamepadKeys.Button.Y);

        fl = hardwareMap.get(Servo.class, "frontleft"); // lower: 0.984, upper: 0
        fr = hardwareMap.get(Servo.class, "frontright"); // lower: .1, upper: 1
        br = hardwareMap.get(Servo.class, "backright"); // lower: 0.954, upper: 0
        bl = hardwareMap.get(Servo.class, "backleft"); // lower: 0.02, upper: 1

        capperServo = hardwareMap.get(Servo.class, "capperServo"); // registered in driver station

        // drivetrain motor init
        fleft = hardwareMap.get(DcMotorEx.class, "front_left");
        fright = hardwareMap.get(DcMotorEx.class, "front_right");
        bleft = hardwareMap.get(DcMotorEx.class, "rear_left");
        bright = hardwareMap.get(DcMotorEx.class, "rear_right");

        motorExLeft = (DcMotorEx)hardwareMap.get(DcMotor.class, "intake");
        outtakeServo = hardwareMap.get(Servo.class, "outtake servo");
        outtakeServo.setDirection(Servo.Direction.FORWARD);

        intakeSurgical = hardwareMap.get(DcMotorEx.class, "intakeSurgical");
        intakeSurgical.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        intakeSurgical.setDirection(DcMotor.Direction.FORWARD);
        intakeSurgical.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        carousel = hardwareMap.get(DcMotorEx.class,"caro");

        outtake = hardwareMap.get(DcMotorEx.class, "intake");
        outtake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        outtake.setDirection(DcMotor.Direction.REVERSE);
        outtake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        intakeExtension = hardwareMap.get(DcMotorEx.class, "intakeExtension");
        intakeExtension.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        intakeExtension.setDirection(DcMotor.Direction.REVERSE);
        intakeExtension.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //PIDCoefficients coeffs = new PIDCoefficients(kP, kI, kD);
        //PIDFController controller = new PIDFController(coeffs, 0, 0, 0, (x, v) -> kG);

        intakePosition = hardwareMap.get(Servo.class, "intakeLift");

        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        // init robot
        intakeExtensionLowerLimit = intakeExtension.getCurrentPosition();
        intakeExtensionUpperLimit = intakeExtensionLowerLimit + 270;
        intakePosition.setPosition(intakeUp);
        motorExLeft.setTargetPosition(outtakeDownPosition);
        outtakeServo.setPosition(outtakeServoLowerLimit);

        double scaler = 0.6;

        fl.setPosition(flMec);
        fr.setPosition(frMec);
        br.setPosition(brMec);
        bl.setPosition(blMec);

        boolean a = true;
        boolean b = true;
        boolean x = true;
        boolean y = true;
        boolean pressed = true;

        boolean toggleOff = false;

        capperServo.setPosition(capInit);

        waitForStart();

        while (!isStopRequested()) {
            double elapsed = runtime.seconds() - time;

            mecDrive.update();
            tankDrive.update();

            Pose2d poseEstimate;
            if (isMec) {
                poseEstimate = mecDrive.getPoseEstimate();
            } else {
                poseEstimate = tankDrive.getPoseEstimate();
            }
            PoseStorage.currentPose = poseEstimate;

            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.addData("scaler", scaler);
            telemetry.addData("intake servo position: ", intakePosition.getPosition());
            telemetry.addData("outtakePosition: ", outtake.getCurrentPosition());
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

            // slow rotation with bumpers
            if (gamepad1.left_bumper) {
                rotation = BUMPER_ROTATION_SPEED;
            } else if (gamepad1.right_bumper) {
                rotation = -BUMPER_ROTATION_SPEED;
            }

            if (isMec) {
                mecDrive.setWeightedDrivePower(new Pose2d(translation, rotation));
            } else {
                translation = new Vector2d(-gamepad1.left_stick_y*scaler, 0.0);
                tankDrive.setWeightedDrivePower(new Pose2d(translation, rotation));
            }

            // capper
            if (gamepad2.a) {
                // capper down
                capperServo.setPosition(capUp);
            } else if (gamepad2.y) {
                // capper up
                capperServo.setPosition(capDown);
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
                fleft.setPower(-0.07);
                fright.setPower(-0.07);
                bleft.setPower(-0.07);
                bright.setPower(-0.07);
            }

            if (gamepad2.x) {
                noCarousel = false;
                if (runtime.seconds() - lastX < 1.5) {
                    carousel.setPower(0.1);
                } else {
                    carousel.setPower(1);
                }
            } else {
                lastX = runtime.seconds();
            }

            if (gamepad2.b) {
                noCarousel = false;
                if (runtime.seconds() - lastB < 1.5) {
                    carousel.setPower(-1);
                } else {
                    carousel.setPower(-1);
                }
            } else {
                lastB = runtime.seconds();
            }

            if (noCarousel) carousel.setPower(0);

//            double state = 0;
//            if(gamepad2.x){
//                state=Math.min(1,state+1);
//            }
//            if(gamepad2.b){
//                state=Math.max(-1,state-1);
//            }
//
//            if (elapsed < 1.5) {
//                carousel.setPower(0.8 * state);
//            } else {
//                carousel.setPower(1 * state);
//            }
//            if (redCarousel.isDown()) {
//                carousel.setPower(-1);
//            } else {
//                carousel.setPower(0.0);
//            }
//
//            if (blueCarousel.isDown()) {
//                carousel.setPower(1);
//            } else {
//                carousel.setPower(0.0);
//            }

            telemetry.addData("red carousel down: ", redCarousel.isDown());

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
            if (gamepad2.left_stick_y < -0.02 || extended) {
                double joystickPosition = gamepad2.left_stick_y;
                telemetry.addData("curr pos: ", intakeExtension.getCurrentPosition());
                telemetry.addData("statement: ", intakeExtension.getCurrentPosition() > intakeExtensionLowerLimit);
                if (intakeExtension.getCurrentPosition() > -70) {
//                    telemetry.addData("statement: ", gamepad2.left_stick_y < -0.1);
//                    telemetry.addData("pos: ", (int) (joystickPosition * 270 * (-1)));
//
                    telemetry.addData("test", 0);
                    intakeExtension.setTargetPosition((int) (joystickPosition * 270 * (-1)));
                    intakeExtension.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    intakeExtension.setPower(0.5);
                }
                extended = true;
            }

//            else if (gamepad2.left_stick_y > 0.02 || !extended) { # what code makes left_stick_y > 0.02 go in then?
//                intakeExtension.setTargetPosition(0);
//                intakeExtension.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                intakeExtension.setPower(-0.4);
//                extended = false;
//            }

            if (intakeExtension.getCurrentPosition() >= -70 && intakeExtension.getCurrentPosition() <= 10) {
                if (gamepad2.left_stick_y > -0.02 && gamepad2.left_stick_y < 0.02) {
                    telemetry.addLine("pulling back");
                    intakeExtension.setTargetPosition(-60);
                    intakeExtension.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    intakeExtension.setPower(-0.001);
                }
            }

            telemetry.addData("joystick: ", gamepad2.left_stick_y);
            telemetry.addData("intake extension pos: ", intakeExtension.getCurrentPosition());
            telemetry.addData("extended: ", extended);

//            if(!gamepad2.dpad_left&&!gamepad2.dpad_up&&!gamepad2.dpad_down&&!gamepad2.dpad_right){
//                if (gamepad2.left_stick_y > 0.1) {
//                    if(lastvaluefront==-1){
//                        startpoint = (int) intakeExtensionUpperLimit;
//                    }else if(getRuntime()-lastvaluefront>0.1){
//                        startpoint=intakeExtension.getCurrentPosition();
//                    }
//                    lastvaluefront=getRuntime();
//                    int pos =  intakeExtension.getCurrentPosition();
//                    pos = startpoint-pos;
//                    pos = startpoint+(int)intakeExtensionLowerLimit-pos;
//                    double dist = Math.abs(startpoint-intakeExtensionLowerLimit);
//                    double function = Math.max((0.12 * (Math.sin((Math.PI / dist) * pos) * Math.exp(Math.PI * pos / dist))), 0.2);
//                    if(function>1.5){
//                        function=0.1;
//                    }
//                    // intake should come back up
//                    if (intakeExtension.getCurrentPosition() >= intakeExtensionLowerLimit) {
//                        power = 1/1.7 *(gamepad2.left_stick_y * (function));
//                        intakeExtension.setTargetPosition((int) intakeExtensionLowerLimit);
//                        intakeExtension.setPower(-(power+0.2));
//                    } else {
//                        intakeExtension.setPower(0.0);
//                    }
//                } else if (gamepad2.left_stick_y < -0.1) {
//
//                    if(lastvaluelast==-1){
//                        startpoint = (int) intakeExtensionLowerLimit;
//                    }else if(getRuntime()-lastvaluelast>0.1){
//                        startpoint=intakeExtension.getCurrentPosition();
//
//                    }
//                    lastvaluelast=getRuntime();
//                    int pos =intakeExtension.getCurrentPosition();
//                    pos = pos-startpoint;
//
//                    pos=startpoint+(int)intakeExtensionUpperLimit-pos;
//
//                    double dist = Math.abs(startpoint-intakeExtensionUpperLimit);
//                    double function = Math.max((0.12 * (Math.sin((Math.PI / dist) * pos) * Math.exp(Math.PI * pos / dist))), 0.2);
//                    if(function>1.01){
//                        function=0.1;
//                    }
//                    // intake extends
//
//                    if (intakeExtension.getCurrentPosition() <= intakeExtensionUpperLimit) {
//                        power = 1/1.7* (gamepad2.left_stick_y * (function));
//                        intakeExtension.setPower(-(power+0.1));
//                    } else {
//                        intakeExtension.setPower(0.0);
//                    }
//                } else {
//                    if(Math.abs(intakeExtension.getCurrentPosition()-intakeExtensionLowerLimit)<30.0){
//                        intakeExtension.setPower(-(0.2+0.1));
//                    }else{
//                        if(motorExLeft.getCurrentPosition()==outtakeDownPosition) {
//                            intakeExtension.setPower(-(0.2+0.1));
//                        }else{
//                            intakeExtension.setPower(0.0);
//                        }
//                    }
//
//                }
//            }else{
//
//
//            }

            // outtake

            if (gamepad2.dpad_up) {
//                if(timer ==-1){
//                    timer=getRuntime();
//                }
//                if(intakeExtension.getCurrentPosition()-intakeExtensionLowerLimit<200){
//                    intakeExtension.setPower(0.3);
//                }
//                if(getRuntime()-timer>outtakeDelay) {
                intakeExtension.setTargetPosition(40);
                intakeExtension.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                intakeExtension.setPower(0.5);

                sleep(500);

                motorExLeft.setTargetPosition(outtakeThirdLevelPosition);
                motorExLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motorExLeft.setPower(outtakePower);
//                }
//            }else{
//                timer = -1;
//            }
            }

            if (gamepad2.dpad_left) {
//                if(timer ==-1){
//                    timer=getRuntime();
//                }
//                if(intakeExtension.getCurrentPosition()-intakeExtensionLowerLimit<200){
//                    intakeExtension.setPower(0.3);
//                }
//                if(getRuntime()-timer>outtakeDelay) {


                motorExLeft.setTargetPosition(outtakeFirstLevelPosition);
                motorExLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motorExLeft.setPower(outtakePower);
//                }
//            }else{
//                timer = -1;
//            }
            }

//            // FIX THIS
//            if (gamepad2.y) {
//                if (outtake.getCurrentPosition() < outtakeThirdLevelPosition && outtake.getCurrentPosition() > -5) {
//                    telemetry.addLine("raising manually");
//                    outtake.setPower(0.2);
//                }
//            }
//
//            if (gamepad2.a) {
//                if (outtake.getCurrentPosition() < outtakeThirdLevelPosition && outtake.getCurrentPosition() > -5) {                    telemetry.addLine("raising manually");
//                    telemetry.addLine("lowering manually");
//                    outtake.setPower(-0.2);
//                }
//            }

            if (gamepad2.dpad_right) {
                if(intakeExtension.getCurrentPosition()-intakeExtensionLowerLimit<100){
                    intakeExtension.setPower(0.2);
                }

                outtakeServo.setPosition(outtakeServoLimitUpperMidHub);
            }

            if (gamepad2.dpad_down) {

                if(intakeExtension.getCurrentPosition()-intakeExtensionLowerLimit<100){
                    intakeExtension.setPower(0.7);
                }

                intakeExtension.setTargetPosition(40);
                intakeExtension.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                intakeExtension.setPower(0.7);

                sleep(500);

                motorExLeft.setTargetPosition(outtakeDownPosition);
                motorExLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motorExLeft.setPower(outtakePower*0.6);

                outtakeServo.setPosition(outtakeServoLowerLimit);
            }
            telemetry.addData("intake Position: ", intakePosition.getPosition());
            if(gamepad1.right_trigger>0.6&&pressed){
                if(isMec){
                    // switcing from mec to tank
                    if(intakePosition.getPosition() > mecDown - 0.01 && intakePosition.getPosition() < mecDown + 0.01){
                        intakePosition.setPosition(tankDown);
                    }
                    fr.setPosition(frTank);
                    br.setPosition(brTank);
                    fl.setPosition(flTank);
                    bl.setPosition(blTank);

                    isMec=false;

                }else{
                    // switching from tank to mec

                    fl.setPosition(flMec);
                    fr.setPosition(frMec);
                    br.setPosition(brMec);
                    bl.setPosition(blMec);

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
