package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.Constants.*;


import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config

public class Intake {

    DcMotorEx intakeExtension, intakeSurgical;
    Servo intakePosition;

    private int intakeExtensionLowerLimit, intakeExtensionUpperLimit;

    public Intake(HardwareMap hardwareMap) {

        intakeSurgical = hardwareMap.get(DcMotorEx.class, "intakeSurgical");
        intakeSurgical.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        intakeSurgical.setDirection(DcMotor.Direction.FORWARD);
        intakeSurgical.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        intakeExtension = hardwareMap.get(DcMotorEx.class, "intakeExtension");
        intakeExtension.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        intakeExtension.setDirection(DcMotor.Direction.REVERSE);
        intakeExtension.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        intakePosition = hardwareMap.get(Servo.class, "intakeLift");

    }

    public void init() {

        intakeExtension.setTargetPosition(-60);
        intakeExtension.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        intakeExtension.setPower(-0.5);

        intakeExtensionLowerLimit = intakeExtension.getCurrentPosition();
        intakeExtensionUpperLimit = intakeExtensionLowerLimit + 270;

        intakePosition.setPosition(intakeUp);
    }

    public int getIntakeExtensionLowerLimit() {
        return intakeExtensionLowerLimit;
    }

    public int getIntakeExtensionUpperLimit() {
        return intakeExtensionUpperLimit;
    }

    public void intakeUp() {
        intakePosition.setPosition(intakeUp);
    }

    public void intakeMecDown() {
        intakePosition.setPosition(intakeMecDown);
    }

    public void intakeTankDown() {
        intakePosition.setPosition(intakeTankDown);
    }

    public void intakeExtend(double joystickPosition) {

        if (intakeExtension.getCurrentPosition() >= intakeExtensionLowerLimit) {
            intakeExtension.setTargetPosition((int) (intakeExtensionLowerLimit + joystickPosition * 270 * (-1)));
            intakeExtension.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            intakeExtension.setPower(0.7);
        }

    }

    public void intakeRetract() {

        intakeExtension.setTargetPosition(intakeExtensionLowerLimit);
        intakeExtension.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        intakeExtension.setPower(-0.5);

    }

    public void intakeHoldIn() {

        intakeExtension.setTargetPosition(intakeExtensionLowerLimit);
        intakeExtension.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        intakeExtension.setPower(-0.001);
    }
}
