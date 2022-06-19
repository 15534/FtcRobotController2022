package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import static org.firstinspires.ftc.teamcode.Constants.*;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.SampleTankDrive;

@Config
public class DriveTrain {

    private DcMotorEx fleft, fright, bleft, bright;
    private Servo fr, br, fl, bl;

    private SampleMecanumDrive mecDrive;
    private SampleTankDrive tankDrive;

    public DriveTrain(HardwareMap hardwareMap) {
        // drive train servo init
        fl = hardwareMap.get(Servo.class, "frontleft");
        fr = hardwareMap.get(Servo.class, "frontright");
        br = hardwareMap.get(Servo.class, "backright");
        bl = hardwareMap.get(Servo.class, "backleft");

        // drivetrain motor init
        fleft = hardwareMap.get(DcMotorEx.class, "front_left");
        fright = hardwareMap.get(DcMotorEx.class, "front_right");
        bleft = hardwareMap.get(DcMotorEx.class, "rear_left");
        bright = hardwareMap.get(DcMotorEx.class, "rear_right");

        // roadrunner drive trains
        mecDrive = new SampleMecanumDrive(hardwareMap);
        tankDrive = new SampleTankDrive(hardwareMap);

        mecDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        tankDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public SampleMecanumDrive getMecDrive() {
        return mecDrive;
    }

    public SampleTankDrive getTankDrive() {
        return tankDrive;
    }

    public void mecToTank() {
        fr.setPosition(frTank);
        br.setPosition(brTank);
        fl.setPosition(flTank);
        bl.setPosition(blTank);
    }

    public void tankToMec() {
        fl.setPosition(flMec);
        fr.setPosition(frMec);
        br.setPosition(brMec);
        bl.setPosition(blMec);
    }

    public void duckTension() {
        fleft.setPower(duckTension);
        fright.setPower(duckTension);
        bleft.setPower(duckTension);
        bright.setPower(duckTension);
    }

    public void init () {
        tankToMec();
    }
}
