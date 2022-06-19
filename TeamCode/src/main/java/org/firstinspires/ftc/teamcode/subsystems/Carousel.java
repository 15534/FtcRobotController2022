package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import static org.firstinspires.ftc.teamcode.Constants.*;

public class Carousel {

    DcMotorEx CarouselSpinner;

    public Carousel(HardwareMap hardwareMap) {
        CarouselSpinner = hardwareMap.get(DcMotorEx.class, "caro");
    }

    public void SpinX(ElapsedTime runtime, double lastX_press) {
        if (runtime.seconds() - lastX_press < 1.25) {
            CarouselSpinner.setPower(1);
        } else {
            CarouselSpinner.setPower(0.5);
        }
    }

    public void SpinB(ElapsedTime runtime, double lastB_press) {
        if (runtime.seconds() - lastB_press < 1.25) {
            CarouselSpinner.setPower(-0.5);
        } else {
            CarouselSpinner.setPower(-1);
        }
    }
}