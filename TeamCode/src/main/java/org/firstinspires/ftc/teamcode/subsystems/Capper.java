package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import static org.firstinspires.ftc.teamcode.Constants.*;

@Config
public class Capper {
    private Servo capperServo;

    public Capper(HardwareMap hardwareMap) {
        capperServo = hardwareMap.get(Servo.class, "capperServo");
    }

    public void init() {
        capperServo.setPosition(capInit);
    }

    public void down(){
        if (capperServo.getPosition() + capStep < capDown) {
            capperServo.setPosition(capperServo.getPosition() + capStep);
        }
    }

    public void downSlow() {
        if (capperServo.getPosition() + capStep < capDown) {
            capperServo.setPosition(capperServo.getPosition() + capStepSlow);
        }
    }

    public void up() {
        if (capperServo.getPosition() - capStep > capInit) {
            capperServo.setPosition(capperServo.getPosition() - capStep);
        }
    }

    public void upSlow() {
        if (capperServo.getPosition() - capStep > capInit) {
            capperServo.setPosition(capperServo.getPosition() - capStepSlow);
        }
    }
}
