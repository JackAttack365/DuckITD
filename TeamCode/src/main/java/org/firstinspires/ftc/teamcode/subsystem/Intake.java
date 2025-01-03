package org.firstinspires.ftc.teamcode.subsystem;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Config;

public class Intake extends SubSystem {
    public CRServo intake;

    public Intake(Config config) {
        super(config);
    }

    public Intake(Config config, boolean isOneController) {
        super(config, isOneController);
    }

    @Override
    public void init() {
        intake = config.hardwareMap.get(CRServo.class, "intake");
    }

    @Override
    public void update() {
        if (config.gamePad1.left_trigger >= 0.1) {
            intakeBack();
        } else if (config.gamePad1.right_trigger >= 0.1) {
            intakeOn();
        } else {
            intakeOff();
        }
    }

    public void intakeOn() {
        intake.setPower(-1);
    }

    public void intakeOff() {
        intake.setPower(0);
    }

    public void intakeBack() {
        intake.setPower(1);
    }
}
