package org.firstinspires.ftc.teamcode.subsystem;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Config;

public class Claw extends SubSystem {
    CRServo intake;
    Servo wrist;

    public Claw(Config config) {
        super(config);
    }

    public Claw(Config config, boolean isOneController) {
        super(config, isOneController);
    }

    @Override
    public void init() {
        intake = config.hardwareMap.get(CRServo.class, "intake");
        wrist = config.hardwareMap.get(Servo.class, "claw");
    }

    @Override
    public void update() {

    }

    public void intakeOn() {
        intake.setPower(1);
    }

    public void intakeOff() {
        intake.setPower(0);
    }

    public void intakeBack() {
        intake.setPower(-0.5);
    }

    public void wristIn() {
        wrist.setPosition(0.888883);
    }
    public void wristOut() {
        wrist.setPosition(0.5);
    }
}
