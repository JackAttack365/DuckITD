package org.firstinspires.ftc.teamcode.subsystem;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Config;

public class Arm extends SubSystem {
    public DcMotor arm;
    DcMotor lift;

    public final double ARM_TICKS_PER_DEGREE = 19.7924893140647;

    public final double ARM_COLLAPSED_INTO_ROBOT  = 0;
    public final double ARM_COLLECT               = 250 * ARM_TICKS_PER_DEGREE;
    public final double ARM_CLEAR_BARRIER         = 230 * ARM_TICKS_PER_DEGREE;
    public final double ARM_SCORE_SPECIMEN        = 160 * ARM_TICKS_PER_DEGREE;
    public final double ARM_SCORE_SAMPLE_IN_LOW   = 160 * ARM_TICKS_PER_DEGREE;
    public final double ARM_SCORE_SAMPLE_IN_HIGH  = 160 * ARM_TICKS_PER_DEGREE;
    public final double ARM_ATTACH_HANGING_HOOK   = 120 * ARM_TICKS_PER_DEGREE;
    public final double ARM_WINCH_ROBOT           = 15  * ARM_TICKS_PER_DEGREE;

    public Arm(Config config) {
        super(config);
    }

    public Arm(Config config, boolean isOneController) {
        super(config, isOneController);
    }

    @Override
    public void init() {
        arm = config.hardwareMap.get(DcMotor.class, "arm");
        lift = config.hardwareMap.get(DcMotor.class, "lift");

        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public void update() {
        if (config.gamePad1.right_stick_y >= 0.1) {
            lift.setPower(config.gamePad1.right_stick_y);
        } else if (config.gamePad1.right_stick_y <= -0.1) {
            lift.setPower(config.gamePad1.right_stick_y);
        } else {
            lift.setPower(0);
        }

        if (config.gamePad2.a) {
            armToPos(ARM_COLLECT);
        } else if (config.gamePad2.b) {
            armToPos(ARM_CLEAR_BARRIER);
        } else if (config.gamePad2.x) {
            armToPos(ARM_SCORE_SAMPLE_IN_HIGH);
        }

    }

    public void armToPos(double position) {
        arm.setTargetPosition((int) (position));

        arm.setPower(1);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void liftToPos(double position) {
        lift.setTargetPosition((int) (position));

        lift.setPower(1);
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
}
