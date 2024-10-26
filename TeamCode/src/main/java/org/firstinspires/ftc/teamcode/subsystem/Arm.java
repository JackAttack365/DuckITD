package org.firstinspires.ftc.teamcode.subsystem;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Config;

public class Arm extends SubSystem {
    DcMotor arm, lift;

    public final double ARM_TICKS_PER_DEGREE = 19.7924893140647;

    public final double ARM_COLLAPSED_INTO_ROBOT  = 0;
    public final double ARM_COLLECT               = 250 * ARM_TICKS_PER_DEGREE;
    public final double ARM_CLEAR_BARRIER         = 230 * ARM_TICKS_PER_DEGREE;
    public final double ARM_SCORE_SPECIMEN        = 160 * ARM_TICKS_PER_DEGREE;
    public final double ARM_SCORE_SAMPLE_IN_LOW   = 160 * ARM_TICKS_PER_DEGREE;
    public final double ARM_SCORE_SAMPLE_IN_HIGH  = 200 * ARM_TICKS_PER_DEGREE;
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
    }

    @Override
    public void update() {

    }

    public void armToPos(double position) {
        arm.setTargetPosition((int) (position));

        ((DcMotorEx) arm).setVelocity(2100);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void liftToPos(double position) {
        lift.setTargetPosition((int) (position));

        ((DcMotorEx) lift).setVelocity(2100);
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
}
