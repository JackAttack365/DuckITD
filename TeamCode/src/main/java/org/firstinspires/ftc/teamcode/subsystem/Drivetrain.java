package org.firstinspires.ftc.teamcode.subsystem;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.Config;

public class Drivetrain extends SubSystem {
    DcMotor left, right;

    IMU imu;

    public Drivetrain(Config config) {
        super(config);
    }

    public Drivetrain(Config config, boolean isOneController) {
        super(config, isOneController);
    }

    @Override
    public void init() {
        left = config.hardwareMap.get(DcMotor.class, "left");
        right = config.hardwareMap.get(DcMotor.class, "right");
        imu = config.hardwareMap.get(IMU.class, "imu");

        left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        left.setDirection(DcMotorSimple.Direction.REVERSE);
        right.setDirection(DcMotorSimple.Direction.REVERSE);

        left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public void update() {
        double lateral = config.gamePad1.left_stick_y;
        double yaw = config.gamePad1.left_stick_x;
        double speed = 1 - (config.gamePad1.right_trigger + config.gamePad1.left_trigger) / 2;

        double leftPower = (-yaw + lateral) * speed;
        double rightPower = (yaw + lateral) * speed;

        double max;
        max = Math.max(Math.abs(leftPower), Math.abs(rightPower));

        if (max > 1.0) {
            leftPower /= max;
            rightPower /= max;
        }

        left.setPower(leftPower);
        right.setPower(rightPower);

        config.telemetry.addData("left", leftPower);
        config.telemetry.addData("right", rightPower);
    }

    public void fwd() {
        left.setPower(-0.8);
        right.setPower(-0.8);
    }

    public void stop() {
        left.setPower(0);
        right.setPower(0);
    }
}