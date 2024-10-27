package org.firstinspires.ftc.teamcode.subsystem;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.Config;

public class Drivetrain extends SubSystem {
    DcMotor left,right;
    DcMotor motors[]  = new DcMotor[2];

    IMU imu;

    public Drivetrain(Config config) {
        super(config);
    }

    public Drivetrain(Config config, boolean isOneController) {
        super(config, isOneController);
    }

    @Override
    public void init() {
        left = config.hardwareMap.get(DcMotor.class,"left");
        right = config.hardwareMap.get(DcMotor.class,"right");
        imu = config.hardwareMap.get(IMU.class, "imu");

        left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        left.setDirection(DcMotorSimple.Direction.REVERSE);
        right.setDirection(DcMotorSimple.Direction.REVERSE);

        left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motors[0]=(left);
        motors[1]=(right);
    }

    @Override
    public void update() {

    }

    public void fwd() {
        left.setPower(0.8);
        right.setPower(0.8);
    }

    public void stop() {
        left.setPower(0);
        right.setPower(0);
    }

    public void forwards(int distance) throws InterruptedException {
        // To convert cm into motor position counter values
        final double DISTANCE_CONSTANT=2;
        // What power to use to drive the robot
        final double DRIVE_POWER=0.6;
        // How long to pause before checking movement
        final int SLEEP_INTERVAL=10;


        int targetPosition=(int)DISTANCE_CONSTANT*distance;


        left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // Set the target position by converting the distance into motor
        // position values
        left.setTargetPosition(targetPosition);
        right.setTargetPosition(targetPosition);
        // Set the motor into the mode that uses the encoder to keep
        // track of the position
        left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        right.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        config.telemetry.addData("motor1",left.getCurrentPosition());
        config.telemetry.update();

        Thread.sleep(SLEEP_INTERVAL);

        // Loop as long as either motor reports as busy
        boolean isBusy=false;
        do {

            int currentPosition=left.getCurrentPosition();
            config.telemetry.addData("motor1", currentPosition);

            for(DcMotor motor : motors) {
                motor.setPower(DRIVE_POWER);
            }
            Thread.sleep(SLEEP_INTERVAL);
            isBusy=false;
            if(left.isBusy())isBusy=true;
            if(right.isBusy())isBusy=true;

        } while(isBusy);

        left.setPower(0);
        right.setPower(0);
        left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    public void turn(int angle) throws InterruptedException {
        imu.resetYaw();
        double currentAngle=getAngle();
        int direction=0;
        double targetAngle=currentAngle+angle;
        double remainingAngle=Math.abs(targetAngle-currentAngle);

        if(angle<0) {
            direction=-1;
        } else {
            direction=1;
        }

        while(remainingAngle>0.5) {

            double power=getTurnPower(remainingAngle);
            left.setPower(-1*direction*power);
            right.setPower(1*direction*power);
            currentAngle=getAngle();

            // Adjust angle for angles greater than 180 or less than -180
            if(direction>0 && currentAngle<-10) {
                currentAngle=360+currentAngle;
            } else if(direction<0 && currentAngle>10) {
                currentAngle=-360+currentAngle;
            }

            remainingAngle=Math.abs(targetAngle-currentAngle);
        }

        for(DcMotor motor : motors) {
            motor.setPower(0);
        }
    }

    public double getTurnPower(double remainingAngle) {
        return((remainingAngle+10)/100);
    }

    public double getAngle() {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        return orientation.getYaw(AngleUnit.DEGREES);
    }
}
