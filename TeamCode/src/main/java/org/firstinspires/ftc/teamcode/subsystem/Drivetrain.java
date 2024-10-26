package org.firstinspires.ftc.teamcode.subsystem;

import com.qualcomm.robotcore.hardware.DcMotor;
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

        motors[0]=(left);
        motors[1]=(right);
    }

    @Override
    public void update() {

    }

    public void forwards(int distance) throws InterruptedException {
        // To convert cm into motor position counter values
        final double DISTANCE_CONSTANT=2;
        // What power to use to drive the robot
        final double DRIVE_POWER=0.8;
        // What power to use to drive the robot
        final double MIN_POWER=0.1;
        // How long to pause before checking movement
        final int SLEEP_INTERVAL=10;
        // Acceleration distance (in encoder clicks). 300mm in this case:
        final double ACCEL_DIST=300.0*DISTANCE_CONSTANT;


        int targetPosition=(int)DISTANCE_CONSTANT*distance;

        for(DcMotor motor : motors) {
            // Stop and reset the motor counter
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            // Set the target position by converting the distance into motor
            // position values
            motor.setTargetPosition(targetPosition);
            // Set the motor into the mode that uses the encoder to keep
            // track of the position
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        }

        config.telemetry.addData("motor1",left.getCurrentPosition());
        config.telemetry.update();

        // Sleep a bit to make sure the motor report as "busy"
        wait(SLEEP_INTERVAL);
        // Loop as long as either motor reports as busy
        boolean isBusy=false;
        do {

            int currentPosition=left.getCurrentPosition();
            config.telemetry.addData("motor1", currentPosition);

            // Determine the closest distance to either starting position
            // or target. When close to start, we accelerate, when close to
            // target, we decelerate. When we are far from both, the robot
            // drives at DRIVE_POWER speed. To avoid not moving at all, the
            // minimum speed is set to MIN_POWER. The distance over which to
            // accerate or decelerate is ACCEL_DIST. All math is done in
            // encoder "clicks", 300 mm is about 600 encoder clicks.
            int lengthToTarget=Math.abs(targetPosition-currentPosition);
            if (lengthToTarget>Math.abs(currentPosition)) {
                lengthToTarget=Math.abs(currentPosition);
            }

            double power=(DRIVE_POWER-MIN_POWER)*(lengthToTarget/ACCEL_DIST)+MIN_POWER;
            if(lengthToTarget>=ACCEL_DIST) {
                power=DRIVE_POWER;
            }

            for(DcMotor motor : motors) {
                motor.setPower(power);
            }

            // Sleep until next check
            wait(SLEEP_INTERVAL);
            isBusy=false;
            for(DcMotor motor : motors) {
                if(motor.isBusy())isBusy=true;
            }
        } while(isBusy);

        for(DcMotor motor : motors) {
            motor.setPower(0);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }
    public void turn(int angle) throws InterruptedException {
        imu.resetYaw();
        wait(50);
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
