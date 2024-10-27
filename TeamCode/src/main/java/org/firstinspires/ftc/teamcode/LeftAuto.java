package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.subsystem.Arm;
import org.firstinspires.ftc.teamcode.subsystem.Intake;
import org.firstinspires.ftc.teamcode.subsystem.Drivetrain;

@Autonomous(name="LeftAuto")
public class LeftAuto extends LinearOpMode {
    Config     config = null;

    Drivetrain drive = null;
    Arm        arm = null;
    Intake     intake = null;

    @Override
    public void runOpMode() throws InterruptedException {
        config = new Config(telemetry, hardwareMap, gamepad1, gamepad2);

        drive = new Drivetrain(config);
        arm   = new Arm(config);
        intake  = new Intake(config);

        drive.init();
        arm.init();
        intake.init();

        waitForStart();

        arm.armToPos(arm.ARM_SCORE_SAMPLE_IN_HIGH);
        sleep(1000);
        arm.liftToPos(1700);
        sleep(1000);
        drive.fwd();
        sleep(450);
        drive.stop();
        arm.arm.setPower(0);
        sleep(1000);
        intake.intake.setPower(1);
        sleep(5000);
        intake.intakeOff();
    }
}
