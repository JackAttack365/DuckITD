package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.subsystem.Arm;
import org.firstinspires.ftc.teamcode.subsystem.Claw;
import org.firstinspires.ftc.teamcode.subsystem.Drivetrain;

public class LeftAuto extends LinearOpMode {
    Config     config = null;

    Drivetrain drive = null;
    Arm        arm = null;
    Claw       claw = null;

    @Override
    public void runOpMode() throws InterruptedException {
        config = new Config(telemetry, hardwareMap, gamepad1, gamepad2);

        drive = new Drivetrain(config);
        arm   = new Arm(config);
        claw  = new Claw(config);

        drive.init();
        arm.init();
        claw.init();

        waitForStart();

        arm.armToPos(arm.ARM_SCORE_SAMPLE_IN_HIGH);
        arm.liftToPos(500);
        drive.forwards(500);
        claw.intakeBack();
    }
}
