package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class intakeTest extends LinearOpMode {
    IntakeCopy intake;
    @Override
    public void runOpMode() throws InterruptedException {
        intake = new IntakeCopy(hardwareMap, telemetry);

        waitForStart();
        while (opModeIsActive()){
            telemetry.addData("Intake Position", intake.getIntakePos());
            telemetry.update();
        }

    }
}
