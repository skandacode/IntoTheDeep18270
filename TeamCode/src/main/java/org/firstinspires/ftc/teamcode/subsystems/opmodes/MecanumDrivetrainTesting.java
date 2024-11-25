package org.firstinspires.ftc.teamcode.subsystems.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.sfdev.assembly.state.StateMachine;
import com.sfdev.assembly.state.StateMachineBuilder;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.TeleopSomewhatAuto;
import org.firstinspires.ftc.teamcode.pathing.WayPoint;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDrivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Outtake;

@Config
@TeleOp
public class MecanumDrivetrainTesting extends LinearOpMode {
    MecanumDrivetrain drive;
    Intake intake;
    Outtake outtake;

    @Override
    public void runOpMode() throws InterruptedException {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry= new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        drive=new MecanumDrivetrain(hardwareMap, telemetry, dashboard);
        intake= new Intake(hardwareMap);
        outtake= new Outtake(hardwareMap, telemetry);

        StateMachine sampleMachine = new StateMachineBuilder()
                .state(TeleopSomewhatAuto.SampleStates.IDLE)
                .onEnter(() -> {
                    intake.retract();
                    intake.setPower(0);
                })
                .transition(() -> gamepad1.y)
                .state(TeleopSomewhatAuto.SampleStates.EXTEND)
                .onEnter(()->intake.setExtended(true))
                .transitionTimed(0.1)
                .state(TeleopSomewhatAuto.SampleStates.DROP)
                .onEnter(() -> {
                    intake.intakePosition();
                    intake.setPower(0.8);
                })
                .transition(()->intake.getDistance()<4)
                .state(TeleopSomewhatAuto.SampleStates.RETRACT)
                .onEnter(() -> {
                    intake.retract();
                    intake.setCover(true);
                    intake.setPower(0.7);
                    outtake.transferPos();
                })
                .transitionTimed(0.2)
                .state(TeleopSomewhatAuto.SampleStates.OPENCOVER)
                .onEnter(() -> intake.setCover(false))
                .transition(() -> intake.isDone())

                .state(TeleopSomewhatAuto.SampleStates.WAIT)
                .onEnter(() -> intake.setPower(0.4))
                .transitionTimed(0.7)

                .state(TeleopSomewhatAuto.SampleStates.CLOSE)
                .onEnter(() -> outtake.closeClaw())
                .transitionTimed(0.2)

                .state(TeleopSomewhatAuto.SampleStates.LIFT)
                .onEnter(() -> outtake.setTargetPos(3000))
                .transitionTimed(0.2)

                .state(TeleopSomewhatAuto.SampleStates.WRIST).onEnter(() -> {
                    outtake.scorePos();
                    intake.setPower(0);
                    intake.setPower(-0.2);
                })
                .transition(() -> gamepad1.left_bumper)

                .state(TeleopSomewhatAuto.SampleStates.OPEN).onEnter(() -> outtake.openClaw()).transitionTimed(1.5)

                .state(TeleopSomewhatAuto.SampleStates.LOWERLIFT).onEnter(() -> {
                    outtake.setTargetPos(0);
                    outtake.transferPos();
                }).transition(() -> (outtake.atTarget() || gamepad1.y), TeleopSomewhatAuto.SampleStates.IDLE).build();


        WayPoint bucketPos1=new WayPoint(new Pose2D(DistanceUnit.INCH, -48, -56, AngleUnit.DEGREES, 43),
                new Pose2D(DistanceUnit.INCH, 1, 1, AngleUnit.DEGREES, 2));
        WayPoint bucketPos2=new WayPoint(new Pose2D(DistanceUnit.INCH, -50, -58, AngleUnit.DEGREES, 43),
                new Pose2D(DistanceUnit.INCH, 1, 1, AngleUnit.DEGREES, 2));
        WayPoint bucketPos3=new WayPoint(new Pose2D(DistanceUnit.INCH, -52, -59, AngleUnit.DEGREES, 43),
                new Pose2D(DistanceUnit.INCH, 1, 1, AngleUnit.DEGREES, 2));

        WayPoint presample1=new WayPoint(new Pose2D(DistanceUnit.INCH, -45, -56, AngleUnit.DEGREES, 90),
                new Pose2D(DistanceUnit.INCH, 0.5, 0.5, AngleUnit.DEGREES, 1));
        WayPoint sample1=new WayPoint(new Pose2D(DistanceUnit.INCH, -45, -51, AngleUnit.DEGREES, 89),
                new Pose2D(DistanceUnit.INCH, 0.5, 0.5, AngleUnit.DEGREES, 1));
        WayPoint presample2=new WayPoint(new Pose2D(DistanceUnit.INCH, -53, -56, AngleUnit.DEGREES, 90),
                new Pose2D(DistanceUnit.INCH, 0.5, 0.5, AngleUnit.DEGREES, 1));
        WayPoint sample2=new WayPoint(new Pose2D(DistanceUnit.INCH, -53, -51, AngleUnit.DEGREES, 90),
                new Pose2D(DistanceUnit.INCH, 0.5, 0.5, AngleUnit.DEGREES, 1));
        WayPoint presample3=new WayPoint(new Pose2D(DistanceUnit.INCH, -49, -52.3, AngleUnit.DEGREES, 126),
                new Pose2D(DistanceUnit.INCH, 0.5, 0.5, AngleUnit.DEGREES, 1));
        WayPoint sample3=new WayPoint(new Pose2D(DistanceUnit.INCH, -52, -48, AngleUnit.DEGREES, 126),
                new Pose2D(DistanceUnit.INCH, 0.5, 0.5, AngleUnit.DEGREES, 1));


        WayPoint startPoint=new WayPoint(new Pose2D(DistanceUnit.INCH, -36, -63, AngleUnit.DEGREES, 90),
                new Pose2D(DistanceUnit.INCH, 0.5, 0.5, AngleUnit.DEGREES, 0.5));

        waitForStart();
        drive.setTarget(startPoint);
        drive.setPosition(startPoint.getPosition());
        drive.calibrateIMU();
        sampleMachine.start();
        sampleMachine.setState(TeleopSomewhatAuto.SampleStates.RETRACT);
        while (opModeIsActive()){
            drive.update();
            drive.updatePIDS();
            if (gamepad1.a){
                drive.setTarget(bucketPos1);
            }
            if (gamepad1.touchpad){
                drive.setTarget(bucketPos2);
            }
            if (gamepad1.options){
                drive.setTarget(bucketPos3);
            }

            if (gamepad1.b){
                drive.setTarget(startPoint);
            }
            if (gamepad1.x){
                drive.setTarget(sample1);
            }
            if (gamepad1.dpad_up){
                drive.setTarget(sample2);
            }
            if (gamepad1.dpad_down){
                drive.setTarget(sample3);
            }
            if (gamepad1.right_bumper){
                drive.setTarget(presample1);
            }
            if (gamepad1.dpad_left){
                drive.setTarget(presample2);
            }
            if (gamepad1.dpad_right){
                drive.setTarget(presample3);
            }

            sampleMachine.update();
            intake.update();
            outtake.update();
            telemetry.update();
        }
    }
}
