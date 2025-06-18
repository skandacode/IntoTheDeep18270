import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.hardware.CachedMotorEx;

@Config
@TeleOp
public class testing extends LinearOpMode {
    CachedMotorEx frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor;
    public static double frontLeftPower = 0;
    public static double backLeftPower = 0;
    public static double frontRightPower = 0;
    public static double backRightPower = 0;
    public static double claw_pos = 0.7;
    public static double turret_pos = 0.7;
    Limelight3A limelight;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry=new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        Servo claw = hardwareMap.servo.get("claw");
        Servo turret = hardwareMap.servo.get("turret");
        frontLeftMotor=new CachedMotorEx(hardwareMap, "frontleft");
        frontRightMotor=new CachedMotorEx(hardwareMap, "frontright");
        backLeftMotor=new CachedMotorEx(hardwareMap, "backleft");
        backRightMotor=new CachedMotorEx(hardwareMap, "backright");
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        limelight.start();
        limelight.pipelineSwitch(0);

        waitForStart();
        while (opModeIsActive()){

            frontLeftMotor.setPower(frontLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backLeftMotor.setPower(backLeftPower);
            backRightMotor.setPower(backRightPower);
            claw.setPosition(claw_pos);
            turret.setPosition(turret_pos);
            LLResult result = limelight.getLatestResult();
            if (result != null && result.isValid()) {
                double tx = result.getTx(); // How far left or right the target is (degrees)
                double ty = result.getTy(); // How far up or down the target is (degrees)
                double ta = result.getTa(); // How big the target looks (0%-100% of the image)

                telemetry.addData("Target X", tx);
                telemetry.addData("Target Y", ty);
                telemetry.addData("Target Area", ta);
            } else {
                telemetry.addData("Limelight", "No Targets");
            }



            telemetry.update();
        }
    }
}
