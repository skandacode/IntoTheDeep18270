package org.firstinspires.ftc.teamcode.subsystems.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@TeleOp
public class BatteryBeakSim extends LinearOpMode {
    VoltageSensor batteryVoltageSensor;
    DcMotorEx intakeMotor;
    @Override
    public void runOpMode() throws InterruptedException {
        intakeMotor= (DcMotorEx) hardwareMap.dcMotor.get("extendoMotor");
        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();
        double prevVoltage=batteryVoltageSensor.getVoltage();
        double prevCurrent=intakeMotor.getCurrent(CurrentUnit.AMPS);
        telemetry.addData("Current battery voltage", prevVoltage);
        telemetry.addData("Current motor current", prevVoltage);
        telemetry.update();
        waitForStart();
        while (opModeIsActive()){
            intakeMotor.setPower(-gamepad1.left_stick_y);
            double currVoltage=batteryVoltageSensor.getVoltage();
            double currCurrent=intakeMotor.getCurrent(CurrentUnit.AMPS);
            telemetry.addData("Current battery voltage", currVoltage);
            telemetry.addData("Current motor current", currCurrent);
            telemetry.addData("Battery Resistance", (prevVoltage-currVoltage)/(currCurrent-prevCurrent));
            telemetry.update();
        }
    }
}
