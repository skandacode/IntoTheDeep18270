package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "Axon Servo Motion Profile Test", group = "Test")
public class ServoTesting extends OpMode {

    private ServoMotionProfile servoProfile;
    private Servo axonServo;
    private ElapsedTime programTimer;

    @Override
    public void init() {
        axonServo = hardwareMap.get(Servo.class, "e");
        servoProfile = new ServoMotionProfile(axonServo);
        programTimer = new ElapsedTime();

        // Configure motion parameters
        servoProfile.setMotionParameters(
                0.8,    // Max velocity: 0.8 positions/second
                1.5,    // Acceleration: 1.5 positions/second²
                1.5     // Deceleration: 1.5 positions/second²
        );
    }

    @Override
    public void loop() {
        double time = programTimer.seconds();

        // Automated sequence - move between positions
        if (time < 2.0) {
            servoProfile.setTargetPosition(0.0);  // Move to position 0
        } else if (time < 4.0) {
            servoProfile.setTargetPosition(1.0);  // Move to position 1
        } else if (time < 6.0) {
            servoProfile.setTargetPosition(0.5);  // Move to middle
        } else {
            // Reset timer for continuous operation
            programTimer.reset();
        }

        // Update the motion profile
        servoProfile.update();

        // Telemetry
        telemetry.addData("Program Time", "%.2f s", time);
        telemetry.addData("Current Position", "%.3f", servoProfile.getCurrentPosition());
        telemetry.addData("Target Position", "%.3f", servoProfile.getTargetPosition());
        telemetry.addData("Current Velocity", "%.3f", servoProfile.getCurrentVelocity());
        telemetry.addData("Is Moving", servoProfile.isMoving());
        telemetry.addData("Time Remaining", "%.2f s", servoProfile.getTimeRemaining());
        telemetry.update();
    }
}
