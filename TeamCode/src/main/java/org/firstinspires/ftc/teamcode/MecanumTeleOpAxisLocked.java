package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class MecanumTeleOpAxisLocked {

    private final DcMotor frontLeftMotor;
    private final DcMotor backLeftMotor;
    private final DcMotor frontRightMotor;
    private final DcMotor backRightMotor;

    private final Telemetry telemetry;

    public MecanumTeleOpAxisLocked(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        frontLeftMotor = hardwareMap.get(DcMotor.class, "frontleft");
        backLeftMotor = hardwareMap.get(DcMotor.class, "backleft");
        frontRightMotor = hardwareMap.get(DcMotor.class, "frontright");
        backRightMotor = hardwareMap.get(DcMotor.class, "backright");

        // Reverse right side (adjust if reversed for your setup)
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void drive(double y, double x, double rx, boolean dpadUp, boolean dpadDown, boolean dpadLeft, boolean dpadRight) {
        // Remember, Y stick value is reversed
        y = -y;
        x = -x * 1.1; // Counteract imperfect strafing
        rx = -rx;

        // --- D-pad overrides for precise movement ---
        if (dpadUp) y = -0.5;
        if (dpadDown) y = 0.5;
        if (dpadRight) x = -0.5;
        if (dpadLeft) x = 0.5;

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = ((y + x + rx) / denominator) * 0.5;
        double backLeftPower = ((y - x + rx) / denominator) * 0.5;
        double frontRightPower = ((y - x - rx) / denominator) * 0.5;
        double backRightPower = ((y + x - rx) / denominator) * 0.5;

        frontLeftMotor.setPower(frontLeftPower);
        backLeftMotor.setPower(backLeftPower);
        frontRightMotor.setPower(frontRightPower);
        backRightMotor.setPower(backRightPower);

        telemetry.addData("FL", frontLeftPower);
        telemetry.addData("FR", frontRightPower);
        telemetry.addData("BL", backLeftPower);
        telemetry.addData("BR", backRightPower);
    }

    public void stop() {
        frontLeftMotor.setPower(0);
        backLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backRightMotor.setPower(0);
    }
}
