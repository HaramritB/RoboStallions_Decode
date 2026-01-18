package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Transfer {

    private final DcMotorEx transfer;

    public Transfer(HardwareMap hw) {
        transfer = hw.get(DcMotorEx.class, "transfer");
        transfer.setDirection(DcMotorSimple.Direction.FORWARD);
        transfer.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        stop();
    }

    public void update(double triggerValue) {
        if (triggerValue > 0.1) {
            transfer.setPower(.95);   // full speed while held nigge
        } else {
            transfer.setPower(0.25);
        }
    }

    public void autoTransfer() {
        transfer.setPower(1.0);
    }

    public void stop() {
        transfer.setPower(0.0);
    }

    public double getPower() {
        return transfer.getPower();
    }
}
