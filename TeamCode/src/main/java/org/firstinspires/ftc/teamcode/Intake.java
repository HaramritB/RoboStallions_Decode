package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;


public class Intake {
    
    private DcMotor intakeMotor;
    private Servo intakeServo;
    private ColorSensor colorSensor;
    
    // I think we might need to change these values later but based on my reearch this is what I thought was adequate
    private double intakePower = 0.85;
    private double ejectPower = -0.9;
    private double servoDown = 0.42;
    private double servoUp = 0.1;
    
    private boolean isRunning = false;
    
    public Intake(HardwareMap hardwareMap) {
        intakeMotor = hardwareMap.get(DcMotor.class, "intake");
        intakeServo = hardwareMap.get(Servo.class, "kicker");
        
        try {
            colorSensor = hardwareMap.get(ColorSensor.class, "intake_color");
        } catch (Exception e) {
            colorSensor = null;
        }
        
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeServo.setPosition(servoUp);
    }
    
    public void startIntake() {
        intakeMotor.setPower(intakePower);
        intakeServo.setPosition(servoDown);
        isRunning = true;
    }
    
    public void startIntake(double power) {
        intakeMotor.setPower(power);
        intakeServo.setPosition(servoDown);
        isRunning = true;
    }
    
    public void eject() {
        intakeMotor.setPower(ejectPower);
        isRunning = false;
    }
    
    public void eject(double power) {
        intakeMotor.setPower(-Math.abs(power));
        isRunning = false;
    }
    
    public void stop() {
        intakeMotor.setPower(0);
        isRunning = false;
    }
    
    public void retract() {
        intakeServo.setPosition(servoUp);
        stop();
    }

    public void deploy() {
        intakeServo.setPosition(servoDown);
    }
    
    public void setServoPower(double position) {
        intakeServo.setPosition(position);
    }
    
    public void setPower(double power) {
        intakeMotor.setPower(power);
    }
    
    public boolean hasArtifact() {
        if (colorSensor == null) return false;
        int r = colorSensor.red();
        int g = colorSensor.green();
        int b = colorSensor.blue();
        return r > 80 || g > 80 || b > 80;
    }
    
    public boolean isRunning() {
        return isRunning;
    }
    
    public double getMotorPower() {
        return intakeMotor.getPower();
    }
    
    public double getServoPosition() {
        return intakeServo.getPosition();
    }
    
    public int[] getRGB() {
        if (colorSensor == null) return new int[]{0, 0, 0};
        return new int[]{
            colorSensor.red(),
            colorSensor.green(),
            colorSensor.blue()
        };
    }
    
    public void setIntakePower(double power) {
        this.intakePower = power;
    }
    
    public void setEjectPower(double power) {
        this.ejectPower = -Math.abs(power);
    }
    
    public void setServoPositions(double down, double up) {
        this.servoDown = down;
        this.servoUp = up;
    }
}


/* Usage ex:

IntakeSubsystem intake = new IntakeSubsystem(hardwareMap);

intake.startIntake();
intake.eject();
intake.stop();
intake.retract();
intake.deploy();

if (intake.hasArtifact()) {
    // do smthn
}
*/