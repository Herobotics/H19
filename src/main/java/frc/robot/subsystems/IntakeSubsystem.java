package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {
    private final TalonFX extendoMotor = new TalonFX(Constants.IntakeConstants.INTAKE_MOVER_MOTOR_ID);
    private final VictorSPX spinningmotor = new VictorSPX(Constants.IntakeConstants.INTAKE_ROLLER_MOTOR_ID);
    // private static final Angle maxAngle = Angle(0.5);

    public IntakeSubsystem() {
        SmartDashboard.putNumber("Extendo Motor Position", 0);
        SmartDashboard.putNumber("Extendo Motor Voltage", 0);
        SmartDashboard.putNumber("Spinning Motor Voltage", 0);
    }

    // 
    public Command MoveOut() {
        return this.run(() -> {
            // spinningmotor.set(VictorSPXControlMode.PercentOutput, Constants.IntakeConstants.ROLLER_MOTOR_INTAKE_PERCENT);
            //extendoMotor.setPosition(Constants.IntakeConstants.MOVER_MOTOR_EXTENSION_POSITION);
            extendoMotor.setVoltage(Constants.IntakeConstants.MOVER_MOTOR_EXTENSION_VOLTAGE);
        });
    }

    public Command MoveIn() {
        return this.run(() -> {
            //spinningmotor.set(VictorSPXControlMode.PercentOutput, 0);
            //extendoMotor.setPosition(Constants.IntakeConstants.MOVER_MOTOR_RETRACT_POSITION);
            extendoMotor.setVoltage(Constants.IntakeConstants.MOVER_MOTOR_RETRACT_VOLTAGE);
        });
    }
    
    public Command FEEEED() {
        return this.run(() -> {
            spinningmotor.set(VictorSPXControlMode.PercentOutput, Constants.IntakeConstants.ROLLER_MOTOR_INTAKE_PERCENT);
            extendoMotor.setVoltage(0);
            //extendoMotor.setPosition(Constants.IntakeConstants.MOVER_MOTOR_EXTENSION_VOLTAGE);
        });
    }
    public Command Feedout() {
        return this.run(() -> {
            spinningmotor.set(VictorSPXControlMode.PercentOutput, -1.0 * Constants.IntakeConstants.ROLLER_MOTOR_INTAKE_PERCENT);
            extendoMotor.setVoltage(0);
            //extendoMotor.setPosition(Constants.IntakeConstants.MOVER_MOTOR_EXTENSION_VOLTAGE);
        });
    }

    // Default command
    public Command StopEveryMotor() {
        return this.runOnce(() -> {
            //extendoMotor.setVoltage(0);

            // Hold position
            //extendoMotor.setPosition(extendoMotor.getPosition().getValueAsDouble());
            spinningmotor.set(VictorSPXControlMode.PercentOutput, 0);
        });
    }

    public double HowFarOutIsTheIntake() {
        return extendoMotor.getPosition().getValueAsDouble();
    }

    @Override
    // This method will be called once per scheduler run
    public void periodic() {
        // Log stats
        SmartDashboard.putNumber("Extendo Motor Position", extendoMotor.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("Extendo Motor Voltage", extendoMotor.getMotorVoltage().getValueAsDouble());
        SmartDashboard.putNumber("Spinning Motor Voltage", spinningmotor.getMotorOutputVoltage());
    }
    
}
