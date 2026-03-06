package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {
    private final TalonFX extendoMotor = new TalonFX(5);
    private final VictorSPX spinningmotor = new VictorSPX(2);
    // private static final Angle maxAngle = Angle(0.5);

    public IntakeSubsystem() {
        
    }

    // Aiming the shooter
    public Command IntakeOut() {
        return this.runOnce(() -> {
            spinningmotor.set(VictorSPXControlMode.PercentOutput, Constants.IntakeConstants.ROLLER_MOTOR_INTAKE_SPEED);
            extendoMotor.setPosition(Constants.IntakeConstants.MOVER_MOTOR_EXTENDED_ANGLE);
        });
    }
    
    public Command IntakeReversed() {
        return this.runOnce(() -> {
            spinningmotor.set(VictorSPXControlMode.PercentOutput, -1.0 * Constants.IntakeConstants.ROLLER_MOTOR_INTAKE_SPEED);
            extendoMotor.setPosition(Constants.IntakeConstants.MOVER_MOTOR_EXTENDED_ANGLE);
        });
    }

    public Command IntakeIn() {
        return this.runOnce(() -> {
            spinningmotor.set(VictorSPXControlMode.PercentOutput, 0);
            extendoMotor.setPosition(Constants.IntakeConstants.MOVER_MOTOR_RETRACTED_ANGLE);
        });
    }

    public Command StopJustRoller() {
        return this.runOnce(() -> {
            spinningmotor.set(VictorSPXControlMode.PercentOutput, 0);
        });
    }
    
}
