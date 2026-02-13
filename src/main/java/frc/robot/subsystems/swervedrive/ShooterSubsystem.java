package frc.robot.subsystems.swervedrive;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {
    private final TalonFX aimingMotor = new TalonFX(5);
    private static final Angle maxAngle = Angle(0.5);

    public ShooterSubsystem() {
        
    }

    // Aim shooter -- right thumb stick
    // TBD whether is part of the subsystem or not

    // Firing shooter

    // Stop shooter?

    // Spinning up the wheels

    // Aiming the shooter
    public Command AimUp() {
        return run(() -> {
            Angle desiredAngle = 
            aimingMotor.setPosition(aimingMotor.getPosition().getValue());
        });
    }
    
    public Command AimDown() {
        return 
    }
    
}
