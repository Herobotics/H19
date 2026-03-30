package frc.robot.subsystems.swervedrive;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;

public class AlignerSubsystem extends SubsystemBase {
    private double distance;
    private double angle;
    private double area;  // Can never be 0
    private int numTargets;
    // Example LookupTable (distance in feet, RPS)
    private static final InterpolatingDoubleTreeMap SHOOTER_MAP = new InterpolatingDoubleTreeMap();
    static {
        SHOOTER_MAP.put(64.0, 60.0);
        SHOOTER_MAP.put(128.0, 85.0);
        SHOOTER_MAP.put(114.0, 80.0);
        SHOOTER_MAP.put(108.0, 78.0);
        SHOOTER_MAP.put(0.0, 60.0);
        SHOOTER_MAP.put(200.0, 100.0);
    }

    public AlignerSubsystem() {
        setEmptyValues();
        printResults();
        LimelightHelpers.setLEDMode_ForceOff("");  // Don't blind people
        LimelightHelpers.setPipelineIndex(Constants.limelight_name, 0);
    }

    public void printResults() {
        SmartDashboard.putBoolean("Aligned?", isAligned());
        SmartDashboard.putNumber("Distance From Hub", this.distance);
        SmartDashboard.putNumber("Angle from Hub", this.angle);
        SmartDashboard.putNumber("Area of AprilTag", this.area);
        SmartDashboard.putNumber("Num Targets", this.numTargets);
    }

    public void getAprilTag() {
        this.setEmptyValues();
        if (LimelightHelpers.getTV(Constants.limelight_name)) {
            this.area = LimelightHelpers.getTA(Constants.limelight_name);
            this.angle = LimelightHelpers.getTX(Constants.limelight_name);
            this.numTargets = LimelightHelpers.getTargetCount(Constants.limelight_name);
        }
        if (this.area > 0.0) {
            this.distance = Constants.AprilTag.AREA_FACTOR / (Math.sqrt(this.area));
        }
    }

    // Use the values to determine if we are aligned
    public boolean isAligned() {
        if (((this.distance > Constants.AprilTag.MIN_DISTANCE) &&
            (this.distance < Constants.AprilTag.MAX_DISTANCE)) &&
            ((this.angle > Constants.AprilTag.MIN_ANGLE) &&
            (this.angle < Constants.AprilTag.MAX_ANGLE))) {
            return true;
        }
        return false;
    }

    public double getRPS() {
        if ((this.distance > Constants.AprilTag.MIN_DISTANCE) &&
            (this.distance < Constants.AprilTag.MAX_DISTANCE)) {
                return SHOOTER_MAP.get(this.distance).doubleValue();
            }
        else {
            return -1.0;
        }
    }

    public double getTurnAmount() {
        return -1.0 * this.angle * Constants.AprilTag.K_ALIGN_P;
    }

    private void setEmptyValues() {
        this.distance = -1.0;
        this.angle = 0.0;
        this.area = -1.0;  // Can never be 0
        this.numTargets = -1;
    }

    @Override
    // This method will be called once per scheduler run
    public void periodic() {
        // Log stats
        getAprilTag();
        printResults();
    }
    
}