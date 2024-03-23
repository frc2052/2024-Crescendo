package frc.robot.util.io;

import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.networktables.BooleanTopic;
import edu.wpi.first.networktables.DoubleArrayTopic;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.auto.AutoFactory.Auto;

// Trying something different this year, instead of a normal class
// the dashboard this year is using a singleton class, so only one instance will
// run at once
public class Dashboard {
    private static Dashboard INSTANCE;
    private final NetworkTableInstance ntinst;
    private final NetworkTable rPiTable;

    private final SendableChooser<DriveMode> driveModeChooser;
    private final SendableChooser<Idle> shamperIdleModeChooser;
    private final SendableChooser<Auto> autoChooser;
    private final SendableChooser<ClimberMode> climberModeChooser;

    public static Dashboard getInstance() {
        if (INSTANCE == null) {
            INSTANCE = new Dashboard();
        }

        return INSTANCE;
    }

    private Dashboard() {
        ntinst = NetworkTableInstance.getDefault();
        rPiTable = ntinst.getTable("RaspberryPi");

        //Creates options for different choosers

        driveModeChooser = new SendableChooser<DriveMode>();
        driveModeChooser.addOption(DriveMode.FIELD_CENTRIC.name(), DriveMode.FIELD_CENTRIC);
        driveModeChooser.addOption(DriveMode.ROBOT_CENTRIC.name(), DriveMode.ROBOT_CENTRIC);
        driveModeChooser.setDefaultOption(DriveMode.FIELD_CENTRIC.name(), DriveMode.FIELD_CENTRIC);
        SmartDashboard.putData(Constants.Dashboard.DRIVE_MODE_KEY, driveModeChooser);

        shamperIdleModeChooser = new SendableChooser<Idle>();
        shamperIdleModeChooser.addOption(Idle.NO_IDLE.name(), Idle.NO_IDLE);
        shamperIdleModeChooser.addOption(Idle.IDLE.name(), Idle.IDLE);
        shamperIdleModeChooser.setDefaultOption(Idle.IDLE.name(), Idle.IDLE);
        SmartDashboard.putData(Constants.Dashboard.IDLE_CHOOSER_KEY, shamperIdleModeChooser);
        
        autoChooser = new SendableChooser<Auto>();
        for (Auto auto : Auto.values()) {
            autoChooser.addOption(auto.name(), auto);
        }
        autoChooser.setDefaultOption(Auto.NO_AUTO.name(), Auto.NO_AUTO);
        SmartDashboard.putData("Auto", autoChooser);

        climberModeChooser = new SendableChooser<ClimberMode>();
        climberModeChooser.addOption(ClimberMode.BRAKE.name(), ClimberMode.BRAKE);
        climberModeChooser.addOption(ClimberMode.COAST.name(), ClimberMode.COAST);
        climberModeChooser.setDefaultOption(ClimberMode.BRAKE.name(), ClimberMode.BRAKE);
        SmartDashboard.putData(Constants.Dashboard.CLIMBER_MODE_KEY, climberModeChooser);
        
        SmartDashboard.putString(Constants.Dashboard.IDLE_MODE_KEY, ShamperIdleMode.SPEAKER_IDLE.name());

        SmartDashboard.putNumber("Shamper Angle Manual", 0);
    }

    public <V> void putData(String key, V value) {
        if (value instanceof Float) {
            SmartDashboard.putNumber(key, (Float) value);
        } else if (value instanceof Integer) {
            SmartDashboard.putNumber(key, (Integer) value);
        } else if (value instanceof Number) {
            SmartDashboard.putNumber(key, (Double) value);
        } else if (value instanceof String) {
            SmartDashboard.putString(key, (String) value);
        } else if (value instanceof Boolean) {
            SmartDashboard.putBoolean(key, (Boolean) value);
        } else if (value instanceof Sendable) {
            Shuffleboard.getTab("main").add(key, (Sendable) value);
        }
    }

    public DoubleArrayTopic getRaspberryPiCameraPoseMeters(String cameraName){
        return rPiTable.getDoubleArrayTopic(cameraName);
    }

    public BooleanTopic getRaspberryPiValidReadingState(String cameraName){
        return rPiTable.getBooleanTopic(cameraName + "tagFound");
    }
    
    public NetworkTable getrPiTable(){
        return rPiTable;
    }

    public boolean pixyCamBroken() {
        return SmartDashboard.getBoolean("Pixy Cam Broken", false);
    }

    public boolean isFieldCentric() {
        return driveModeChooser.getSelected() == DriveMode.FIELD_CENTRIC;      
    }

    public boolean isClimberCoast() {
        return climberModeChooser.getSelected() == ClimberMode.COAST;
    }

    public void updateIsClimbing(boolean isClimbing) {
        SmartDashboard.putBoolean("Is Climbing", isClimbing);
    }
    
    public boolean shouldIdle() {
        return shamperIdleModeChooser.getSelected() == Idle.IDLE;
    }

    public Auto getAuto() {
        return autoChooser.getSelected();
    }

    // Enums for Dashboard elements:
    public static enum DriveMode {
        FIELD_CENTRIC,
        ROBOT_CENTRIC;
    }

    public static enum ClimberMode {
        BRAKE,
        COAST;
    }

    public static enum Idle {
        NO_IDLE,
        IDLE;
    }

    public static enum ShamperIdleMode {
        SPEAKER_IDLE,
        AMP_IDLE;
    }
}
