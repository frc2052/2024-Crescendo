package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotState;
import frc.robot.auto.AutoFactory.Auto;
import frc.robot.util.io.Dashboard;

/**
 * Subsystem to control the robot's LEDs, by determining what number should be encoded to DIO pins and
 * sent to the Arduino we used for controlling the patterns and colors
 */
public class LedSubsystem extends SubsystemBase {
    private static LedSubsystem INSTANCE;
    
    private final DigitalOutput codeChannel1, codeChannel2, codeChannel3, codeChannel4, codeChannel5;

    private LEDStatusMode currentStatusMode;

    private boolean disableLEDs;
    private boolean robotDisabled;

    private LedSubsystem() {
        // DIO outputs
        codeChannel1 = new DigitalOutput(Constants.LEDs.CHANNEL_1_PIN);
        codeChannel2 = new DigitalOutput(Constants.LEDs.CHANNEL_2_PIN);
        codeChannel3 = new DigitalOutput(Constants.LEDs.CHANNEL_3_PIN);
        codeChannel4 = new DigitalOutput(Constants.LEDs.CHANNEL_4_PIN);
        codeChannel5 = new DigitalOutput(Constants.LEDs.CHANNEL_5_PIN);
        robotDisabled = true;

        currentStatusMode = LEDStatusMode.OFF;

        // For manually inputing code to encode to DIO pins
        // SmartDashboard.putNumber("LED CODE", 0);
    }

    public static LedSubsystem getInstance() {  // Method to allow calling this class and getting the single instance from anywhere, creating the instance if the first time.
        if (INSTANCE == null) {
            INSTANCE = new LedSubsystem();
        }
        return INSTANCE;
    }

    public static enum LEDStatusMode {
        OFF(5), // have to do 5 because we don't have pin 8 and it reads pin 8 high when sending 0
        CONE(1),
        CUBE(2),
        DISABLED_RED_PULSE(3),
        DISABLED_BLUE_PULSE(4),
        NO_AUTO(5),
        NOTE_DETECTED(6),
        RAINBOW(7),
        LAVA(8),
        WATER(9);

        private final int code;

        private LEDStatusMode(int code) {
            this.code = code;
        }

        public int getPositionTicks() {
            return code;
        }
    }

    @Override
    public void periodic() {
        int code = 5;
        if(!disableLEDs) {
            if (DriverStation.isDisabled()) {
                // If disabled, finds gets the alliance color from the driver station and pulses that. Only pulses color if connected to station or FMS, else pulses default disabled color (Firefl status mode)
                Auto selected = Dashboard.getInstance().getAuto();
                if (selected == Auto.NO_AUTO || selected == null){
                    currentStatusMode = LEDStatusMode.NO_AUTO;
                } else if (RobotState.getInstance().isRedAlliance()) {
                    currentStatusMode = LEDStatusMode.LAVA;
                } else if (!RobotState.getInstance().isRedAlliance()) {
                    currentStatusMode = LEDStatusMode.WATER;
                } else {
                    currentStatusMode = LEDStatusMode.OFF; // Reaches here if DriverStation.getAlliance returns Invalid, which just means it can't determine our alliance and we do cool default effect
                }
            } else if (DriverStation.isAutonomous()) {
                if(RobotState.getInstance().getNoteHeldDetected()){
                    currentStatusMode = LEDStatusMode.NOTE_DETECTED;
                } else if (RobotState.getInstance().getIsShamperAtGoalAngle()){
                    currentStatusMode = LEDStatusMode.RAINBOW;
                } else {
                    if (RobotState.getInstance().isRedAlliance()) {
                        currentStatusMode = LEDStatusMode.LAVA;
                    } else if (!RobotState.getInstance().isRedAlliance()) {
                        currentStatusMode = LEDStatusMode.WATER;
                    }
                }
            }

            if (DriverStation.isTeleopEnabled()) {
                if(RobotState.getInstance().getNoteHeldDetected() && RobotState.getInstance().getIsShamperAtGoalAngle()){
                    currentStatusMode = LEDStatusMode.CUBE;
                } else if(RobotState.getInstance().getNoteHeldDetected()){
                    currentStatusMode = LEDStatusMode.CONE;
                } else {
                    currentStatusMode = LEDStatusMode.NO_AUTO;
                }

            }

            code = currentStatusMode.code;
        } else {
            //LEDs are disabled
            code = 5;
        }

        // Code for encoding the code to binary on the digitalOutput pins
        Dashboard.getInstance().putData("Sending LED Code", code);
        codeChannel1.set((code & 1) > 0);   // 2^0
        codeChannel2.set((code & 2) > 0);   // 2^1
        codeChannel3.set((code & 4) > 0);   // 2^2
        codeChannel4.set((code & 8) > 0);   // 2^3
        codeChannel5.set((code & 16) > 0);  // 2^4
    }

    public void setLEDStatusMode(LEDStatusMode statusMode) {
        if (!disableLEDs) {
            currentStatusMode = statusMode;
        }
    }

    public LEDStatusMode getLEDStatusMode(){
        return currentStatusMode;
    }

    public void clearStatusMode() {
        currentStatusMode = LEDStatusMode.OFF;
    }

    // Disables LEDs (turns them off)
    public void disableLEDs() {
        disableLEDs = true;
    }

    // Enables LEDs (turns them on)
    public void enableLEDs() {
        disableLEDs = false;
    }

    public boolean getRobotDisabled(){
        return robotDisabled;
    }
}