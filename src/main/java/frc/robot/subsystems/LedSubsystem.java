package frc.robot.subsystems;

import Constants.java;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDSubsystem extends SubsystemBase {
     private static LEDSubsystem INSTANCE;
    
    private final DigitalOutput codeChannel1, codeChannel2, codeChannel3;
    private LEDStatusMode currentStatusMode;

    private boolean disableLEDs;
    private boolean robotDisabled;
    


    private LEDSubsystem() {
        // DIO outputs
        codeChannel1 = new DigitalOutput(Constants.LEDs.CHANNEL_1_PIN);
        codeChannel2 = new DigitalOutput(Constants.LEDs.CHANNEL_2_PIN);
        codeChannel3 = new DigitalOutput(Constants.LEDs.CHANNEL_3_PIN);
        codeChannel4 = new DigitalOutput(Constants.LEDs.CHANNEL_4_PIN);
        codeChannel5 = new DigitalOutput(Constants.LEDs.CHANNEL_5_PIN);
        
        robotDisabled = true;
        

        currentStatusMode = LEDStatusMode.OFF;

}

    public static LEDSubsystem getInstance() { 
            if (INSTANCE == null) {
            INSTANCE = new LEDSubsystem();
        }
        return INSTANCE;
    }


    public static enum LEDStatusMode {
        amp(0),
        coop(1),
        neutral(2),
        red(3),
        blue(4),


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
        int code = 0;
        if(!disableLEDs) {
            if (robotDisabled) {
                if  (DriverStation.getAlliance() == Alliance.Red) {
                    currentStatusMode = LEDStatusMode.red
                } else if (DriverStation.getAlliance() == Alliance.Blue) {
                    currentStatusMode = LEDStatusMode.blue
                } else {
                    currentStatusMode = LEDStatusMode.neutral
                }
            }

            code = currentStatusMode.code;
        } else {
            code = 2;
        }

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

    public void robotDisabled(){
        robotDisabled = true;
    }

    public void robotEnabled(){
        robotDisabled = false;
    }

    public boolean getRobotDisabled(){
        return robotDisabled;
    }







