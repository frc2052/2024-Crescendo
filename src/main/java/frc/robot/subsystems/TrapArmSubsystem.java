package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class TrapArmSubsystem extends SubsystemBase{
    private final Relay trapArm;
    private boolean open;

    public TrapArmSubsystem() {
        trapArm = new Relay(Constants.Trap.TRAP_RELAY_PIN);
        trapArm.set(Relay.Value.kReverse);
    }

    public void open(){
        open = true;
        trapArm.set(Relay.Value.kForward);
    }

    public void close(){
        open = false;
        trapArm.set(Relay.Value.kReverse);
    }

    public void off(){
        trapArm.set(Relay.Value.kOff);
    }

    public boolean getIsOpen(){
        return open;
    }
}
