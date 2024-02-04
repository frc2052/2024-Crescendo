package frc.robot.states;

import java.util.ArrayList;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.shamper.ShamperAmpAngleCommand;
import frc.robot.commands.shamper.ShamperAngleCommand;
import frc.robot.commands.shamper.ShamperShootCommand;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.ShamperSubsystem;
import frc.robot.subsystems.ShamperSubsystem.ShamperSpeeds;
import frc.robot.util.AimingCalculator;
import frc.robot.subsystems.TrapArmSubsystem;

public class Superstructure extends SubsystemBase{
  private SuperstructureState state;
  
  private ShamperSubsystem shamper;
  private ClimberSubsystem climber;

  public  Superstructure(ShamperSubsystem shamper, ClimberSubsystem climber){
    this.shamper = shamper;
    this.climber = climber;
  }

  public void setState(SuperstructureState state){
    this.state = state;
    switch(state){
      case DEFAULT:
        setDefault();
      case SPEAKER_IDLE:
        setSpeakerIdle();
      case SPEAKER_SCORE:
        setSpeakerScoring();
      case AMP_IDLE:
        setAmpIdle();
      case AMP_SCORE:
        setAmpScore();
      case CLIMBING:
        setClimbing();
    }
  }

  public SuperstructureState getState() {
    return state;
  }

  private void setDefault() {
    new ShamperAngleCommand(shamper, Constants.Shamper.Angle.DEFAULT);
    new ShamperShootCommand(shamper, ShamperSpeeds.SPEAKER_IDLE);
  }

  //TODO: finish states
  private void setSpeakerIdle() {
    new ShamperAngleCommand(shamper, Constants.Shamper.Angle.DEFAULT);
    new ShamperShootCommand(shamper, ShamperSpeeds.SPEAKER_IDLE);
  }
  
  private void setSpeakerScoring() {
    new ShamperAngleCommand(shamper, );
    new ShamperShootCommand(shamper, ShamperSpeeds.SPEAKER_IDLE);
  }  
  
  private void setAmpIdle() {
    new ShamperAngleCommand(shamper, Constants.Shamper.Angle.AMP);
    new ShamperShootCommand(shamper, ShamperSpeeds.AMP);
  }  

  private void setAmpScore() {
    new ShamperAngleCommand(shamper, Constants.Shamper.Angle.AMP);
    new ShamperShootCommand(shamper, ShamperSpeeds.AMP);
  }

  private void setClimbing() {
    new ShamperAngleCommand(shamper, Constants.Shamper.Angle.CLIMB);
    new ShamperShootCommand(shamper, ShamperSpeeds.OFF);
  }

  public enum SuperstructureState {
    DEFAULT,
    SPEAKER_IDLE,
    SPEAKER_SCORE,
    AMP_IDLE,
    AMP_SCORE,
    CLIMBING;
  }
}
