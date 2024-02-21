package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.RobotState;
import frc.robot.commands.climb.ClimberRetractCommand;
import frc.robot.commands.indexer.IndexerIndexCommand;
import frc.robot.commands.indexer.IndexerLoadCommand;
import frc.robot.commands.intake.IntakeCommand;
import frc.robot.commands.climb.ClimberExtendCommand;
import frc.robot.commands.shamper.ShamperAngleCommand;
import frc.robot.commands.shamper.ShamperIdleCommand;
import frc.robot.commands.shamper.ShamperShootCommand;
import frc.robot.subsystems.ShamperSubsystem.ShamperSpeed;
import frc.robot.util.AimingCalculator;
import frc.robot.util.calculator.ShootingAngleCalculator;

public class Superstructure extends SubsystemBase {
  private SuperstructureState state;
  private ShamperSubsystem shamper;
  private IndexerSubsystem indexer;
  private IntakeSubsystem intake;

  public  Superstructure(ShamperSubsystem shamper, IndexerSubsystem indexer, IntakeSubsystem intake) {
    this.shamper = shamper;
    this.indexer = indexer;
    this.intake = intake;

    this.state = SuperstructureState.DEFAULT;
  }

  public void setState(SuperstructureState state) {
    this.state = state;
    switch(state){
      case DEFAULT:
        setDefault();
      case INTAKE:
        setIntake();
      case PODIUM_IDLE:
        setPodiumIdle();
      case PODIUM_SCORE:
        setPodiumScoring();
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
    SequentialCommandGroup setDefaultPositionCommand = new SequentialCommandGroup(
      new ShamperAngleCommand(shamper, Constants.Shamper.Angle.DEFAULT),
      new ShamperShootCommand(shamper, indexer, ShamperSpeed.OFF)
    );
    
    setDefaultPositionCommand.schedule();
  }

  private void setIntake(){
    SequentialCommandGroup setIntakePositionCommand = new SequentialCommandGroup(
      new IntakeCommand(intake),
      new IndexerLoadCommand(indexer) 
    );

    setIntakePositionCommand.schedule();
  }

  private void setPodiumIdle() {
    SequentialCommandGroup setPodiumIdlePositionCommand = new SequentialCommandGroup(
      new ShamperAngleCommand(shamper, Constants.Shamper.Angle.DEFAULT),
      new ShamperIdleCommand(shamper, ShamperSpeed.SPEAKER_IDLE)
    );

    setPodiumIdlePositionCommand.schedule();
  }

  private void setPodiumScoring() {
    SequentialCommandGroup setPodiumScoringCommand = new SequentialCommandGroup(
      new ShamperAngleCommand(shamper, Constants.Shamper.Angle.DEFAULT),
      new ShamperShootCommand(shamper, indexer, ShamperSpeed.SPEAKER_SCORE)
    );

    setPodiumScoringCommand.schedule();
  }

  private void setSpeakerIdle() {
    SequentialCommandGroup setSpeakerIdleCommand = new SequentialCommandGroup(
      new ShamperAngleCommand(shamper, ShootingAngleCalculator.getInstance().getShooterConfig(RobotState.getInstance().getRobotPose()).getAngleDegrees()),
      new ShamperIdleCommand(shamper, ShamperSpeed.SPEAKER_IDLE)
    );

    setSpeakerIdleCommand.schedule();
  }
  
  private void setSpeakerScoring() {
    SequentialCommandGroup setSpeakerScoringCommand = new SequentialCommandGroup(
      new ShamperAngleCommand(shamper, ShootingAngleCalculator.getInstance().getShooterConfig(RobotState.getInstance().getRobotPose()).getAngleDegrees()),
      new ShamperShootCommand(shamper, indexer, ShamperSpeed.SPEAKER_SCORE, ShamperSpeed.SPEAKER_IDLE)
    );

    setSpeakerScoringCommand.schedule();
  }
  
  private void setAmpIdle() {
    SequentialCommandGroup setAmpIdleCommand = new SequentialCommandGroup(
      new ShamperAngleCommand(shamper, Constants.Shamper.Angle.AMP),
      new ShamperIdleCommand(shamper, ShamperSpeed.AMP_IDLE)
    );
    
    setAmpIdleCommand.schedule();
  }  

  private void setAmpScore() {
    SequentialCommandGroup setAmpScoreCommand = new SequentialCommandGroup(
      new ShamperAngleCommand(shamper, Constants.Shamper.Angle.AMP),
      new ShamperShootCommand(shamper, indexer, ShamperSpeed.AMP_SCORE, ShamperSpeed.SPEAKER_IDLE)
    );

    setAmpScoreCommand.schedule();
  }

  private void setClimbing() {
    // no
  }

  @Override
  public void periodic() {
    if(state.needsRefresh()){
      setState(state);
    }
    
    System.out.println(Math.toDegrees(ShootingAngleCalculator.getInstance().getShooterConfig(RobotState.getInstance().getRobotPose()).getAngleDegrees()));
  }

  public enum SuperstructureState {
    DEFAULT(false),
    INTAKE(false),
    PODIUM_IDLE(false),
    PODIUM_SCORE(false),
    SPEAKER_IDLE(true),
    SPEAKER_SCORE(true),
    AMP_IDLE(false),
    AMP_SCORE(false),
    CLIMBING(false);

    private boolean refreshRequired;

    private SuperstructureState(boolean refreshRequired) {
      this.refreshRequired = refreshRequired;
    }

    public boolean needsRefresh() {
      return refreshRequired;
    }
  }
}
