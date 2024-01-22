package org.firstinspires.ftc.teamcode.Robot.Commands.ScoringCommands.PrimitiveMovements;

import org.firstinspires.ftc.teamcode.CommandFramework.Command;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.ScoringMechanism.Slides;

public class MoveSlides extends Command {
    Slides slides;

    double increment;

    public MoveSlides(Slides slides, double increment) {
        this.slides = slides;
        this.increment = increment;
    }

    @Override
    public void init() {
        slides.setSlideTargetPosition(increment);
    }

    @Override
    public void periodic() {
    }

    @Override
    public boolean completed() {
        return true;
    }

    @Override
    public void shutdown() {

    }
}
