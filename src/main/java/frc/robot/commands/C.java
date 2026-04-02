package frc.robot.commands;

import java.util.Optional;
import java.util.function.Consumer;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class C<T extends Subsystem> extends Command {
    T subsystem;
    double duration;
    Optional<Consumer<T>> executeFunction;
    Optional<Consumer<T>> initFunction;
    Optional<Consumer<T>> endFunction;

    Timer t = new Timer();

    public C(T s, Optional<Consumer<T>> initFunction, Optional<Consumer<T>> executeFunction, Optional<Consumer<T>> endFunction, double durationSeconds) {
        this.subsystem = s;
        this.executeFunction = executeFunction;
        this.initFunction    = initFunction   ;
        this.endFunction     = endFunction    ;

        this.duration = durationSeconds;

        addRequirements(this.subsystem);
    }

    public static <T extends Subsystem>C<T> resourceCommand(T s, Consumer<T> initFunction, Consumer<T> endFunction, double durationSeconds) {
        return new C<T>(s, Optional.of(initFunction), Optional.empty(), Optional.of(endFunction), durationSeconds);
    }

    public static <T extends Subsystem>C<T> durationCommand(T s, Consumer<T> initFunction, Consumer<T> executeFunction, Consumer<T> endFunction, double durationSeconds) {
        return new C<T>(s, Optional.of(initFunction), Optional.of(executeFunction), Optional.of(endFunction), durationSeconds);
    }

    public static <T extends Subsystem>C<T> durationCommand(T s, Consumer<T> executeFunction, double durationSeconds) {
        return new C<T>(s, Optional.empty(), Optional.of(executeFunction), Optional.empty(), durationSeconds);
    }

    public static <T extends Subsystem>C<T> durationCommand(T s, Consumer<T> executeFunction, Consumer<T> endFunction, double durationSeconds) {
        return new C<T>(s, Optional.empty(), Optional.of(executeFunction), Optional.of(endFunction), durationSeconds);
    }

    public static <T extends Subsystem>C<T> instantCommand(T s, Consumer<T> initFunction) {
        // is this guaranteed to initialize? check!
        return new C<T>(s, Optional.of(initFunction), Optional.empty(), Optional.empty(), 0);
    }

    @Override
    public void initialize() {
        this.t.restart();
        System.out.println("init, t = " + t.get());
        this.initFunction.ifPresent(f -> f.accept(this.subsystem));
    }

    @Override
    public void execute() {
        System.out.println("executing, t = " + t.get());
        this.executeFunction.ifPresent(f -> f.accept(this.subsystem));
    }

    @Override
    public boolean isFinished() {
        return t.hasElapsed(duration);
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("ending, t = " + t.get());
        this.endFunction.ifPresent(f -> f.accept(this.subsystem));
    }
}
