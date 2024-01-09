package frc.robot.lib.dashboard;

import java.util.ArrayList;
import java.util.Collection;
import java.util.function.Supplier;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;

public class SupplierWidget<T> {
    private static final Collection<SupplierWidget<?>> list = new ArrayList<>();

    private final Supplier<T> mSupplier;
    private final GenericEntry mEntry;

    public SupplierWidget(String tab, String title, T initialValue, Supplier<T> supplier, WidgetConfig config) {
        mSupplier = supplier;

        if (config.getUseBuiltInWidgets()) {
            mEntry = Shuffleboard.getTab(tab).add(title, initialValue)
                .withPosition(config.getColumn(), config.getRow())
                .withSize(config.getWidth(), config.getHeight())
                .withWidget(config.getBuiltInWidget())
                .withProperties(config.getProperties())
                .getEntry();
        } else {
            mEntry = Shuffleboard.getTab(tab).add(title, initialValue)
                .withPosition(config.getColumn(), config.getRow())
                .withSize(config.getWidth(), config.getHeight())
                .withWidget(config.getExternalWidget())
                .withProperties(config.getProperties())
                .getEntry();
        }
        
        list.add(this);
    }

    public void update() {
        mEntry.setValue(mSupplier.get());
    }

    public static void updateAll() {
        list.forEach((w) -> w.update());
    }
}
