package frc.robot.lib.phoenixpro;

import java.util.function.Consumer;
import java.util.function.Function;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;

public class CANcoderLiveConfigHelper {
    /**
     * Allows editing of a CANcoder configuration at runtime by pulling, modifying, and applying a configuration.
     * @param encoder The CANcoder to configure.
     * @param mutator A function that accepts the current cancoder config, and returns a new config to be applied.
     */
    public static void editConfig(CANcoder encoder, Function<CANcoderConfiguration, CANcoderConfiguration> mutator) {
        final CANcoderConfiguration config = new CANcoderConfiguration();
        PhoenixErrorChecker.checkErrorAndRetry(() -> encoder.getConfigurator().refresh(config, 0.1));
        CANcoderConfiguration newConfig = mutator.apply(config);
        PhoenixErrorChecker.checkErrorAndRetry(() -> encoder.getConfigurator().apply(newConfig, 0.1));
    }

    /**
     * Allows a function to be performed that uses information from a cancoder config.
     * @param encoder The CANcoder to get the config from.
     * @param consumer A function that accepts a cancoder config, but does not return anything.
     */
    public static void usingConfig(CANcoder encoder, Consumer<CANcoderConfiguration> consumer) {
        CANcoderConfiguration config = new CANcoderConfiguration();
        PhoenixErrorChecker.checkErrorAndRetry(() -> encoder.getConfigurator().refresh(config, 0.1));
        consumer.accept(config);
    }

    /**
     * Allows the code to return values of any type from a cancoder's configuration. Note that multiple values
     * can be saved from the configuration if they are returned as a single object.
     * @param <T> The type of the object returned.
     * @param encoder The CANcoder to use the config from.
     * @param consumer A function that takes a CANcoder config, and returns an object of type T.
     * @return The object returned when the cancoder's config is applied to the consumer funtion.
     */
    public static <T> T getValueFromConfig(CANcoder encoder, Function<CANcoderConfiguration, T> consumer) {
        CANcoderConfiguration config = new CANcoderConfiguration();
        PhoenixErrorChecker.checkErrorAndRetry(() -> encoder.getConfigurator().refresh(config, 0.1));
        return consumer.apply(config);
    }
}
