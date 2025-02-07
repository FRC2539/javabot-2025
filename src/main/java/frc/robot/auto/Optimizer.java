package frc.robot.auto;

import java.util.Random;

public class Optimizer {
    public static interface Tweakable<T extends Tweakable<T>> {
        // the goal of this is to be maximized
        public double getReward();

        public T tweak(Random random);
    }

    public static <T extends Tweakable<T>> T optimize(T tweakable) {
        Random rand = new Random(2539);
        T bestTweakable = tweakable;
        double bestReward = tweakable.getReward();

        T absoluteBestTweakable = bestTweakable;
        double absoluteBestReward = bestReward;

        for (int i = 0; i < 1000; i++) {
            T nextTweakable = bestTweakable.tweak(rand);
            double nextReward = nextTweakable.getReward();
            if (nextReward > bestReward || rand.nextDouble() > 0.75) {
                bestReward = nextReward;
                bestTweakable = nextTweakable;
            }

            if (bestReward > absoluteBestReward) {
                absoluteBestReward = bestReward;
                absoluteBestTweakable = bestTweakable;
            }
        }

        return absoluteBestTweakable;
    }
}
