package pose_estimator;

import gtsam.BatchFixedLagSmoother;
import gtsam.FixedLagSmoother;
import gtsam.Key;
import gtsam.Marginals;
import gtsam.Matrix;
import gtsam.NonlinearFactor;
import gtsam.NonlinearFactorGraph;
import gtsam.Pose2;
import gtsam.Values;
import gtsam.Vector;
import gtsam.Vector3;
import gtsam.shared_ptr;

/**
 * Port of estimate.py from 2024.
 */
public class Solver {
    private final BatchFixedLagSmoother isam;
    private final NonlinearFactorGraph new_factors;
    private final Values new_values;
    /** key is Key, "X(timestamp in us)", value is timestamp in us */
    private final FixedLagSmoother.KeyTimestampMap new_timestamps;

    private Values result;

    /** @param lag in microseconds, not seconds as in python */
    public Solver(double lag) throws Throwable {

        // Initialize the model
        // initial module positions are at their origins.
        // TODO: some other initial positions?

        isam = new BatchFixedLagSmoother(lag);
        result = new Values();
        // between updates we accumulate inputs here

        new_factors = new NonlinearFactorGraph();
        new_values = new Values();
        new_timestamps = new FixedLagSmoother.KeyTimestampMap();
    }

    /**
     * Add a new robot pose variable to the estimator, if it doesn't already exist.
     * 
     * @param time_us
     * @param initial_value cloned, ok to delete after this.
     */
    public void addVariable(long time_us, Pose2 initial_value) throws Throwable {
        Key key = Key.X(time_us);
        // System.out.printf("add state %d\n", key.j);
        if (result.exists(key)) {
            // System.out.printf("Key %d is already in the model\n", key.j);
            return;
        }
        if (new_values.exists(key)) {
            // System.out.printf("Key %d is already in the values", key.j);
            return;
        }
        // if you're using the batch smoother, the initial value
        // almost doesn't matter:
        // TODO: use the previous pose as the initial value
        new_values.insert(key, initial_value);
        new_timestamps.put(key, time_us);
    }

    /** Add a factor to the graph. */
    public <T extends NonlinearFactor> void add(shared_ptr<T> f)
            throws Throwable {
        new_factors.add(f);
    }

    /**
     * Run the solver
     */
    public void update() throws Throwable {
        isam.update(new_factors, new_values, new_timestamps);
        result = isam.calculateEstimate();

        // reset the accumulators
        new_factors.resize(0);
        new_values.clear();
        new_timestamps.clear();
    }

    public long result_size() throws Throwable {
        // result.print();
        return result.size();
    }

    /** The mean expected pose. */
    public Pose2 mean_pose2(Key key) throws Throwable {
        return result.atPose2(key);
    }

    public Vector sigma_pose2(Key key) throws Throwable {
        Marginals m = marginal_covariance();
        Matrix s = m.marginalCovariance(key);
        return s.diagonal_cwiseSqrt();
    }

    public Marginals marginal_covariance() throws Throwable {
        NonlinearFactorGraph factors = isam.getFactors();
        return new Marginals(factors, result);
    }

    public Pose2 sample_Pose2(Key key) throws Throwable {
        Marginals marginals = marginal_covariance();
        Matrix cov = marginals.marginalCovariance(key);
        Vector3 t = new Vector3(cov.draw());
        return mean_pose2(key).expmap(t);
    }

}
