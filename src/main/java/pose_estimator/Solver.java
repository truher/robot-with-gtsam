package pose_estimator;

import gtsam.BatchFixedLagSmoother;
import gtsam.FixedLagSmoother;
import gtsam.FixedLagSmoother.Result;
import gtsam.GaussianFactorGraph;
import gtsam.ISAM2Params;
import gtsam.IncrementalFixedLagSmoother;
import gtsam.Key;
import gtsam.KeyVector;
import gtsam.Marginals;
import gtsam.Matrix;
import gtsam.NonlinearFactor;
import gtsam.NonlinearFactorGraph;
import gtsam.Pose2;
import gtsam.Values;
import gtsam.Vector;
import gtsam.Vector3;
import gtsam.VectorValues;
import gtsam.shared_ptr;

/**
 * Port of estimate.py from 2024.
 */
public class Solver {
    private final boolean INCREMENTAL;
    private final BatchFixedLagSmoother batchSmoother;
    private final IncrementalFixedLagSmoother incrementalSmoother;
    private final NonlinearFactorGraph new_factors;
    private final Values new_values;
    private final NonlinearFactorGraph fullGraph;
    private final Values fullValues;

    /** key is Key, "X(timestamp in us)", value is timestamp in us */
    private final FixedLagSmoother.KeyTimestampMap new_timestamps;

    private Values result;

    /** @param lag in microseconds, not seconds as in python */
    public Solver(double lag, boolean incremental) throws Throwable {
        INCREMENTAL = incremental;
        // Initialize the model
        // initial module positions are at their origins.
        // TODO: some other initial positions?

        // Incremental solver
        ISAM2Params isam2Params = new ISAM2Params();
        isam2Params.findUnusedFactorSlots(true);
        incrementalSmoother = new IncrementalFixedLagSmoother(lag, isam2Params);

        // Batch solver
        batchSmoother = new BatchFixedLagSmoother(lag);

        result = new Values();
        // between updates we accumulate inputs here

        new_factors = new NonlinearFactorGraph();
        new_values = new Values();
        new_timestamps = new FixedLagSmoother.KeyTimestampMap();

        fullGraph = new NonlinearFactorGraph();
        fullValues = new Values();
    }

    public void addVariable(Key key, double time_us, Pose2 initial_value) throws Throwable {
        // System.out.print("adding key:\n");
        // key.print();
        // System.out.printf("with value (%f %f %f)\n",
        // initial_value.x(), initial_value.y(), initial_value.theta());
        if (exists(key))
            return;
        new_values.insert(key, initial_value);
        fullValues.insert(key, initial_value);
        new_timestamps.put(key, time_us);
        // System.out.println("added!");
    }

    public void addVariable(Key key, double time_us, double initial_value) throws Throwable {
        // System.out.print("adding key:\n");
        // key.print();
        // System.out.printf("with value: %f\n", initial_value);
        if (exists(key))
            return;
        new_values.insert(key, initial_value);
        fullValues.insert(key, initial_value);
        // System.out.printf("adding timestamp %f\n", time_us);
        new_timestamps.put(key, time_us);
        // System.out.println("added!");
    }

    private boolean exists(Key k) throws Throwable {
        return result.exists(k) || new_values.exists(k);
    }

    /**
     * Add a factor to the graph.
     * The variables referenced must exist.
     * returns true if successful.
     */
    public <T extends NonlinearFactor> boolean add(shared_ptr<T> f)
            throws Throwable {
        KeyVector keys = f.get().keys();
        for (int i = 0; i < keys.size(); ++i) {
            Key k = keys.at(i);
            if (!new_values.exists(k) && !result.exists(k)) {
                complain(f, k);
                return false;
            }
        }
        // System.out.println("adding factor");
        // f.get().print();
        new_factors.add(f);
        fullGraph.add(f);
        return true;
    }

    private <T extends NonlinearFactor> void complain(
            shared_ptr<T> f, Key k) throws Throwable {
        System.out.println("FATAL ERROR!");
        System.out.println("factor:");
        System.out.flush();
        f.get().print();
        System.out.flush();
        System.out.println("key does not exist in results or new values:");
        System.out.flush();
        k.print();
        System.out.println("result:");
        result.print("result");
        System.out.println("new values:");
        new_values.print("new values");
    }

    /**
     * Run the solver
     */
    public void update() throws Throwable {
        // System.out.println("update");
        // new_factors.print("new factors");
        // new_values.print("new values");
        if (INCREMENTAL) {
            Result updateResult = incrementalSmoother.update(new_factors, new_values, new_timestamps);
            // updateResult.print();
            result = incrementalSmoother.calculateEstimate();
        } else {
            Result updateResult = batchSmoother.update(new_factors, new_values, new_timestamps);
            // updateResult.print();
            result = batchSmoother.calculateEstimate();
        }

        // reset the accumulators
        new_factors.resize(0);
        new_values.clear();
        new_timestamps.clear();
    }

    // public long result_size() throws Throwable {
    //     // result.print();
    //     return result.size();
    // }

    /** The mean expected pose. */
    public Pose2 mean_pose2(Key key) throws Throwable {
        return result.atPose2(key);
    }

    public double mean_double(Key key) throws Throwable {
        // System.out.println("Looking for double key:");
        // key.print();
        // System.out.println("In results:");
        // result.print("");
        return result.atDouble(key);
    }

    public Vector sigma_pose2(Key key) throws Throwable {
        Marginals m = marginal_covariance();
        Matrix s = m.marginalCovariance(key);
        return s.diagonal_cwiseSqrt();
    }

    public Marginals marginal_covariance() throws Throwable {
        if (INCREMENTAL) {
            NonlinearFactorGraph factors = incrementalSmoother.getFactors();
            return new Marginals(factors, result);
        } else {
            NonlinearFactorGraph factors = batchSmoother.getFactors();
            return new Marginals(factors, result);

        }
    }

    public Pose2 sample_Pose2(Key key) throws Throwable {
        Marginals marginals = marginal_covariance();
        Matrix cov = marginals.marginalCovariance(key);
        Vector3 t = new Vector3(cov.draw());
        return mean_pose2(key).expmap(t);
    }

    public void check_smoother() throws Throwable {
        check_smoother(fullGraph, fullValues, incrementalSmoother, Key.X(0));
    }

    void check_smoother(final NonlinearFactorGraph fullgraph,
            final Values fullinit,
            final IncrementalFixedLagSmoother smoother,
            final Key key) throws Throwable {
        shared_ptr<GaussianFactorGraph> linearized = fullgraph.linearize(fullinit);
        VectorValues delta = linearized.get().optimize();
        Values fullfinal = fullinit.retract(delta);

        Pose2 expected = fullfinal.atPose2(key);
        Pose2 actual = smoother.calculateEstimatePose2(key);

        System.out.printf("expected (%f %f)\n", expected.x(), expected.y());
        System.out.printf("actual (%f %f)\n", actual.x(), actual.y());
    }

}
