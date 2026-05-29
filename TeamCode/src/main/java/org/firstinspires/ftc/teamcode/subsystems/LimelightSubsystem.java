package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.config.AllianceConfig;
import org.firstinspires.ftc.teamcode.config.DistanceTable;

import java.util.List;

/**
 * LimelightSubsystem wraps all Limelight access.
 *
 * On every loop call update() exactly once. After that, the cached
 * accessors (hasTarget, getDistanceInches, etc.) reflect what was seen.
 *
 * Diagnostics: if tags aren't being detected, the getter methods below
 * (getLastStalenessMs, getVisibleTagCount, getVisibleTagIds) tell you
 * what the camera actually reported. Use them on telemetry when
 * debugging.
 */
public class LimelightSubsystem {

    private static final double METERS_TO_INCHES = 39.3701;
    private static final long STALE_RESULT_MS = 100;

    private final Limelight3A limelight;
    private final AllianceConfig allianceConfig;
    private boolean connected = false;
    private String initErrorMessage = null;

    // Cached per-loop state
    private LLResult lastResult = null;
    private boolean hasTarget = false;
    private double distanceInches = 0.0;
    private double txDegrees = 0.0;
    private List<LLResultTypes.FiducialResult> lastFiducials = null;

    // Diagnostics: track why tags might be missing
    private long lastStalenessMs = -1;
    private boolean lastResultValid = false;
    private int lastVisibleTagCount = 0;
    private String lastVisibleTagIds = "";
    private String lastSkipReason = "no update() called yet";

    public LimelightSubsystem(HardwareMap hardwareMap, AllianceConfig allianceConfig) {
        this.allianceConfig = allianceConfig;
        Limelight3A ll = null;
        try {
            ll = hardwareMap.get(Limelight3A.class, "limelight");
            ll.setPollRateHz(100);
            ll.pipelineSwitch(allianceConfig.limelightPipeline);
            ll.start();
            connected = true;
        } catch (Exception e) {
            initErrorMessage = e.getMessage();
            connected = false;
        }
        limelight = ll;
    }

    /**
     * Poll Limelight once and cache results. Call exactly once per loop.
     */
    public void update() {
        // Reset per-frame state. Diagnostics are NOT reset — they're
        // overwritten below so the last meaningful value persists.
        hasTarget = false;
        distanceInches = 0.0;
        txDegrees = 0.0;
        lastFiducials = null;

        if (!connected || limelight == null) {
            lastSkipReason = "not connected";
            return;
        }

        LLResult result = limelight.getLatestResult();
        lastResult = result;

        if (result == null) {
            lastResultValid = false;
            lastStalenessMs = -1;
            lastVisibleTagCount = 0;
            lastVisibleTagIds = "";
            lastSkipReason = "result == null";
            return;
        }

        lastResultValid = result.isValid();
        lastStalenessMs = result.getStaleness();

        if (!lastResultValid) {
            lastVisibleTagCount = 0;
            lastVisibleTagIds = "";
            lastSkipReason = "result.isValid() == false";
            return;
        }

        if (lastStalenessMs > STALE_RESULT_MS) {
            lastSkipReason = "result stale (" + lastStalenessMs + " ms)";
            // Note: we still parse fiducials below for diagnostics, so the
            // driver can see "yes the camera is seeing tags, they're just
            // arriving too slowly".
        }

        List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
        lastFiducials = fiducials;

        if (fiducials == null || fiducials.isEmpty()) {
            lastVisibleTagCount = 0;
            lastVisibleTagIds = "";
            lastSkipReason = "no fiducials in result";
            return;
        }

        // Record what we DO see, for diagnostics
        lastVisibleTagCount = fiducials.size();
        StringBuilder ids = new StringBuilder();
        for (LLResultTypes.FiducialResult fr : fiducials) {
            if (ids.length() > 0) ids.append(",");
            ids.append(fr.getFiducialId());
        }
        lastVisibleTagIds = ids.toString();

        // Don't process if data is stale (already noted above)
        if (lastStalenessMs > STALE_RESULT_MS) {
            return;
        }

        // Find our target tag (or the closest if config says -1)
        LLResultTypes.FiducialResult target = null;
        double closest = Double.MAX_VALUE;
        int targetId = allianceConfig.aprilTagId;

        for (LLResultTypes.FiducialResult fr : fiducials) {
            Pose3D tagPose = fr.getRobotPoseTargetSpace();
            if (tagPose == null) continue;

            double x = tagPose.getPosition().x * METERS_TO_INCHES;
            double y = tagPose.getPosition().y * METERS_TO_INCHES;
            double z = tagPose.getPosition().z * METERS_TO_INCHES;
            double dist = Math.sqrt(x * x + y * y + z * z);

            if (targetId == -1) {
                if (dist < closest) {
                    closest = dist;
                    target = fr;
                    distanceInches = dist;
                }
            } else if (fr.getFiducialId() == targetId) {
                target = fr;
                distanceInches = dist;
                break;
            }
        }

        if (target != null && distanceInches > 0) {
            hasTarget = true;
            txDegrees = target.getTargetXDegrees();
            lastSkipReason = "OK";
        } else {
            // Tags were visible but not our target tag
            lastSkipReason = "target tag " + targetId + " not in visible set [" + lastVisibleTagIds + "]";
        }
    }

    // -------------------------------------------------------------------------
    // Accessors — read from cache, do not re-poll
    // -------------------------------------------------------------------------

    public boolean isConnected()            { return connected; }
    public String getInitErrorMessage()     { return initErrorMessage; }
    public boolean hasTarget()              { return hasTarget; }
    public double getDistanceInches()       { return distanceInches; }
    public double getTxDegrees()            { return txDegrees; }
    public LLResult getLastResult()         { return lastResult; }
    public List<LLResultTypes.FiducialResult> getLastFiducials() { return lastFiducials; }

    /** Pipeline index from this alliance config (what we configured). */
    public int getConfiguredPipeline()       { return allianceConfig.limelightPipeline; }

    /** Tag ID this alliance is looking for. */
    public int getTargetTagId()              { return allianceConfig.aprilTagId; }

    // -------------------------------------------------------------------------
    // Diagnostics — surface these on telemetry when debugging tag detection
    // -------------------------------------------------------------------------

    /** Most recent staleness in ms (-1 if no result). */
    public long getLastStalenessMs()         { return lastStalenessMs; }

    /** Was the most recent result marked valid by the SDK? */
    public boolean isLastResultValid()       { return lastResultValid; }

    /** How many tags of any kind did the camera see this loop? */
    public int getVisibleTagCount()          { return lastVisibleTagCount; }

    /** Comma-separated list of every tag ID currently visible. */
    public String getVisibleTagIds()         { return lastVisibleTagIds; }

    /**
     * Human-readable string explaining why hasTarget might be false this
     * frame. Useful for surfacing to telemetry during debugging.
     */
    public String getLastSkipReason()        { return lastSkipReason; }

    /**
     * Convenience: power level for the current target distance using the
     * alliance-specific lookup table. Returns a fallback if no target.
     */
    public double getInterpolatedPower(double fallbackIfNoTarget) {
        if (!hasTarget) return fallbackIfNoTarget;
        return DistanceTable.interpolatePower(distanceInches, allianceConfig.powerPresets);
    }
}
