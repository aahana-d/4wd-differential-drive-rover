-- SQLite schema for the rover's host-side telemetry/validation database
-- populated by basestation/telemetry_logger.cpp while bench/field testing the rover, then queried to validate PID tuning, the voltage-compensation feature, and the comm-loss failsafe.

PRAGMA foreign_keys = ON;

-- one row per test run (a bench session, a field trial, an A/B compensation-on-vs-off comparison, etc).
CREATE TABLE IF NOT EXISTS sessions (
    session_id      INTEGER PRIMARY KEY AUTOINCREMENT,
    started_at_utc  TEXT NOT NULL DEFAULT (STRFTIME('%Y-%m-%dT%H:%M:%fZ', 'now')),
    notes           TEXT,
    comp_enabled    INTEGER NOT NULL DEFAULT 1 CHECK (comp_enabled IN (0,1))
);

-- one row per TELEMETRY_DATA packet received from the rover.
CREATE TABLE IF NOT EXISTS telemetry_samples (
    sample_id       INTEGER PRIMARY KEY AUTOINCREMENT,
    session_id      INTEGER NOT NULL REFERENCES sessions(session_id),
    rover_millis    INTEGER NOT NULL,           -- rover's own millis() timestamp, from the packet
    received_at_utc TEXT NOT NULL DEFAULT (STRFTIME('%Y-%m-%dT%H:%M:%fZ', 'now')),
    battery_voltage REAL NOT NULL,
    comp_factor     REAL NOT NULL,
    fl_target_radps REAL NOT NULL, fl_actual_radps REAL NOT NULL, fl_cmd REAL NOT NULL,
    rl_target_radps REAL NOT NULL, rl_actual_radps REAL NOT NULL, rl_cmd REAL NOT NULL,
    fr_target_radps REAL NOT NULL, fr_actual_radps REAL NOT NULL, fr_cmd REAL NOT NULL,
    rr_target_radps REAL NOT NULL, rr_actual_radps REAL NOT NULL, rr_cmd REAL NOT NULL
);
CREATE INDEX IF NOT EXISTS idx_telemetry_session_time ON telemetry_samples(session_id, rover_millis);

-- one row per command frame sent to the rover, for protocol / failsafe validation.
CREATE TABLE IF NOT EXISTS commands_sent (
    command_id      INTEGER PRIMARY KEY AUTOINCREMENT,
    session_id      INTEGER NOT NULL REFERENCES sessions(session_id),
    sent_at_utc     TEXT NOT NULL DEFAULT (STRFTIME('%Y-%m-%dT%H:%M:%fZ', 'now')),
    cmd_id          INTEGER NOT NULL,
    payload_hex     TEXT,
    was_acked       INTEGER CHECK (was_acked IN (0,1))
);

-- one row every time the logger observes a NACK or a CRC error, used to validate protocol robustness and the comm-loss failsafe.
CREATE TABLE IF NOT EXISTS fault_events (
    event_id        INTEGER PRIMARY KEY AUTOINCREMENT,
    session_id      INTEGER NOT NULL REFERENCES sessions(session_id),
    occurred_at_utc TEXT NOT NULL DEFAULT (STRFTIME('%Y-%m-%dT%H:%M:%fZ', 'now')),
    event_type      TEXT NOT NULL, -- e.g. 'NACK', 'CRC_ERROR'
    detail          TEXT
);

-- ---------------------------------------------------------------------
-- Validation views used during bench testing
-- ---------------------------------------------------------------------

-- per-wheel RMS speed-tracking error for each session: the core PID tuning/validation metric
CREATE VIEW IF NOT EXISTS v_wheel_rms_error AS
SELECT
    session_id,
    SQRT(AVG((fl_target_radps - fl_actual_radps) * (fl_target_radps - fl_actual_radps))) AS fl_rms_error,
    SQRT(AVG((rl_target_radps - rl_actual_radps) * (rl_target_radps - rl_actual_radps))) AS rl_rms_error,
    SQRT(AVG((fr_target_radps - fr_actual_radps) * (fr_target_radps - fr_actual_radps))) AS fr_rms_error,
    SQRT(AVG((rr_target_radps - rr_actual_radps) * (rr_target_radps - rr_actual_radps))) AS rr_rms_error
FROM telemetry_samples
GROUP BY session_id;

-- speed error bucketed by battery voltage, split by whether voltage compensation was enabled for that session.
-- comparing avg_abs_error_front_wheels across comp_enabled
CREATE VIEW IF NOT EXISTS v_voltage_vs_error AS
SELECT
    s.comp_enabled,
    ROUND(t.battery_voltage, 1) AS voltage_bucket,
    AVG(ABS(t.fl_target_radps - t.fl_actual_radps) + ABS(t.fr_target_radps - t.fr_actual_radps)) / 2.0
        AS avg_abs_error_front_wheels,
    COUNT(*) AS sample_count
FROM telemetry_samples t
JOIN sessions s ON s.session_id = t.session_id
GROUP BY s.comp_enabled, voltage_bucket
ORDER BY s.comp_enabled, voltage_bucket DESC;

-- Gaps between consecutive telemetry samples (by rover_millis) larger than 3x the expected 200ms telemetry period - a proxy for comm loss / failsafe engagement during a session.
CREATE VIEW IF NOT EXISTS v_telemetry_gaps AS
SELECT * FROM (
    SELECT
        session_id,
        rover_millis,
        rover_millis - LAG(rover_millis) OVER (PARTITION BY session_id ORDER BY rover_millis) AS gap_ms
    FROM telemetry_samples
)
-- SQLite has no QUALIFY clause, so the filter is applied via an outer WHERE over a subquery.
WHERE gap_ms > 600;
