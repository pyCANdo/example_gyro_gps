# Position prediction

## Model 1: simple GNSS and IMU prediction

Model 1 predicts latitude and longitude between GNSS fixes. A valid GNSS
position and motion update initializes or corrects the model's position, speed,
and heading. At each IMU sample, it advances that state using the elapsed time.
When calibrated vehicle dynamics are valid, it also uses longitudinal and
lateral acceleration plus yaw rate. Otherwise, it continues with its current
velocity. This is an experimental short-term prediction; drift grows between
GNSS corrections.

Prediction is **disabled at boot**. These are standard Classic CAN data frames:

| CAN ID | Message | Payload |
| --- | --- | --- |
| `0x302` | `Position_ModelSelect` | Exactly 1 byte: `00` disables prediction, `01` selects Model 1, `02` selects Model 2. Other values are ignored. |
| `0x303` | `Position_ModelParameters` | Exactly 8 bytes: unsigned `Parameter0` through `Parameter7`, one byte each. A frame replaces all eight values. |
| `0x204` | `Predicted_Position` | Output: signed 32-bit little-endian `Latitude` and `Longitude`, each scaled by `0.0000001 deg`. The layout matches `0x200 GNSS_Position`. |

Model 1 uses `Parameter0` to set the maximum age of the last GNSS correction.
`00` keeps the default **3000 ms** limit; values `01`–`FF` mean 100–25,500 ms
in 100 ms steps. `Parameter1`–`Parameter7` are available to the selected model
but have no effect in Model 1 yet. All parameters start at zero on boot and
remain stored when switching models. Changing the selected model resets its
prediction state. Model 1 then initializes from the latest available valid GNSS
position and motion fix on an IMU update.

While Model 1 has a valid prediction, the board sends `0x204` after new IMU
samples at up to 100 Hz. No predicted-position frame is sent before a valid
fix, after the correction-age limit, or while Model 0 is selected. The output
frame has no validity bit; firmware publishes it only while its prediction is
valid.

Example SocketCAN commands:

```sh
cansend can0 303#0A00000000000000  # 1000 ms correction-age limit
cansend can0 302#01                # enable Model 1
cansend can0 302#00                # disable prediction
```

## Model 2: IMU prediction with GNSS residual feedback

Model 2 (`cansend can0 302#02`) predicts from **calibrated IMU acceleration and
yaw rate on each sensor step**. GNSS position and motion fixes correct the
state. Before each correction, it measures the difference between the new
GNSS position and the IMU prediction. A short history of these differences is
used to fit a bounded velocity correction by minimizing a quadratic cost. This
is a moving-horizon-inspired *estimator*, not an MPC controller: it estimates
position and has no vehicle control output.

For each north/east component, a recent residual window produces a target
velocity correction `t_i = previous_correction + residual_i / window_seconds`.
Model 2 minimizes
`J(b) = residual_weight × Σ (b - t_i)² / (i + 1) +`
`2(b - previous_correction)² + penalty × b²`.
The result is limited to 5 m/s. The IMU supplies the trajectory between fixes;
the cost function only adjusts its drift from observed GNSS errors.

Model 2 never falls back to predicting solely from old GNSS velocity. It waits
for calibrated IMU dynamics, a valid yaw rate, and a new valid GNSS fix after
selection. A timing fault or saturated dynamics resets it and requires another
new fix. Model 2 uses the same `0x204` latitude and longitude output as Model 1.

The shared eight-byte `0x303` message has these meanings when Model 2 is
selected. Send all eight bytes on every parameter update. A zero byte selects
the listed default; settings remain in memory when switching models.

| Byte | Tuning | Zero default |
| --- | --- | --- |
| `Parameter0` | Maximum age since GNSS correction, 100 ms per count | 3000 ms |
| `Parameter1` | GNSS fix latency compensation, 10 ms per count | 0 ms |
| `Parameter2` | Position correction gain, byte / 255 | 1.0 |
| `Parameter3` | Velocity correction gain, byte / 255 | 1.0 |
| `Parameter4` | Weight of GNSS residuals in the velocity-correction cost, byte / 32 | 1.0 |
| `Parameter5` | Number of recent residual windows, limited to 1–8 | 4 |
| `Parameter6` | Penalty on large velocity corrections, byte / 16 | 0.5 |
| `Parameter7` | Largest single residual used for learning, metres | 25 m |

`0x205 Position_Feedback` reports the last GNSS-minus-prediction north/east
residual in metres and the fitted north/east velocity correction in m/s. All
four signals are signed 16-bit values scaled by 0.01. It is sent at 10 Hz
while Model 2 is initialized, repeating the latest values until the next fix.
This lets a CAN log show whether residuals shrink after tuning.

Start with `cansend can0 303#0000000000000000`, then select Model 2. If logs
show a repeatable GNSS fix delay, set `Parameter1` to the measured delay in
10 ms steps. For example, `cansend can0 303#000A000000000000` uses 100 ms.
Increasing `Parameter4` makes the model react more strongly to residuals;
increasing `Parameter6` restrains that correction. Tune against recorded runs:
GNSS timing error, multipath, and IMU calibration error can all affect the
residual, so a smaller residual on one fix is not proof of better positioning.
