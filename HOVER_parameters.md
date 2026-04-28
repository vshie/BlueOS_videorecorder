# HAUV `HOVER_*` Parameters

Parameters added by `HAUV.lua` on the `hauv-v2` branch.


| Parameter           | Default | Units | Description                                                               |
| ------------------- | ------- | ----- | ------------------------------------------------------------------------- |
| `HOVER_DELAY_S`     | 30      | s     | Countdown before dive starts                                              |
| `HOVER_LIGHT_D`     | 200.0   | m     | Depth at which lights turn on                                             |
| `HOVER_HOVER_M`     | 2.0     | min   | Minutes to hover at target depth                                          |
| `HOVER_SURF_D`      | 2.0     | m     | Surface depth threshold                                                   |
| `HOVER_MAX_AH`      | 12.0    | Ah    | Max battery amp-hours consumed before abort                               |
| `HOVER_MIN_V`       | 13.0    | V     | Min battery voltage before abort                                          |
| `HOVER_REC_DEPTH`   | 50.0    | m     | Depth at which video recording starts                                     |
| `HOVER_H_OFF`       | 3       | m     | Hover this far above target depth (or actual max depth if bottom reached) |
| `HOVER_T_DEPTH`     | 410     | m     | Target/max depth; hover above if reached                                  |
| `HOVER_D_THRTL`     | 1750    | PWM   | Descent throttle (PWM µs)                                                 |
| `HOVER_A_THRTL`     | 1460    | PWM   | Ascent throttle (PWM µs), used when climb rate is insufficient            |
| `HOVER_SIM_MODE`    | 0       | —     | 0 = normal, 1 = simulation (SITL)                                         |
| `HOVER_MAX_D_RATE`  | 1.0     | m/s   | Maximum descent rate                                                      |
| `HOVER_MIN_A_RATE`  | 0.7     | m/s   | Minimum ascent rate                                                       |
| `HOVER_WS_EN`       | 1       | —     | Water sampling enable: 0 = off, non-zero = on                             |
| `HOVER_THRTL_STEP`  | 2       | PWM   | Throttle adjustment step size                                             |
| `HOVER_T_BUFFER`    | 1.3     | ×     | Buffer multiplier for dive/ascent timeout calculation                     |
| `HOVER_SURF_DEPTH`  | 0.65    | m     | Depth threshold below which surface-holding throttle increases            |
| `HOVER_WS_INTERVAL` | 5.0     | m     | Water sampling interval (depth step), used only if `HOVER_WS_EN` > 0      |
| `HOVER_WS_HTIME`    | 0.5     | min   | Hover time at each water sampling stop                                    |
