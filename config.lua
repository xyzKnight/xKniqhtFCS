-- config.lua

-- IMPORTS
local u = require("math_utils_v6")
local vec3 = u.Vector3
local c = {}

c.solver = {}

-- cannon config
c.solver.cannon = {
    pos        = vec3.new(0, 0, 0),
    barrel_len = 6
}

-- solver projectile config rotary
c.solver.projectile = {
    muzzle_speed         = 9,
    gravity              = 0.025,
    drag_multiplier      = 0.99,
    proximity_detonation = 0
}

-- solver projectile config medium
--c.solver.projectile = {
--    muzzle_speed         = 17,
--    gravity              = 0.04,
--    drag_multiplier      = 0.99,
--    proximity_detonation = 0
--}

-- predictor config
c.solver.predictor = {

    dt = 0.05,

    -- "dumb"     --> Constant Acceleration (CA)
    -- "aircraft" --> CTRA numerically integrated
    mode = "dumb",

    -- true  --> use omega + quaternion from radar peripheral
    -- false --> derive turn rate from smoothed velocity x acceleration
    -- only used with plane mode: true
    use_radar_rotation = false,

    -- physical constraints
    max_speed = 1000.0,
    max_accel = 20.0,
    min_speed = 1.0,

    -- velocity + acceleration EMA smoothing (same alpha for both, keeps them consistent)
    accel_alpha = 0.3,
    max_sim_time = 10,

}

-- solver system constants
c.solver.sys = {
    dt                   = 0.05,
    converge_iterations  = 7,
    converge_sensitivity = 0.4,
    converge_max_step    = math.huge,
    converge_tolerance   = 0.1,   -- stop iterating early if miss_dist drops below this (blocks)
    converge_reset_miss  = 50,    -- invalidate warm-start aim vec if last miss_dist exceeded this
    max_sim_time         = 10,
    min_pitch            = 0
}

-- peripheral config
c.peripheral = {
    RSC_TYPE          = "Create_RotationSpeedController",
    BLOCK_READER_TYPE = "blockReader",
    RADAR_TYPE        = "sp_radar",

    PITCH_RSC_INDEX = 2,
    YAW_RSC_INDEX   = 1,

    PITCH_RSC_SIGN  = -1,
    YAW_RSC_SIGN    = 1,

    RADAR_SCAN_RANGE = 10000
}

-- controller pipeline
c.controller = {
    DT                 = 0.05,
    D_READ             = 1,
    D_WRITE            = 2,
    D_TOTAL            = 3,
    D_TARGET_ALIGNMENT = 1,
    RSC_TO_CANNON      = 8,
    MAX_RSC_RPM        = 256
}

return c
