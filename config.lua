-- config.lua

-- IMPORTS
local u = require("math_utils_v6")
local vec3 = u.Vector3
local c = {}

c.solver = {}

-- cannon config
c.solver.cannon = {
    pos    = vec3.new(0,0,0),
    barrel_len   = 6
}


-- solver projectile config rotary
c.solver.projectile = {
    muzzle_speed     = 9,       
    gravity          = 0.025,
    drag_multiplier  = 0.99,
    proximity_detonation = 0
}



-- solver projectile config medium
--c.solver.projectile = {
    --muzzle_speed     = 17,       
    --gravity          = 0.04,
    --drag_multiplier  = 0.99,
    --proximity_detonation = 0
--}


-- predictor config
c.solver.predictor = {

    dt = 0.05,

    -- physical constraints
    max_speed = 75.0,
    max_accel = 20.0,   
    min_speed = 1.0,    
                     

    -- IMM weight smoothing
    model_weight_alpha = 0.1,

    -- acceleration smoothing
    accel_alpha = 0.08,

    -- model bias
    -- aircraft live in banked turns --> CTRA
    -- constant acceleration target --> CA
    bias = {
        cv   = 0.0,
        ca   = 1.0,
        ctra = 0.0,
    },

    -- error sensitivity
    error_gain = 1.0,

    -- turn detection threshold
    omega_small = 1e-5,

    -- stability clamp
    min_weight = 0.01,

}

-- solver system constants
c.solver.sys = {
    dt                   = 0.05,
    converge_iterations  = 5,
    converge_sensitivity = 0.4,
    converge_max_step    = math.huge,
    max_sim_time         = 10,
    min_pitch            = 0
}

-- peripheral config
c.peripheral = {
    -- replace with names given by command <peripherals>
    RSC_TYPE                = "Create_RotationSpeedController",
    BLOCK_READER_TYPE       = "blockReader",
    RADAR_TYPE              = "sp_radar",

    -- the index in the table of all connected speed controllers that maps to pitch / yaw
    PITCH_RSC_INDEX = 1,
    YAW_RSC_INDEX   = 2,

    -- whether or not to invert the RPM output
    PITCH_RSC_SIGN  = -1,
    YAW_RSC_SIGN    = 1,

    RADAR_SCAN_RANGE = 2000
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
