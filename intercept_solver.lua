-- intercept_solver.lua  (Fire Control System Numerical Solver)

-- IMPORTS
local u    = require("math_utils_v6")
local vec3 = u.Vector3
local quat = u.Quaternion

local Predictor = require("predictor")

-- CONSTANTS
local projectile_y_break_dist = 50  
local small                   = 1e-6

local solver = {}
solver.__index = solver

-- simulates projectile and target trajectory to find closest pass
function solver:simulateProjectile(aimVec, t_start)
    local function getDisplacement(v, a, dt) return v:scale(dt) + a:scale(dt * dt * 0.5) end
    local t           = t_start
    local cannon_pos  = self.c.cannon.pos
    local gravity_vec = vec3.new(0, self.c.projectile.gravity, 0)
    local drag        = self.c.projectile.drag_multiplier
    local max_t       = self.c.sys.max_sim_time
    local sim_dt      = self.c.sys.dt

    local sim_proj_pos  = cannon_pos + aimVec:scale(self.c.cannon.barrel_len)
    local sim_proj_velo = aimVec:scale(self.c.projectile.muzzle_speed)

    local sim_target_pos = self.target.pos + getDisplacement(self.target.velo, self.target.accel, t)
    local target_velo_at_t = self.target.velo + self.target.accel:scale(t)

    local min_miss_dist  = math.huge
    local min_proj_pos   = sim_proj_pos
    local min_target_pos = sim_target_pos
    local min_time       = t

    while t <= max_t do
        local prev_proj_pos   = sim_proj_pos
        local prev_target_pos = sim_target_pos

        t = t + sim_dt

        sim_proj_velo = (sim_proj_velo - gravity_vec):scale(drag)
        sim_proj_pos  = sim_proj_pos + sim_proj_velo

        sim_target_pos = self.predictor:predictCTRA(t)

        local target_mid = (prev_target_pos + sim_target_pos):scale(0.5)
        local miss_dist, closest_proj_pos = vec3.segmentPointDistance(prev_proj_pos, sim_proj_pos, target_mid)

        if miss_dist <= min_miss_dist then
            min_miss_dist  = miss_dist
            min_proj_pos   = closest_proj_pos
            min_target_pos = target_mid
            min_time       = t
        end

        if sim_proj_pos.y < sim_target_pos.y - projectile_y_break_dist
        and sim_proj_velo.y < 0 then
            break
        end
    end

    local intercept_mid = (min_proj_pos + min_target_pos):scale(0.5)
    local miss_dir      = (intercept_mid - cannon_pos):norm()

    return {
        proj_pos   = min_proj_pos,
        target_pos = min_target_pos,
        miss_dist  = min_miss_dist,
        miss_dir   = miss_dir,
        time       = min_time,
    }
end

-- Iteratively rotates aimVec toward the intercept point.
function solver:convergeAim(t_start)
    local aimVec
    if self._last_aim_vec then
        local target_jump = (self.target.pos - self._last_target_pos):length()
        if target_jump < 200 then
            aimVec = self._last_aim_vec
        end
    end

    if not aimVec then aimVec = (self.target.pos - self.c.cannon.pos):norm() end

    local cpass = self:simulateProjectile(aimVec, t_start)

    for i = 1, self.c.sys.converge_iterations do
        local shot_dir   = (cpass.proj_pos   - self.c.cannon.pos):norm()
        local target_dir = cpass.miss_dir

        local axis     = shot_dir:cross(target_dir)
        local axis_len = axis:length()
        if axis_len < small then break end

        axis = axis:norm()
        local dot   = u.clamp(shot_dir:dot(target_dir), -1, 1)
        local angle = math.acos(dot)

        local step = u.clamp(
            self.c.sys.converge_sensitivity * angle,
            -self.c.sys.converge_max_step,
            self.c.sys.converge_max_step
        )

        local q    = quat.fromAxisAngle(axis, step)
        aimVec     = aimVec:rotateByQuat(q):norm()
        cpass      = self:simulateProjectile(aimVec, t_start)
    end

    self._last_aim_vec    = aimVec
    self._last_target_pos = self.target.pos

    local pitch, yaw = aimVec:aimAt()
    return {
        pitch      = u.clamp(math.deg(pitch), self.c.sys.min_pitch, math.huge),
        yaw        = math.deg(yaw),
        miss_dist  = cpass.miss_dist,
        time       = cpass.time,
        proj_pos   = cpass.proj_pos,
        target_pos = cpass.target_pos,
    }
end

-- returns pitch and yaw to aim to hit a given target 
function solver:solve(target_data, t_start)
    if target_data.pos == nil or target_data.velo == nil then return 0, 0 end

    self.predictor:update(target_data)
    local state     = self.predictor:getState()
    self.target     = state

    self.intercept  = self:convergeAim(t_start)
    print("tgo:",self.intercept.time)
    return self.intercept.pitch, self.intercept.yaw
end

function solver.new(config)
    local self     = setmetatable({}, solver)
    self.c         = config
    self.predictor = Predictor.new(self.c.predictor)

    self.target = {
        pos   = vec3.new(0, 0, 0),
        velo  = vec3.new(0, 0, 0),
        accel = vec3.new(0, 0, 0),
    }

    self.cannon    = { pitch = 0, yaw = 0, pos = vec3.new(0, 0, 0) }
    self.intercept = {
        pitch      = 0,
        yaw        = 0,
        miss_dist  = 0,
        time       = 0,
        proj_pos   = vec3.new(0, 0, 0),
        target_pos = vec3.new(0, 0, 0),
    }

    self._last_aim_vec    = nil
    self._last_target_pos = vec3.new(0, 0, 0)
    return self
end

return solver