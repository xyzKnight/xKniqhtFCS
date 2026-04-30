-- predictor.lua
-- generative AI partially used for this file

local u    = require("math_utils_v6")
local vec3 = u.Vector3
local quat = u.Quaternion

local SMALL = 1e-9

local predictor = {}
predictor.__index = predictor

function predictor:_buildTable()
    local tbl   = {}
    local dt    = self.c.dt
    local steps = math.ceil(self.c.max_sim_time / dt)

    if self.c.mode == "dumb" then
        local x0 = self.target_state.pos
        local v0 = self.target_state.smooth_velo
        local a  = self.target_state.accel
        for i = 0, steps do
            local t = i * dt
            tbl[i] = x0 + v0 * t + a * (0.5 * t * t)
        end

    elseif self.c.mode == "aircraft" then
        local pos  = self.target_state.pos
        local velo = self.target_state.smooth_velo

        if self.c.use_radar_rotation then
            local orientation = self.target_state.orientation
            local omega_vec   = self.target_state.omega
            local omega_mag   = omega_vec:length()

            local speed = velo:length()
            local v_hat = speed > SMALL and velo:scale(1 / speed) or vec3.new(1, 0, 0)
            local a_tan = self.target_state.accel:dot(v_hat)

            tbl[0] = pos

            for i = 1, steps do
                local dq
                if omega_mag > SMALL then
                    dq = quat.fromAxisAngle(omega_vec:scale(1 / omega_mag), omega_mag * dt)
                else
                    dq = quat.identity()
                end
                orientation = (orientation * dq):norm()
                velo        = velo:rotateByQuat(dq)

                local spd_now   = velo:length()
                local new_speed = u.clamp(spd_now + a_tan * dt, self.c.min_speed, self.c.max_speed)
                if spd_now > SMALL then velo = velo:scale(new_speed / spd_now) end

                pos    = pos + velo * dt
                tbl[i] = pos
            end

        else
            local speed_sq = velo:dot(velo)
            local speed    = math.sqrt(speed_sq)

            local omega_vec
            if speed_sq > SMALL then
                omega_vec = velo:cross(self.target_state.accel):scale(1 / speed_sq)
            else
                omega_vec = vec3.new(0, 0, 0)
            end

            local omega_mag = omega_vec:length()
            local max_omega = self.c.max_accel / math.max(speed, self.c.min_speed)
            if omega_mag > max_omega then
                omega_vec = omega_vec:scale(max_omega / omega_mag)
                omega_mag = max_omega
            end

            local v_hat = speed > SMALL and velo:scale(1 / speed) or vec3.new(1, 0, 0)
            local a_tan = self.target_state.accel:dot(v_hat)

            tbl[0] = pos

            for i = 1, steps do
                local dq
                if omega_mag > SMALL then
                    dq = quat.fromAxisAngle(omega_vec:scale(1 / omega_mag), omega_mag * dt)
                else
                    dq = quat.identity()
                end

                velo = velo:rotateByQuat(dq)

                local spd_now   = velo:length()
                local new_speed = u.clamp(spd_now + a_tan * dt, self.c.min_speed, self.c.max_speed)
                if spd_now > SMALL then velo = velo:scale(new_speed / spd_now) end

                pos    = pos + velo * dt
                tbl[i] = pos
            end
        end
    end

    self._tick_table  = tbl
    self._table_steps = steps
end

function predictor:predict(t)
    local dt      = self.c.dt
    local i_float = t / dt
    local i_lo    = math.floor(i_float)
    local frac    = i_float - i_lo

    local tbl   = self._tick_table
    local steps = self._table_steps

    if i_lo >= steps then return tbl[steps] end
    if i_lo < 0      then return tbl[0]     end

    local p_lo = tbl[i_lo]
    local p_hi = tbl[math.min(i_lo + 1, steps)]
    return p_lo + (p_hi - p_lo) * frac
end

function predictor:update(target_data)
    local alpha     = self.c.accel_alpha
    local prev_velo = self.target_state.smooth_velo

    -- smooth velocity first, then derive acceleration from the smoothed delta so
    -- omega = smooth_velo x accel uses a consistent kinematic pair
    local smooth_velo = prev_velo + (target_data.velo - prev_velo) * alpha
    local raw_accel   = (smooth_velo - prev_velo):scale(1 / self.c.dt)
    local accel       = self.target_state.accel + (raw_accel - self.target_state.accel) * alpha

    self.target_state.pos         = target_data.pos
    self.target_state.velo        = target_data.velo
    self.target_state.smooth_velo = smooth_velo
    self.target_state.accel       = accel
    self.target_state.read_time   = target_data.read_time

    if self.c.mode == "aircraft" and self.c.use_radar_rotation then
        if target_data.omega and target_data.orientation then
            self.target_state.omega       = target_data.omega
            self.target_state.orientation = target_data.orientation
        end
    end

    self:_buildTable()
end

function predictor:getState()
    return self.target_state
end

function predictor:getTickTable()
    return self._tick_table, self._table_steps, self.target_state.read_time
end

function predictor.new(config)
    local self = setmetatable({}, predictor)
    self.c = config

    self.target_state = {
        pos         = vec3.new(0, 0, 0),
        velo        = vec3.new(0, 0, 0),
        smooth_velo = vec3.new(0, 0, 0),
        accel       = vec3.new(0, 0, 0),
        omega       = vec3.new(0, 0, 0),
        orientation = quat.identity(),
        read_time   = 0,
    }

    self._tick_table  = {}
    self._table_steps = 0
    self:_buildTable()

    return self
end

return predictor
