local u = require("math_utils_v6")
local vec3 = u.Vector3
local quat = u.Quaternion

local SMALL = 1e-6

local predictor = {}
predictor.__index = predictor

function predictor:ctra(t)
    local v0 = self.target_state.velo
    local x0 = self.target_state.pos
    local a = self.target_state.accel

    local speed_sq = v0:dot(v0)
    local speed = math.sqrt(speed_sq)
    if speed < SMALL then return x0 end
    local v_hat = v0 / speed

    -- Parallel acceleration (changes speed)
    local a_parallel = a:dot(v_hat)

    -- Turn rate
    local omega
    if speed_sq < SMALL then omega = vec3.new(0,0,0)
    else omega = v0:cross(a) / speed_sq end
    local omega_mag = omega:length()

    -- Straight-line fallback
    if omega_mag < SMALL then return x0 + v0 * t + v_hat * (0.5 * a_parallel * t * t) end

    local min_speed = self.c.min_speed or 1.0
    local max_omega = self.c.max_accel / math.max(speed, min_speed)
    if omega_mag > max_omega then
        omega = omega:scale(max_omega / omega_mag)
        omega_mag = max_omega
    end

    -- Closed-form circular motion
    local theta = omega_mag * t
    local sin_term = math.sin(theta) / omega_mag
    local cos_term = (1 - math.cos(theta)) / (omega_mag * omega_mag)

    -- Exact circular displacement
    local term1 = v0 * sin_term
    local term2 = omega:cross(v0) * cos_term
    local x_circular = x0 + term1 + term2

    local min_speed = self.c.min_speed or 1.0
    local new_speed = u.clamp(speed + a_parallel * t, min_speed, self.c.max_speed)
    local scale = new_speed / speed 
    local x_final = x0 + (x_circular - x0) * scale
    return x_final
end

function predictor:cv(t)
    return self.target_state.pos + self.target_state.velo * t
end

function predictor:ca(t)
    return self.target_state.pos + self.target_state.velo * t + self.target_state.accel * (0.5 * t * t)
end

function predictor:update_model_weights(target_data)
    local e_cv, e_ca, e_ctra = self:get_model_errors(target_data)

    local function likelihood(e)
        return math.exp(-e * e * 5)
    end

    local l_cv   = likelihood(e_cv) * self.c.bias.cv
    local l_ca   = likelihood(e_ca) * self.c.bias.ca
    local l_ctra = likelihood(e_ctra) * self.c.bias.ctra

    local sum = l_cv + l_ca + l_ctra + SMALL

    l_cv   = l_cv / sum
    l_ca   = l_ca / sum
    l_ctra = l_ctra / sum

    local alpha = self.c.model_weight_alpha  -- smoothing factor

    self.weights.cv   = u.lerp(self.weights.cv,l_cv,alpha)
    self.weights.ca   = u.lerp(self.weights.ca,l_ca,alpha)
    self.weights.ctra = u.lerp(self.weights.ctra,l_ctra,alpha)
end

function predictor:update_target_state(target_data)
    self.live = true
    self.target_state.pos = target_data.pos
    self.target_state.prev_velo = self.target_state.velo
    self.target_state.velo = target_data.velo

    local accel = (self.target_state.velo - self.target_state.prev_velo) / self.c.dt
    self.target_state.accel = u.lerp(self.target_state.prev_accel, accel, self.c.accel_alpha)
    self.target_state.prev_accel = self.target_state.accel
end

function predictor:getState()
    return self.target_state
end

function predictor:get_model_errors(target_data)
    local p_cv =   self:cv(self.c.dt)
    local p_ca =   self:ca(self.c.dt)
    local p_ctra = self:ctra(self.c.dt)

    local e_cv =   (target_data.pos - p_cv):length()
    local e_ca =   (target_data.pos - p_ca):length()
    local e_ctra = (target_data.pos - p_ctra):length()
    
    return e_cv, e_ca, e_ctra
end

function predictor:update(target_data)
    self:update_model_weights(target_data)
    self:update_target_state(target_data)
end

function predictor:predictIMM(t)
    local p_cv   = self:cv(t)
    local p_ca   = self:ca(t)
    local p_ctra = self:ctra(t)

    return 
        p_cv   * self.weights.cv +
        p_ca   * self.weights.ca +
        p_ctra * self.weights.ctra
end

function predictor:predictCTRA(t)
    return self:ctra(t)
end

function predictor.new(config)
    local self = setmetatable({},predictor)
    self.c = config
    self.target_state = {
        live = false,
        pos = vec3.new(0,0,0),
        velo = vec3.new(0,0,0),
        prev_velo = vec3.new(0,0,0),
        accel = vec3.new(0,0,0),
        prev_accel = vec3.new(0,0,0)
    }

    self.weights = {
        cv = 0.33,
        ca = 0.33,
        ctra = 0.34
    }

    return self
end

return predictor