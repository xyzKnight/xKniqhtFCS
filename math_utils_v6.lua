-- math_utils_v6.lua
local M = {}

-- Scalar Utilities

--- Clamps x to the range [min, max].
function M.clamp(x, min, max)
    return math.max(min, math.min(max, x))
end

--- Wraps an angle in radians into the range (-pi, pi].
function M.wrapAngle(theta)
    return (theta + math.pi) % (2 * math.pi) - math.pi
end

function M.wrap_angle_deg(delta)
    while delta >  180 do delta = delta - 360 end
    while delta < -180 do delta = delta + 360 end
    return delta
end

--- Truncates n to sf significant figures after the decimal point.
--- Default sf is 3.
function M.trun(n, sf)
    sf = sf or 3
    local f = 10 ^ sf
    return math.floor(n * f) / f
end

--- Linear interpolation between scalars a and b.
--- alpha = 0 returns a, alpha = 1 returns b.
function M.lerp(a, b, alpha)
    return a * (1 - alpha) + b * alpha
end

--- Smoothly interpolates between a and b using a cubic Hermite curve (smoothstep).
--- t is clamped to [0, 1]. Useful for easing transitions.
function M.smoothstep(a, b, t)
    t = M.clamp(t, 0, 1)
    t = t * t * (3 - 2 * t)
    return a + (b - a) * t
end

--- Maps a value from one range [in_min, in_max] to another [out_min, out_max].
function M.map(value, in_min, in_max, out_min, out_max)
    return out_min + (value - in_min) * (out_max - out_min) / (in_max - in_min)
end

--- Rounds n to sf decimal places. Default sf is 0 (rounds to integer).
function M.round(n, sf)
    sf = sf or 0
    local f = 10 ^ sf
    return math.floor(n * f + 0.5) / f
end

--- Returns the sign of x: -1, 0, or 1.
function M.sign(x)
    if x > 0 then return 1
    elseif x < 0 then return -1
    else return 0
    end
end

--- Converts degrees to radians.
function M.toRad(deg)
    return deg * math.pi / 180
end

--- Converts radians to degrees.
function M.toDeg(rad)
    return rad * 180 / math.pi
end

-- Vector2

M.Vector2 = {}
local Vector2Methods = {}

--- Returns the length (magnitude) of the vector.
function Vector2Methods:length()
    return math.sqrt(self.x * self.x + self.y * self.y)
end

--- Returns the squared length. Useful for comparisons where you don't need
--- the actual distance (avoids the sqrt).
function Vector2Methods:lengthSq()
    return self.x * self.x + self.y * self.y
end

--- Returns a new vector scaled by n.
function Vector2Methods:scale(n)
    return M.Vector2.new(self.x * n, self.y * n)
end

--- Returns the dot product with vector b.
function Vector2Methods:dot(b)
    return self.x * b.x + self.y * b.y
end

--- Returns the unit vector. Returns (0, 0) if length is zero.
function Vector2Methods:norm()
    local len = self:length()
    if len == 0 then return M.Vector2.new(0, 0) end
    return self:scale(1 / len)
end

--- Returns a new vector with each component clamped to [min, max].
function Vector2Methods:clamp(min, max)
    return M.Vector2.new(
        M.clamp(self.x, min, max),
        M.clamp(self.y, min, max)
    )
end

--- Returns the vector rotated by angle radians (counterclockwise).
function Vector2Methods:rotate(angle)
    local c = math.cos(angle)
    local s = math.sin(angle)
    return M.Vector2.new(
        self.x * c - self.y * s,
        self.x * s + self.y * c
    )
end

--- Returns the angle of this vector from the positive x-axis, in radians.
--- Range: (-pi, pi].
function Vector2Methods:angle()
    return math.atan2(self.y, self.x)
end

--- Returns the angle between this vector and vector b, in radians.
--- Range: [0, pi].
function Vector2Methods:angleTo(b)
    local d = self:dot(b)
    local l = self:length() * b:length()
    if l == 0 then return 0 end
    return math.acos(M.clamp(d / l, -1, 1))
end

--- Returns the component of this vector projected onto b.
function Vector2Methods:project(b)
    local bLenSq = b:lengthSq()
    if bLenSq == 0 then return M.Vector2.new(0, 0) end
    return b:scale(self:dot(b) / bLenSq)
end

--- Returns this vector reflected about the normal n (n should be unit length).
function Vector2Methods:reflect(n)
    return self - n:scale(2 * self:dot(n))
end

--- Returns the perpendicular vector (rotated 90 degrees counterclockwise).
function Vector2Methods:perp()
    return M.Vector2.new(-self.y, self.x)
end

--- Linearly interpolates between this vector and b by alpha.
function Vector2Methods:lerp(b, alpha)
    return M.Vector2.new(
        M.lerp(self.x, b.x, alpha),
        M.lerp(self.y, b.y, alpha)
    )
end

--- Returns a formatted string representation.
function Vector2Methods:__tostring()
    return string.format("(%g, %g)", self.x, self.y)
end

-- Metamethods

local function vec2Add(a, b)
    return M.Vector2.new(a.x + b.x, a.y + b.y)
end

local function vec2Sub(a, b)
    return M.Vector2.new(a.x - b.x, a.y - b.y)
end

--- Scalar multiplication: vec * n or n * vec
local function vec2Mul(a, b)
    if type(a) == "number" then
        return M.Vector2.new(a * b.x, a * b.y)
    elseif type(b) == "number" then
        return M.Vector2.new(a.x * b, a.y * b)
    end
    error("Vector2 __mul: expected a scalar operand")
end

--- Scalar division: vec / n
local function vec2Div(a, b)
    if type(b) == "number" then
        if b == 0 then error("Vector2 __div: division by zero") end
        return M.Vector2.new(a.x / b, a.y / b)
    end
    error("Vector2 __div: expected a scalar divisor")
end

--- Unary negation: -vec
local function vec2Unm(a)
    return M.Vector2.new(-a.x, -a.y)
end

--- Equality check (component-wise).
local function vec2Eq(a, b)
    return a.x == b.x and a.y == b.y
end

local Vector2MT = {
    __index    = Vector2Methods,
    __add      = vec2Add,
    __sub      = vec2Sub,
    __mul      = vec2Mul,
    __div      = vec2Div,
    __unm      = vec2Unm,
    __eq       = vec2Eq,
    __tostring = Vector2Methods.__tostring,
}

--- Creates a new Vector2.
function M.Vector2.new(x, y)
    return setmetatable({ x = x, y = y }, Vector2MT)
end

-- Vector3

M.Vector3 = {}
local Vector3Methods = {}

--- Returns the length (magnitude) of the vector.
function Vector3Methods:length()
    return math.sqrt(self.x * self.x + self.y * self.y + self.z * self.z)
end

--- Returns the squared length. Useful for distance comparisons without sqrt.
function Vector3Methods:lengthSq()
    return self.x * self.x + self.y * self.y + self.z * self.z
end

--- Returns the horizontal length (xz plane, ignoring y).
--- Useful for ground-plane distance calculations.
function Vector3Methods:horizontalLength()
    return math.sqrt(self.x * self.x + self.z * self.z)
end

--- Returns the squared horizontal length.
function Vector3Methods:horizontalLengthSq()
    return self.x * self.x + self.z * self.z
end

--- Returns a new vector scaled by n.
function Vector3Methods:scale(n)
    return M.Vector3.new(self.x * n, self.y * n, self.z * n)
end

--- Returns the dot product with vector b.
function Vector3Methods:dot(b)
    return self.x * b.x + self.y * b.y + self.z * b.z
end

--- Returns the cross product with vector b.
--- The result is perpendicular to both self and b, with direction
--- following the right-hand rule.
function Vector3Methods:cross(b)
    return M.Vector3.new(
        self.y * b.z - self.z * b.y,
        self.z * b.x - self.x * b.z,
        self.x * b.y - self.y * b.x
    )
end

--- Returns the unit vector. Returns (0, 0, 0) if length is zero.
function Vector3Methods:norm()
    local len = self:length()
    if len == 0 then return M.Vector3.new(0, 0, 0) end
    return self:scale(1 / len)
end

--- Returns a new vector with each component clamped to [min, max].
function Vector3Methods:clamp(min, max)
    return M.Vector3.new(
        M.clamp(self.x, min, max),
        M.clamp(self.y, min, max),
        M.clamp(self.z, min, max)
    )
end

--- Returns pitch and yaw angles (radians) that point from the origin
function Vector3Methods:aimAt()
    local yaw   = math.atan2(-self.x, self.z)
    local pitch = math.atan2(self.y, math.sqrt(self.x ^ 2 + self.z ^ 2))
    return pitch, yaw
end

--- Returns pitch and yaw to aim from this vector's position toward point p.
function Vector3Methods:aimAtRel(p)
    return (p - self):aimAt()
end

--- Rotates this vector by quaternion q using the standard q * v * q^-1 formula.
function Vector3Methods:rotateByQuat(q)
    local vq  = M.Quaternion.new(0, self.x, self.y, self.z)
    local res = (q * vq) * q:conjugate()
    return M.Vector3.new(res.x, res.y, res.z)
end

--- Returns the angle between this vector and vector b, in radians.
--- Range: [0, pi].
function Vector3Methods:angleTo(b)
    local d = self:dot(b)
    local l = self:length() * b:length()
    if l == 0 then return 0 end
    return math.acos(M.clamp(d / l, -1, 1))
end

--- Returns the component of this vector projected onto b.
function Vector3Methods:project(b)
    local bLenSq = b:lengthSq()
    if bLenSq == 0 then return M.Vector3.new(0, 0, 0) end
    return b:scale(self:dot(b) / bLenSq)
end

--- Returns the component of this vector perpendicular to b
--- (i.e. self minus the projection onto b).
function Vector3Methods:reject(b)
    return self - self:project(b)
end

--- Returns this vector reflected about the normal n (n should be unit length).
function Vector3Methods:reflect(n)
    return self - n:scale(2 * self:dot(n))
end

--- Linearly interpolates between this vector and b by alpha.
function Vector3Methods:lerp(b, alpha)
    return M.Vector3.new(
        M.lerp(self.x, b.x, alpha),
        M.lerp(self.y, b.y, alpha),
        M.lerp(self.z, b.z, alpha)
    )
end

--- Returns a copy of this vector with the y component set to zero.
function Vector3Methods:horizontal()
    return M.Vector3.new(self.x, 0, self.z)
end

--- Returns a formatted string representation.
function Vector3Methods:__tostring()
    return string.format("(%g, %g, %g)", self.x, self.y, self.z)
end

-- finds the closest distance between a line and point
function M.Vector3.segmentPointDistance(p0, p1, t)
    local d = p1 - p0
    local l2 = d:dot(d)
    if l2 < 1e-8 then
        return (t - p0):length(), p0
    end
    local u = (t - p0):dot(d) / l2
    u = math.max(0, math.min(1, u))
    local closest = p0 + d * u
    return (t - closest):length(), closest
end

function M.Vector3.fromAngles(pitch,yaw)
    local sinp = math.sin(pitch)
    local cosp = math.cos(pitch)

    local siny = math.sin(yaw)
    local cosy = math.cos(yaw)

    return M.Vector3.new(cosp*siny,sinp,-cosp*cosy)
end

-- Metamethods

local function vec3Add(a, b)
    return M.Vector3.new(a.x + b.x, a.y + b.y, a.z + b.z)
end

local function vec3Sub(a, b)
    return M.Vector3.new(a.x - b.x, a.y - b.y, a.z - b.z)
end

--- Scalar multiplication: vec * n or n * vec
local function vec3Mul(a, b)
    if type(a) == "number" then
        return M.Vector3.new(a * b.x, a * b.y, a * b.z)
    elseif type(b) == "number" then
        return M.Vector3.new(a.x * b, a.y * b, a.z * b)
    end
    error("Vector3 __mul: expected a scalar operand")
end

--- Scalar division: vec / n
local function vec3Div(a, b)
    if type(b) == "number" then
        if b == 0 then error("Vector3 __div: division by zero") end
        return M.Vector3.new(a.x / b, a.y / b, a.z / b)
    end
    error("Vector3 __div: expected a scalar divisor")
end

--- Unary negation: -vec
local function vec3Unm(a)
    return M.Vector3.new(-a.x, -a.y, -a.z)
end

--- Equality check (component-wise).
local function vec3Eq(a, b)
    return a.x == b.x and a.y == b.y and a.z == b.z
end

local Vector3MT = {
    __index    = Vector3Methods,
    __add      = vec3Add,
    __sub      = vec3Sub,
    __mul      = vec3Mul,
    __div      = vec3Div,
    __unm      = vec3Unm,
    __eq       = vec3Eq,
    __tostring = Vector3Methods.__tostring,
}

--- Creates a new Vector3.
function M.Vector3.new(x, y, z)
    return setmetatable({ x = x, y = y, z = z }, Vector3MT)
end

-- Quaternion

M.Quaternion = {}
local QuaternionMethods = {}

--- Returns the conjugate (inverse for unit quaternions).
function QuaternionMethods:conjugate()
    return M.Quaternion.new(self.w, -self.x, -self.y, -self.z)
end

--- Returns the magnitude (length) of the quaternion.
function QuaternionMethods:magnitude()
    return math.sqrt(self.w * self.w + self.x * self.x + self.y * self.y + self.z * self.z)
end

--- Returns the unit quaternion. Returns identity if magnitude is zero.
function QuaternionMethods:norm()
    local mag = self:magnitude()
    if mag == 0 then return M.Quaternion.identity() end
    return self:scale(1 / mag)
end

--- Returns a new quaternion with each component scaled by n.
function QuaternionMethods:scale(n)
    return M.Quaternion.new(self.w * n, self.x * n, self.y * n, self.z * n)
end

--- Decomposes this quaternion into an axis and angle representation.
--- Returns: axis (vec3), angle (radians, in range (-pi, pi]).
function QuaternionMethods:toAxisAngle()
    local q  = self:norm()
    local qw = M.clamp(q.w, -1, 1)

    local angle = 2 * math.acos(qw)
    local s     = math.sqrt(1 - qw * qw)

    local axis
    if s < 1e-6 then
        axis = M.Vector3.new(1, 0, 0)
    else
        axis = M.Vector3.new(q.x / s, q.y / s, q.z / s)
    end

    if angle > math.pi then
        angle = angle - 2 * math.pi
        axis  = axis:scale(-1)
    end

    return axis, angle
end

--- Converts to Euler angles (pitch, yaw, roll) in radians.
--- Convention: yaw+ clockwise, pitch+ downward.
function QuaternionMethods:toEuler()
    local sinr_cosp = 2 * (self.w * self.x + self.y * self.z)
    local cosr_cosp = 1 - 2 * (self.x * self.x + self.y * self.y)
    local roll      = math.atan2(sinr_cosp, cosr_cosp)

    local sinp  = 2 * (self.w * self.y - self.z * self.x)
    local pitch = -math.asin(M.clamp(sinp, -1, 1))

    local siny_cosp = 2 * (self.w * self.z + self.x * self.y)
    local cosy_cosp = 1 - 2 * (self.y * self.y + self.z * self.z)
    local yaw       = math.atan2(-siny_cosp, cosy_cosp)

    return pitch, yaw, roll
end

--- Spherical linear interpolation between this quaternion and b.
--- alpha = 0 returns self, alpha = 1 returns b.
--- Produces a smooth constant-speed rotation between the two orientations.
function QuaternionMethods:slerp(b, alpha)
    local a  = self:norm()
    local bb = b:norm()

    local dot = a.w * bb.w + a.x * bb.x + a.y * bb.y + a.z * bb.z

    if dot < 0 then
        bb  = bb:scale(-1)
        dot = -dot
    end

    if dot > 0.9995 then
        local result = M.Quaternion.new(
            a.w + alpha * (bb.w - a.w),
            a.x + alpha * (bb.x - a.x),
            a.y + alpha * (bb.y - a.y),
            a.z + alpha * (bb.z - a.z)
        )
        return result:norm()
    end

    local theta_0   = math.acos(dot)
    local theta     = theta_0 * alpha
    local sinTheta  = math.sin(theta)
    local sinTheta0 = math.sin(theta_0)

    local s0 = math.cos(theta) - dot * sinTheta / sinTheta0
    local s1 = sinTheta / sinTheta0

    return M.Quaternion.new(
        s0 * a.w + s1 * bb.w,
        s0 * a.x + s1 * bb.x,
        s0 * a.y + s1 * bb.y,
        s0 * a.z + s1 * bb.z
    ):norm()
end

--- Returns a formatted string representation.
function QuaternionMethods:__tostring()
    return string.format("(%g, %g, %g, %g)", self.w, self.x, self.y, self.z)
end

-- Metamethods

--- Quaternion multiplication (Hamilton product).
--- Represents the composition of two rotations: a then b is written as b * a.
local function quatMul(a, b)
    return M.Quaternion.new(
        a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
        a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
        a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
        a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w
    )
end

--- Unary negation (represents the same rotation, opposite sign convention).
local function quatUnm(a)
    return M.Quaternion.new(-a.w, -a.x, -a.y, -a.z)
end

--- Equality check (component-wise).
local function quatEq(a, b)
    return a.w == b.w and a.x == b.x and a.y == b.y and a.z == b.z
end

local QuaternionMT = {
    __index    = QuaternionMethods,
    __mul      = quatMul,
    __unm      = quatUnm,
    __eq       = quatEq,
    __tostring = QuaternionMethods.__tostring,
}

--- Creates a new Quaternion with components (w, x, y, z).
--- w is the scalar part, (x, y, z) is the vector part.
function M.Quaternion.new(w, x, y, z)
    return setmetatable({ w = w, x = x, y = y, z = z }, QuaternionMT)
end

--- Returns the identity quaternion (no rotation).
function M.Quaternion.identity()
    return M.Quaternion.new(1, 0, 0, 0)
end

--- Constructs a quaternion representing a rotation of angle radians
--- around the given axis. The axis does not need to be unit length.
function M.Quaternion.fromAxisAngle(axis, angle)
    local a = axis:norm()
    if a:length() == 0 or angle == 0 then
        return M.Quaternion.identity()
    end
    local ha = angle * 0.5
    local s  = math.sin(ha)
    return M.Quaternion.new(math.cos(ha), a.x * s, a.y * s, a.z * s)
end

--- Constructs a quaternion from Euler angles (pitch, yaw, roll) in radians.
--- Application order: yaw (Y) -> pitch (X) -> roll (Z).
--- Convention: yaw+ clockwise, pitch+ downward.
function M.Quaternion.fromEuler(pitch, yaw, roll)
    local cp = math.cos(-pitch * 0.5)
    local sp = math.sin(-pitch * 0.5)
    local cy = math.cos(-yaw * 0.5)
    local sy = math.sin(-yaw * 0.5)
    local cr = math.cos(roll * 0.5)
    local sr = math.sin(roll * 0.5)

    return M.Quaternion.new(
        cr * cp * cy + sr * sp * sy,
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy
    )
end

--- Constructs a quaternion that rotates the vector "from" to the vector "to".
--- Both vectors should be unit length for best results.
function M.Quaternion.fromVectors(from, to)
    local d = from:dot(to)
    if d >= 1 then
        return M.Quaternion.identity()
    end
    if d <= -1 then
        -- vectors are anti-parallel: pick an arbitrary perpendicular axis
        local perp = from:cross(M.Vector3.new(1, 0, 0))
        if perp:lengthSq() < 1e-6 then
            perp = from:cross(M.Vector3.new(0, 1, 0))
        end
        return M.Quaternion.new(0, perp.x, perp.y, perp.z):norm()
    end
    local axis = from:cross(to)
    return M.Quaternion.new(1 + d, axis.x, axis.y, axis.z):norm()
end

-- Matrix3

M.Matrix3 = {}
local Matrix3Methods = {}

--- Creates a new 3x3 matrix.
function M.Matrix3.new(
    m11,m12,m13,
    m21,m22,m23,
    m31,m32,m33
)
    return setmetatable({
        m11=m11,m12=m12,m13=m13,
        m21=m21,m22=m22,m23=m23,
        m31=m31,m32=m32,m33=m33
    },{
        __index = Matrix3Methods,

        __add = function(a,b)
            return M.Matrix3.new(
                a.m11+b.m11, a.m12+b.m12, a.m13+b.m13,
                a.m21+b.m21, a.m22+b.m22, a.m23+b.m23,
                a.m31+b.m31, a.m32+b.m32, a.m33+b.m33
            )
        end,

        __sub = function(a,b)
            return M.Matrix3.new(
                a.m11-b.m11, a.m12-b.m12, a.m13-b.m13,
                a.m21-b.m21, a.m22-b.m22, a.m23-b.m23,
                a.m31-b.m31, a.m32-b.m32, a.m33-b.m33
            )
        end,

        __mul = function(a,b)

            -- scalar * matrix
            if type(a) == "number" then
                return M.Matrix3.new(
                    a*b.m11,a*b.m12,a*b.m13,
                    a*b.m21,a*b.m22,a*b.m23,
                    a*b.m31,a*b.m32,a*b.m33
                )
            end

            -- matrix * scalar
            if type(b) == "number" then
                return M.Matrix3.new(
                    a.m11*b,a.m12*b,a.m13*b,
                    a.m21*b,a.m22*b,a.m23*b,
                    a.m31*b,a.m32*b,a.m33*b
                )
            end

            -- matrix * vector
            if b.x then
                return M.Vector3.new(
                    a.m11*b.x + a.m12*b.y + a.m13*b.z,
                    a.m21*b.x + a.m22*b.y + a.m23*b.z,
                    a.m31*b.x + a.m32*b.y + a.m33*b.z
                )
            end

            -- matrix * matrix
            return M.Matrix3.new(
                a.m11*b.m11 + a.m12*b.m21 + a.m13*b.m31,
                a.m11*b.m12 + a.m12*b.m22 + a.m13*b.m32,
                a.m11*b.m13 + a.m12*b.m23 + a.m13*b.m33,

                a.m21*b.m11 + a.m22*b.m21 + a.m23*b.m31,
                a.m21*b.m12 + a.m22*b.m22 + a.m23*b.m32,
                a.m21*b.m13 + a.m22*b.m23 + a.m23*b.m33,

                a.m31*b.m11 + a.m32*b.m21 + a.m33*b.m31,
                a.m31*b.m12 + a.m32*b.m22 + a.m33*b.m32,
                a.m31*b.m13 + a.m32*b.m23 + a.m33*b.m33
            )
        end
    })
end

-- returns identity matrix
function M.Matrix3.identity()
    return M.Matrix3.new(
        1,0,0,
        0,1,0,
        0,0,1
    )
end

-- returns zero matrix
function M.Matrix3.zero()
    return M.Matrix3.new(
        0,0,0,
        0,0,0,
        0,0,0
    )
end

-- transposes matrix
function Matrix3Methods:transpose()
    return M.Matrix3.new(
        self.m11,self.m21,self.m31,
        self.m12,self.m22,self.m32,
        self.m13,self.m23,self.m33
    )
end

-- gets determinant of matrix
function Matrix3Methods:det()
    return
        self.m11*(self.m22*self.m33 - self.m23*self.m32)
      - self.m12*(self.m21*self.m33 - self.m23*self.m31)
      + self.m13*(self.m21*self.m32 - self.m22*self.m31)
end

-- gets inverse of matrix
function Matrix3Methods:inverse()
    local det = self:det()
    if det == 0 then return nil end
    local invDet = 1/det

    return M.Matrix3.new(
        (self.m22*self.m33 - self.m23*self.m32)*invDet,
        (self.m13*self.m32 - self.m12*self.m33)*invDet,
        (self.m12*self.m23 - self.m13*self.m22)*invDet,

        (self.m23*self.m31 - self.m21*self.m33)*invDet,
        (self.m11*self.m33 - self.m13*self.m31)*invDet,
        (self.m13*self.m21 - self.m11*self.m23)*invDet,

        (self.m21*self.m32 - self.m22*self.m31)*invDet,
        (self.m12*self.m31 - self.m11*self.m32)*invDet,
        (self.m11*self.m22 - self.m12*self.m21)*invDet
    )
end

--- Outer product (returns a Matrix3)
function Vector3Methods:outer(b)
    return M.Matrix3.new(
        self.x*b.x, self.x*b.y, self.x*b.z,
        self.y*b.x, self.y*b.y, self.y*b.z,
        self.z*b.x, self.z*b.y, self.z*b.z
    )
end

return M