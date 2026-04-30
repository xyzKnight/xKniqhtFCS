-- main.lua

term.clear()
term.setCursorPos(1,1)
print("CC:FCS CONTROLLER BOOT:")

-- IMPORTS
local u = require("math_utils_v6")
local vec3 = u.Vector3
local quat = u.Quaternion

local c = require("config")
local solver_module = require("intercept_solver")

-- PERIPHERAL WRAPPING
local reader = peripheral.find(c.peripheral.BLOCK_READER_TYPE) or error("Block reader not attached")
local radar  = peripheral.find(c.peripheral.RADAR_TYPE)        or error("Radar not attached")

local RSCtbl = {peripheral.find(c.peripheral.RSC_TYPE)} or error("No speed controllers attached")
if #RSCtbl ~= 2 then error("Invalid number of speed controllers attached (must be 2)") end

-- INIT

local RSCs = { pitch = {}, yaw = {} }
RSCs.pitch.rsc  = RSCtbl[c.peripheral.PITCH_RSC_INDEX]
RSCs.yaw.rsc    = RSCtbl[c.peripheral.YAW_RSC_INDEX]
RSCs.pitch.sign = c.peripheral.PITCH_RSC_SIGN
RSCs.yaw.sign   = c.peripheral.YAW_RSC_SIGN

-- STATE
local state = {
    cmd_log = {
        { pitch = 0, yaw = 0 }, -- RPM sitting in RSC
        { pitch = 0, yaw = 0 }, -- RPM executing at cannon
    }
}

local cannon_pitch = 0
local cannon_yaw   = 0
local req_pitch    = 0
local req_yaw      = 0
local rsc_pitch    = 0
local rsc_yaw      = 0
local target_data  = {}

local D_READ = c.controller.D_READ
local req_history = {}
for i = 1, D_READ + 1 do
    req_history[i] = { pitch = 0, yaw = 0 }
end

-- updates log of intercept angles
function update_req_history()
    table.insert(req_history, 1, { pitch = req_pitch, yaw = req_yaw })
    req_history[D_READ + 2] = nil
end

-- prints cannon angle error
function print_error()
    local past_req = req_history[D_READ + 1]
    local comp_ep = u.wrap_angle_deg(past_req.pitch - cannon_pitch)
    local comp_ey = u.wrap_angle_deg(past_req.yaw   - cannon_yaw)
    print("err pitch:",comp_ep)
    print("err yaw:",comp_ey)
end


-- reads cannon mount pitch and yaw
function update_cannon_data()
    while true do
        local data = reader.getBlockData() -- 1 tick yield
        cannon_pitch = u.wrap_angle_deg(data.CannonPitch)
        cannon_yaw   = u.wrap_angle_deg(data.CannonYaw)
    end
end

-- gets target data from sp radar
function update_target_data()
    local temp_id = 6
    while true do
        local scans = radar.scanForShips(c.peripheral.RADAR_SCAN_RANGE) -- 1 tick yield
        local found = false
        for k, v in pairs(scans) do
            if v.id == temp_id or true then
                found = true
                target_data.pos       = vec3.new(v.pos.x,      v.pos.y,      v.pos.z)
                target_data.velo      = vec3.new(v.velocity.x, v.velocity.y, v.velocity.z)
                target_data.read_time = os.clock()
            end
        end
        if not found then
            target_data.pos       = vec3.new(0, 0, 0)
            target_data.velo      = vec3.new(0, 0, 0)
            target_data.read_time = os.clock()
        end
    end
end


-- finds where the cannon will be when the new command begins executing
function reconstruct_mid_state(cannon_pitch, cannon_yaw)
    local function rsc_rpm_to_deg_per_tick(rsc_rpm)
        return (rsc_rpm / c.controller.RSC_TO_CANNON) * 6 * c.controller.DT
    end

    local mid_pitch = cannon_pitch
        + rsc_rpm_to_deg_per_tick(state.cmd_log[2].pitch)
        + rsc_rpm_to_deg_per_tick(state.cmd_log[1].pitch)

    local mid_yaw = cannon_yaw
        + rsc_rpm_to_deg_per_tick(state.cmd_log[2].yaw)
        + rsc_rpm_to_deg_per_tick(state.cmd_log[1].yaw)

    return mid_pitch, mid_yaw
end


-- finds RSC RPM that closes the delta between midstate angles and required angles in one tick
function backsolve_rsc(mid_pitch, mid_yaw, req_pitch, req_yaw)
    local function deg_per_tick_to_rsc_rpm(delta_deg)
        return (delta_deg / c.controller.DT) / 6 * c.controller.RSC_TO_CANNON
    end

    local delta_p = u.wrap_angle_deg(req_pitch - mid_pitch)
    local delta_y = u.wrap_angle_deg(req_yaw   - mid_yaw)

    local rsc_pitch = deg_per_tick_to_rsc_rpm(delta_p)
    local rsc_yaw   = deg_per_tick_to_rsc_rpm(delta_y)

    rsc_pitch = u.clamp(rsc_pitch, -c.controller.MAX_RSC_RPM, c.controller.MAX_RSC_RPM)
    rsc_yaw   = u.clamp(rsc_yaw,   -c.controller.MAX_RSC_RPM, c.controller.MAX_RSC_RPM)

    return rsc_pitch, rsc_yaw
end


-- RSC output
function write_and_log()
    while true do
        parallel.waitForAll(
            function() RSCs.pitch.rsc.setTargetSpeed(rsc_pitch * RSCs.pitch.sign) end,
            function() RSCs.yaw.rsc.setTargetSpeed(rsc_yaw * RSCs.yaw.sign) end
        )
        state.cmd_log[2] = state.cmd_log[1]
        state.cmd_log[1] = { pitch = rsc_pitch, yaw = rsc_yaw }
    end
end


-- main
function main()
    os.sleep(0.1)
    local solver = solver_module.new(c.solver)
    local count  = 0
    while true do
        os.sleep(0)
        term.clear()
        term.setCursorPos(1, 1)
        count = count + 1

        print("C-RAM CONTROLLER:")
        print("")
        print("iter:", count)
        print("\nt-pos:",  solver.target.pos)
        print("t-velo:", solver.target.velo)
        print("t-accel:",solver.target.accel)
        print("\nc-pos:", c.solver.cannon.pos)
        print("c-pitch:", cannon_pitch)
        print("c-yaw:", cannon_yaw)
        print("\nmiss_dist:", solver.intercept.miss_dist)
        print("intercept:", solver.intercept.target_pos)
        -- predictor diagnostics
        local mid_pitch, mid_yaw = reconstruct_mid_state(cannon_pitch, cannon_yaw)

        update_req_history()
        req_pitch, req_yaw = solver:solve(target_data, (c.controller.D_TOTAL + c.controller.D_TARGET_ALIGNMENT) * c.controller.DT)
        print_error()
        rsc_pitch, rsc_yaw = backsolve_rsc(mid_pitch, mid_yaw, req_pitch, req_yaw)
    end
end

-- parallel call synchonises everything
parallel.waitForAny(
    update_cannon_data,
    update_target_data,
    main,
    write_and_log
)
