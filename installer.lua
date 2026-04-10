-- wget run https://raw.githubusercontent.com/xyzKnight/xKniqhtFCS/refs/heads/main/installer.lua

local projectDir = "xKniqhtFCS"

local files = {
    { "main.lua", "https://raw.githubusercontent.com/xyzKnight/xKniqhtFCS/refs/heads/main/main.lua" },
    { "intercept_solver.lua", "https://raw.githubusercontent.com/xyzKnight/xKniqhtFCS/refs/heads/main/intercept_solver.lua" },
    { "math_utils_v6.lua", "https://raw.githubusercontent.com/xyzKnight/xKniqhtFCS/refs/heads/main/math_utils_v6.lua" },
    { "predictor.lua", "https://raw.githubusercontent.com/xyzKnight/xKniqhtFCS/refs/heads/main/predictor.lua" },
    { "config.lua", "https://raw.githubusercontent.com/xyzKnight/xKniqhtFCS/refs/heads/main/config.lua" },
}

-- INSTALLER

local function ensureDir(path)
    local dir = fs.getDir(path)
    if dir ~= "" and not fs.exists(dir) then
        fs.makeDir(dir)
    end
end

local function downloadFile(url, path)
    local response = http.get(url)
    if not response then
        return false, "Failed to connect"
    end

    local content = response.readAll()
    response.close()

    local file = fs.open(path, "w")
    file.write(content)
    file.close()

    return true
end

print("Installing project to /" .. projectDir .. "...\n")

for _, file in ipairs(files) do
    local relativePath = file[1]
    local url = file[2]

    local fullPath = fs.combine(projectDir, relativePath)

    print("-> " .. fullPath)

    ensureDir(fullPath)

    local ok, err = downloadFile(url, fullPath)
    if not ok then
        print("   [FAILED] " .. err)
    else
        print("   [OK]")
    end
end

print("\nInstallation complete!")
print("Run main.lua to use!")

os.sleep(0.5)

local this = shell.getRunningProgram()
if this then
    fs.delete(this)
end
