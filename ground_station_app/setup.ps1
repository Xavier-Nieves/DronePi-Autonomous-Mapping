# ─────────────────────────────────────────────────────────────────────────────
# setup.ps1 — DronePi Ground Station — One-shot Windows setup
#
# Run this once on a fresh machine (or after a clean reinstall).
# Right-click → "Run with PowerShell", or in a terminal:
#
#   powershell -ExecutionPolicy Bypass -File setup.ps1
#
# What this script does
# ─────────────────────
#   1. Checks Python >= 3.11 is installed
#   2. Installs the ground_station Python package + all dependencies
#   3. Checks if Ollama is installed — offers to install via winget if not
#   4. Pulls the qwen2.5:7b model if Ollama is present
#   5. Runs ground-station install  (creates desktop shortcut + .pyw launcher)
#   6. Generates 10 test flights in the local cache
#   7. Starts the ground station and opens the browser
#
# Requirements
# ────────────
#   - Windows 10/11
#   - Python 3.11+ (https://python.org/downloads)
#   - Internet access for pip and Ollama model download (~4.5 GB)
# ─────────────────────────────────────────────────────────────────────────────

$ErrorActionPreference = "Stop"

function Write-Step  { param($msg) Write-Host "`n>> $msg" -ForegroundColor Cyan }
function Write-OK    { param($msg) Write-Host "   OK  $msg" -ForegroundColor Green }
function Write-Warn  { param($msg) Write-Host "   !!  $msg" -ForegroundColor Yellow }
function Write-Fail  { param($msg) Write-Host "   XX  $msg" -ForegroundColor Red }

# ── Locate this script's directory (must be ground_station_app/) ──────────────
$AppDir = Split-Path -Parent $MyInvocation.MyCommand.Path
Set-Location $AppDir
Write-Step "Working directory: $AppDir"

# ─────────────────────────────────────────────────────────────────────────────
# 1. Python version check
# ─────────────────────────────────────────────────────────────────────────────
Write-Step "Checking Python version..."
try {
    $pyver = python --version 2>&1
    if ($pyver -match "Python (\d+)\.(\d+)") {
        $major = [int]$Matches[1]
        $minor = [int]$Matches[2]
        if ($major -lt 3 -or ($major -eq 3 -and $minor -lt 11)) {
            Write-Fail "Python 3.11+ required. Found: $pyver"
            Write-Fail "Download from https://python.org/downloads"
            exit 1
        }
        Write-OK "$pyver"
    } else {
        Write-Fail "Could not parse Python version. Is Python installed and on PATH?"
        exit 1
    }
} catch {
    Write-Fail "Python not found. Install from https://python.org/downloads"
    exit 1
}

# ─────────────────────────────────────────────────────────────────────────────
# 2. Install Python package and dependencies
# ─────────────────────────────────────────────────────────────────────────────
Write-Step "Installing dronepi-ground Python package..."
python -m pip install --upgrade pip --quiet
python -m pip install -e . --quiet
if ($LASTEXITCODE -ne 0) {
    Write-Fail "pip install failed. Check your internet connection."
    exit 1
}
Write-OK "dronepi-ground installed (typer, rich, httpx, pyulog, pandas)"

# ─────────────────────────────────────────────────────────────────────────────
# 3. Ollama check + optional install
# ─────────────────────────────────────────────────────────────────────────────
Write-Step "Checking Ollama..."
$ollamaInstalled = $false
try {
    $ollamaVer = ollama --version 2>&1
    Write-OK "Ollama found: $ollamaVer"
    $ollamaInstalled = $true
} catch {
    Write-Warn "Ollama not found."
    $response = Read-Host "   Install Ollama via winget? (y/n)"
    if ($response -match "^[Yy]") {
        Write-Step "Installing Ollama via winget..."
        winget install Ollama.Ollama --accept-package-agreements --accept-source-agreements
        if ($LASTEXITCODE -eq 0) {
            Write-OK "Ollama installed. You may need to restart your terminal."
            $ollamaInstalled = $true
        } else {
            Write-Warn "winget install failed. Download manually from https://ollama.com/download"
            Write-Warn "Then run: ollama pull qwen2.5:7b"
        }
    } else {
        Write-Warn "Skipping Ollama. The ground station will run but cannot generate AI reports."
        Write-Warn "Install later: https://ollama.com/download  then  ollama pull qwen2.5:7b"
    }
}

# ─────────────────────────────────────────────────────────────────────────────
# 4. Pull qwen2.5:7b model
# ─────────────────────────────────────────────────────────────────────────────
if ($ollamaInstalled) {
    Write-Step "Checking qwen2.5:7b model..."
    $modelList = ollama list 2>&1
    if ($modelList -match "qwen2.5:7b") {
        Write-OK "qwen2.5:7b already downloaded"
    } else {
        Write-Warn "qwen2.5:7b not found. Downloading (~4.5 GB)..."
        Write-Warn "This may take several minutes depending on your connection."
        ollama pull qwen2.5:7b
        if ($LASTEXITCODE -eq 0) {
            Write-OK "qwen2.5:7b downloaded successfully"
        } else {
            Write-Warn "Model download failed. Run manually: ollama pull qwen2.5:7b"
        }
    }
}

# ─────────────────────────────────────────────────────────────────────────────
# 5. Create desktop shortcut + .pyw launcher
# ─────────────────────────────────────────────────────────────────────────────
Write-Step "Creating desktop shortcut..."
python ground_station\cli.py install
if ($LASTEXITCODE -eq 0) {
    Write-OK "Desktop shortcut created — double-click 'DronePi Ground Station' to launch"
} else {
    Write-Warn "Shortcut creation failed. You can still launch with: python ground_station\cli.py start"
}

# ─────────────────────────────────────────────────────────────────────────────
# 6. Generate test flights (populates cache for offline testing)
# ─────────────────────────────────────────────────────────────────────────────
Write-Step "Generating 10 test flights in local cache..."
$testScript = Join-Path $AppDir "gen_test_flights.py"
if (Test-Path $testScript) {
    python $testScript
    if ($LASTEXITCODE -eq 0) {
        Write-OK "Test flights created in $env:USERPROFILE\.dronepi-ground\cache\"
        Write-OK "Click 'Sync DB' in the dashboard sidebar to index them into the database"
    } else {
        Write-Warn "Test flight generation failed — not critical, skip for now"
    }
} else {
    Write-Warn "gen_test_flights.py not found — skipping test data generation"
    Write-Warn "Copy gen_test_flights.py into $AppDir and run: python gen_test_flights.py"
}

# ─────────────────────────────────────────────────────────────────────────────
# 7. Start the ground station
# ─────────────────────────────────────────────────────────────────────────────
Write-Step "Starting DronePi Ground Station..."
python ground_station\cli.py start
if ($LASTEXITCODE -eq 0) {
    Write-OK "Ground station started at http://localhost:8765"
    Write-OK "Opening browser..."
    Start-Sleep -Seconds 2
    Start-Process "http://localhost:8765"
} else {
    Write-Warn "Start failed. Try manually: python ground_station\cli.py start"
}

# ─────────────────────────────────────────────────────────────────────────────
Write-Host ""
Write-Host "═══════════════════════════════════════════════════════" -ForegroundColor Cyan
Write-Host "  DronePi Ground Station setup complete." -ForegroundColor Cyan
Write-Host "" 
Write-Host "  Dashboard:  http://localhost:8765"
Write-Host "  Stop:       python ground_station\cli.py stop"
Write-Host "  Logs:       python ground_station\cli.py logs -f"
Write-Host "  Replay:     python ground_station\cli.py replay <session_id>"
Write-Host "═══════════════════════════════════════════════════════" -ForegroundColor Cyan
Write-Host ""
