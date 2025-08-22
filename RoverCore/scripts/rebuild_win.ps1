param(
  [ValidateSet("Release","Debug")] [string] $Configuration = "Release",
  [switch] $StaticRuntime  # add -StaticRuntime if you want to link static MSVC runtime
)

# Run this in: "x64 Native Tools Command Prompt for VS"
# Or in PowerShell after importing the VS dev environment.

$buildDir = "build\$Configuration"

Write-Host "[*] Nuking $buildDir"
Remove-Item $buildDir -Recurse -Force -ErrorAction Ignore | Out-Null

$cmakeArgs = @(
  "-S", ".", "-B", $buildDir,
  "-G", "Ninja",
  "-DCMAKE_BUILD_TYPE=$Configuration"
)

if ($StaticRuntime) {
  $cmakeArgs += "-DMSVC_STATIC_RUNTIME=ON"
}

Write-Host "[*] Configuring ($Configuration)"
cmake @cmakeArgs

Write-Host "[*] Building"
cmake --build $buildDir --verbose

Write-Host "[*] Binary:"
Write-Host "    $buildDir\bin\rover_core.exe"
