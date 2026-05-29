param(
  [string]$Ip = "10.110.165.170",
  [int]$Port = 8765,
  [int]$RunSeconds = 40
)

$ErrorActionPreference = "Stop"

$vehicles = @(
  "Dodge Charger",
  "Lincoln MKZ",
  "Tesla Model 3",
  "Audi e-tron",
  "Jeep Wrangler",
  "Tesla Cybertruck",
  "Fuso Rosa",
  "Mercedes Sprinter",
  "Volkswagen T2",
  "Carlacola Truck",
  "European HGV",
  "Firetruck"
)

$views = @("driver", "follow")
$baseUrl = "http://$Ip`:$Port"
$caseNo = 0
$total = $vehicles.Count * $views.Count

Write-Host "===== AIGO Town04 24-view camera test ====="
Write-Host "Target: $baseUrl"
Write-Host "Each case: $RunSeconds sec"

foreach ($vehicle in $vehicles) {
  foreach ($view in $views) {
    $caseNo += 1
    Write-Host ""
    Write-Host "===== TEST $caseNo / $total ====="
    Write-Host "vehiclemodel=$vehicle"
    Write-Host "camera_view=$view"

    $body = @{
      sendstate = "START"
      scene = "Town04"
      sky = "Sunny"
      sunshinetime = "Noon"
      drive_mode = "AIGO"
      loadingtransportation = "1"
      vehiclemodel = $vehicle
      camera_view = $view
    } | ConvertTo-Json

    Invoke-RestMethod "$baseUrl/command" `
      -Method Post `
      -ContentType "application/json" `
      -Body $body | ConvertTo-Json -Depth 10

    $start = Get-Date
    while (((Get-Date) - $start).TotalSeconds -lt $RunSeconds) {
      Start-Sleep -Seconds 10
      try {
        $h = Invoke-RestMethod "$baseUrl/health"
        $d = $h.diagnostics
        $elapsed = [int](((Get-Date) - $start).TotalSeconds)
        Write-Host "[$caseNo/$total][$elapsed sec] mode=$($h.mode) running=$($h.running) vehicle_alive=$($h.vehicle_alive) speed=$($d.speed_kmh) view=$($h.camera_view)"
      } catch {
        Write-Host "health check failed: $($_.Exception.Message)"
      }
    }
  }
}

Write-Host ""
Write-Host "===== 24-view test done ====="
