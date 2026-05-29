$ip = "10.110.165.170"

$body = @{
  sendstate = "START"
  scene = "Town04"
  sky = "Sunny"
  sunshinetime = "Noon"
  drive_mode = "AIGO"
  loadingtransportation = "0"
  loadingsensor = "0"
  vehiclemodel = "Tesla Model 3"
  camera_view = "follow"
} | ConvertTo-Json

Write-Host "===== launch demo ====="
Invoke-RestMethod "http://$ip`:8765/command" -Method Post -ContentType "application/json" -Body $body | ConvertTo-Json -Depth 12

Start-Sleep -Seconds 8

Write-Host "===== health ====="
Invoke-RestMethod "http://$ip`:8765/health" | ConvertTo-Json -Depth 12

Write-Host "===== sensors ====="
Invoke-RestMethod "http://$ip`:8765/sensors" | ConvertTo-Json -Depth 12

Write-Host "===== telemetry ====="
Invoke-RestMethod "http://$ip`:8765/telemetry" | ConvertTo-Json -Depth 12

Write-Host "===== side mirror URLs ====="
Write-Host "http://192.168.110.106:8771/rear_left.mjpg"
Write-Host "http://192.168.110.107:8771/rear_right.mjpg"
