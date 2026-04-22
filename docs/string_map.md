# string map

| 英文| 中文|
|---|---|
| RF_RX_V02 Host |搖控設定工具|
| RF_RX_V02 Connection | 裝置連接 |
| Endpoint | 連接埠 |
| BaudRate | 速率 |
| Refresh | 重新整理 |
| Connect | 連線 |
| Disconnect | 中斷連線 |
| RESET MCU | 重新開機 |
| Disconnected | 已斷線 |
| Open Parameters | 參數設定 |
| Live Snapshot | 即時資料 |
| Read RSSI | 訊號強度 |
| Read BAT_MV | 電芯電壓 |
| - |  |
| Live Trend (1 point/sec) | 即時數據(1點/秒) |
| Start Trend | 啟動即時記錄 |
| Stop Trend | 停止即時記錄 |
| Trend A |  |
| Trend B |  |
| A: - |  |
| B: - |  |
| Please select endpoint. | 請選擇連接埠 |
| Invalid baudrate. | 參數不合法 |
| Connected: {endpoint} | 已連線:{endpoint} |
| MCU Reset | 重新開機! |
| Send RESET command to firmware and reboot MCU? | 確定要重新開機? |
| Yes | 是 |
| No | 否 |
| RESET command sent. | 已送出指令 |
| Reset failed: {message} | 重新開機失敗:{message} |
| RSSI: {value} | 訊號強度: {value} |
| RSSI read failed: {message} | 訊號強度讀取失敗: {message} |
| BAT_MV: {value} | 電池電壓: {value} |
| BAT read failed: {message} | 電池電壓讀取失敗: {value} |
| A {name}: {value} |  |
| B {name}: {value} |  |
| Trend A error: {message} |  |
| Trend B error: {message} |  |
| DF Parameters | 模組參數 |
| Linear Interpolation (4 points x 3 groups) | 控制參數表 |
| VR (RAW -> ANGLE) | 步進馬達對應表(VR->角度) |
| PWM (RAW -> COUNTER) | 比例閥對應表(VR->輸出值) |
| SERVO (RAW -> COUNTER) | 伺服機/電腦對應表(VR->輸出值) |
| Pt | 編號 |
| X (RAW) | VR數值(0~4000) |
| Y (ANGLE) | 角度(0~180) |
| Y (COUNTER) | 輸出數值(0~5000) |
| Read VR | 讀取步進參數 |
| Write VR | 寫入步進參數 |
| Read PWM | 讀取比例閥參數 |
| Write PWM | 寫入比例閥參數 |
| Read SERVO | 讀取伺服/電腦參數 |
| Write SERVO | 寫入伺服/電腦參數 |
| Read 3 Groups | 讀取所有參數 |
| OP_MODE Editor | 運作模式設定 |
| TX/RX | 發射/接收 |
| Control/Switch | 控制/過捲 |
| Gear Type | 油門控制功能 |
| Type |  |
| Composed Value | 控制內碼 |
| Read OP_MODE | 讀取運作模式 |
| Write OP_MODE | 寫入運作模式 |
| Use In Single Tool |  |
| DIO_ON_MASK (16-bit) | 同動控制 |
| AUX1 (bit8) | 接點9 |
| AUX2 (bit9) | 接點10 |
| Mask: 0 (0x0000) |  |
| Read DIO_ON_MASK | 讀取同動控制 |
| Write DIO_ON_MASK | 寫入同動控制 |
| LBT Voltage Bound (mV) | 電池低壓檢測 |
| Low Bound (mV) | 電池低壓檢測-下限 |
| High Bound (mV) | 電池低壓檢測-上限 |
| 0..65535 |  |
| Read LBT | 讀取低壓設定 |
| Write LBT | 寫入低壓設定 |
| Single Parameter Tool | 參數讀寫 |
| Value |  |
| Range: - |  |
| Read | 讀取 |
| Write | 寫入 |
| Read All | 全部讀回 |
| TX | 發射機 |
| RX | 接收機 |
| CONTROL | 控制器 |
| SWITCH | 過捲 |
| GEAR_STEPPER | 步進馬達 |
| GEAR_SERVO | 伺服馬達 |
| GEAR_DAC | 行車電腦 |
| GEAR_PWM | 比例閥 |
| GEAR_STEPPER_DAC | 步進馬達/行車電腦 |
| TX_IO | 發射機-單軸 |
| TX_ADC | 發射機-四軸 |
| VALVE_IO | 開關閥 |
| VALVE_PWM | 比例閥 |
| VR |  |
| PWM |  |
| SERVO |  |
| Range: {min} - {max}  Default: {default} |  |
| Read failed: {message} |  |
| Invalid value. |  |
| OK |  |
| Value is out of range. |  |
| Write OK: {name}={value} |  |
| Write failed: {message} |  |
| Read All failed: {message} |  |
| [{group}] |  |
| Pt{index}: X={x}, Y={y} |  |
| Read 3 groups failed: {message} |  |
| TX Type |  |
| Valve Type |  |
| {value} (0x{hex}, {bin}) |  |
| Read OP_MODE OK: {value} (0x{hex}) |  |
| Read OP_MODE failed: {message} |  |
| Write OP_MODE OK: {value} (0x{hex}) |  |
| Write OP_MODE failed: {message} |  |
| Mask: {value} (0x{hex}) |  |
| Read DIO_ON_MASK OK: {value} (0x{hex}) |  |
| Read DIO_ON_MASK failed: {message} |  |
| Write DIO_ON_MASK OK: {value} (0x{hex}) |  |
| Write DIO_ON_MASK failed: {message} |  |
| Read LBT OK: Low={low}mV, High={high}mV |  |
| Read LBT failed: {message} |  |
| Write LBT |  |
| Low Bound must be 0..65535 mV. |  |
| High Bound must be 0..65535 mV. |  |
| Low Bound must be <= High Bound. |  |
| Write LBT OK: Low={low}mV, High={high}mV |  |
| Write LBT failed: {message} |  |
| Read group OK: {group} |  |
| Read group failed: {message} |  |
| Write Group |  |
| {group}: X (RAW) must be non-decreasing from Pt1 to Pt4. |  |
| Write group OK: {group} |  |
| Write group failed: {message} |  |
| Unknown interpolation group. |  |
| Android Bluetooth transport is not implemented yet. |  |
| Transport is not connected. |  |
| NAK from device. id={id} |  |
| Unexpected reply cmd=0x{cmd} |  |
| Reply id mismatch. expected={expected}, actual={actual} |  |
| Timeout waiting reply for id={id} cmd frame. |  |
