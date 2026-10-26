#include <Arduino.h>
#include <driver/i2s.h>
#include <arduinoFFT.h>

// ================= CẤU HÌNH PHẦN CỨNG =================
#define I2S_WS      6   
#define I2S_SCK     7   
#define I2S_SD      10  // LƯU Ý: Đổi sang GPIO 2 hoặc 3 nếu chân 10 trên mạch bị hỏng
#define MOTOR_PIN   4   // Chân kích Transistor/MOSFET điều khiển motor rung

// ================= CẤU HÌNH FFT =================
const uint16_t samples = 512;           
const double samplingFrequency = 16000; 

float vReal[samples];
float vImag[samples];
int32_t sample_buffer[samples]; 
ArduinoFFT<float> FFT = ArduinoFFT<float>(vReal, vImag, samples, samplingFrequency);

// ================= BIẾN TRẠNG THÁI & NGƯỠNG CHO 4 LUỒNG =================

// 1. LUỒNG BÁO CHÁY (4.8kHz - 5.3kHz) - Quét đỉnh vùng + Grace Period 300ms
int fireConfidence = 0;              // Điểm tích lũy tin cậy
unsigned long lastFireFrameTime = 0; // Mốc thời gian bắt được frame báo cháy gần nhất
#define MIN_FIRE_TONE_RATIO 0.60f    // Ngưỡng tập trung năng lượng dải cao (60%)
#define FIRE_TRIGGER_SCORE  8        // Tích đủ 8 điểm là XUẤT LỆNH RUNG NGAY
#define FIRE_MAX_SCORE      12       // Trụ điểm tối đa
#define FIRE_GRACE_MS       1500      // Cho phép chờ 300ms không bị trừ điểm khi đứt frame

// 2. LUỒNG BÁO CO2 (3.3kHz - 3.4kHz) - Peak Hold + Đối soát 3 đỉnh lệch < 5%
int co2BeepCount = 0;
bool isCo2Beeping = false;
unsigned long lastCo2DetectTime = 0;
unsigned long co2WindowStartTime = 0;
double co2BeepPeakEnergy[3] = {0, 0, 0}; 
#define MIN_CO2_PURITY_RATIO 0.70f  // Tối thiểu 90% năng lượng dải trung-cao tập trung tại f0
#define MAX_CO2_ENERGY_DIFF  0.50f  // Độ lệch năng lượng giữa 3 đỉnh KHÔNG ĐƯỢC QUÁ 5% (0.05)

// 3. LUỒNG TIẾNG NỔ / VA ĐẬP - Thuật toán STA/LTA 2 cấp Log
#define HISTORY_SIZE 31 
int energy_history[HISTORY_SIZE] = {0};
int history_index = 0;
bool history_filled = false; 

// 4. LUỒNG CÒI XE GIAO THÔNG PHẠM VI < 5M (400Hz - 2500Hz)
unsigned long hornSustainStart = 0;
unsigned long lastHornDetectTime = 0;
#define HORN_ENERGY_5M      45000   // Ngưỡng năng lượng bắt còi xe cự ly gần (< 5m)
#define MIN_HORN_FREQ       400.0f  // Tần số còi xe tối thiểu (Hz)
#define MAX_HORN_FREQ       2500.0f // Tần số còi xe tối đa (Hz)
#define MIN_BAND_RATIO      0.60f   // Tỉ lệ tập trung dải năng lượng (>= 60%)
#define MIN_SUSTAIN_MS      250     // Phải kêu liên tục ít nhất 250ms

// ================= CẤU HÌNH I2S (32-bit chuẩn INMP441) =================
const i2s_config_t i2s_config = {
    .mode = i2s_mode_t(I2S_MODE_MASTER | I2S_MODE_RX),
    .sample_rate = 16000,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT, 
    .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT, 
    .communication_format = i2s_comm_format_t(I2S_COMM_FORMAT_STAND_I2S),
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = 8,
    .dma_buf_len = samples,
    .use_apll = false,
    .tx_desc_auto_clear = false,
    .fixed_mclk = 0
};

const i2s_pin_config_t pin_config = {
    .bck_io_num = I2S_SCK,
    .ws_io_num = I2S_WS,
    .data_out_num = I2S_PIN_NO_CHANGE,
    .data_in_num = I2S_SD
};

// ================= HÀM KÍCH HOẠT MOTOR RUNG & XẢ RÁC I2S =================
void triggerVibration(String alertType, float freq, int vibrationType) {
    Serial.printf("\n[!!!] BÁO ĐỘNG KHẨN CẤP: %s (%.0f Hz)\n", alertType.c_str(), freq);
    
    if (vibrationType == 3) {
        // Nhịp T3 cho báo cháy (Rung 3 nhịp dài)
        Serial.println("==> XUẤT NHỊP RUNG T3: [ Rung --- Rung --- Rung ]");
        for(int i = 0; i < 3; i++) {
            digitalWrite(MOTOR_PIN, HIGH); delay(500);
            digitalWrite(MOTOR_PIN, LOW);  delay(500);
        }
    } else if (vibrationType == 4) {
        // Nhịp T4 dồn dập cho báo CO2 (Rung 4 nhịp dồn)
        Serial.println("==> XUẤT NHỊP RUNG T4: [ Rung Rung Rung Rung ]");
        for(int i = 0; i < 4; i++) {
            digitalWrite(MOTOR_PIN, HIGH); delay(200);
            digitalWrite(MOTOR_PIN, LOW);  delay(200);
        }
    } else if (vibrationType == 5) {
        // Nhịp cho Tiếng Nổ / Va đập (Rung 1 hồi cực mạnh 2.5 giây)
        Serial.println("==> XUẤT NHỊP RUNG: [ Rung DÀI KHẨN CẤP (TIẾNG NỔ) ]");
        digitalWrite(MOTOR_PIN, HIGH); 
        delay(2500);
        digitalWrite(MOTOR_PIN, LOW);
    } else if (vibrationType == 6) {
        // Nhịp cho Còi Xe Giao Thông (< 5m) (Rung bíp - bíp ngắn)
        Serial.println("==> XUẤT NHỊP RUNG TYPE 6: [ Rung ngắn --- Rung ngắn (CÒI XE) ]");
        digitalWrite(MOTOR_PIN, HIGH); delay(180);
        digitalWrite(MOTOR_PIN, LOW);  delay(120);
        digitalWrite(MOTOR_PIN, HIGH); delay(180);
        digitalWrite(MOTOR_PIN, LOW);
    }

    // Xả rác bộ đệm I2S 1.5 giây
    Serial.println("Đưa mic vào chế độ xả rác 1.5 giây...");
    unsigned long mute_start = millis();
    size_t dummy_bytes;
    while (millis() - mute_start < 1500) {
        i2s_read(I2S_NUM_0, &sample_buffer, sizeof(sample_buffer), &dummy_bytes, pdMS_TO_TICKS(50));
        delay(1); 
    }
    Serial.println("Đã xả rác xong. Vòng tay tiếp tục bảo vệ...\n");
}

void setup() {
    Serial.begin(115200);
    delay(2000); 
    
    Serial.println("\n\n=========================================================");
    Serial.println(" VÒNG TAY CẢNH BÁO: QUÉT ĐỈNH DẢI BÁO CHÁY + GRACE PERIOD 300MS");
    Serial.println("=========================================================");

    pinMode(MOTOR_PIN, OUTPUT);
    digitalWrite(MOTOR_PIN, LOW); 

    i2s_driver_uninstall(I2S_NUM_0); 
    if (i2s_driver_install(I2S_NUM_0, &i2s_config, 0, NULL) != ESP_OK) {
        Serial.println("Lỗi I2S Driver!"); while (true);
    }
    if (i2s_set_pin(I2S_NUM_0, &pin_config) != ESP_OK) {
        Serial.println("Lỗi I2S Pin!"); while (true);
    }
    
    Serial.println("Đang làm nóng mic và xả nhiễu tĩnh điện (2 giây)...");
    unsigned long flush_start = millis();
    size_t dummy_bytes;
    while(millis() - flush_start < 2000) {
        i2s_read(I2S_NUM_0, &sample_buffer, sizeof(sample_buffer), &dummy_bytes, pdMS_TO_TICKS(50));
        delay(1);
    }
    
    Serial.println("Hệ thống ổn định! Sẵn sàng trực chiến...\n");
}

void loop() {
    size_t bytes_read;
    esp_err_t result = i2s_read(I2S_NUM_0, &sample_buffer, sizeof(sample_buffer), &bytes_read, pdMS_TO_TICKS(50));

    if (result == ESP_OK && bytes_read > 0) {
        int samples_read = bytes_read / 4; 
        
        // 1. Trừ DC Offset
        long dc_sum = 0;
        for (int i = 0; i < samples_read; i++) { dc_sum += (sample_buffer[i] >> 14); }
        int dc_offset = dc_sum / samples_read;

        // 2. Tính năng lượng trung bình (avg_energy) & Chuẩn hóa mảng FFT
        long energy = 0;
        for (int i = 0; i < samples_read; i++) {
            int val = (sample_buffer[i] >> 14) - dc_offset; 
            vReal[i] = (float)val; 
            vImag[i] = 0.0;        
            energy += abs(val);
        }
        
        int avg_energy = energy / samples_read;

        // ================= LOGIC 3: TIẾNG NỔ / VA ĐẬP MẠNH (STA/LTA 2 CẤP LOG) =================
        energy_history[history_index] = avg_energy;
        history_index++;
        if (history_index >= HISTORY_SIZE) {
            history_index = 0;
            history_filled = true; 
        }

        int valid_samples = history_filled ? HISTORY_SIZE : history_index; 

        if (valid_samples >= 10) {
            long sum_log1_short = 0; 
            long sum_log2_long = 0;  

            int num_short = 3;
            int num_long = valid_samples - num_short;

            for(int i = 0; i < valid_samples; i++) {
                int idx = (history_index - 1 - i + HISTORY_SIZE) % HISTORY_SIZE;
                
                if (i < num_short) {
                    sum_log1_short += energy_history[idx];
                } else {
                    sum_log2_long += energy_history[idx];
                }
            }

            int avg_log1 = sum_log1_short / num_short;
            int avg_log2 = sum_log2_long / num_long;

            if (avg_log1 > 30000 && avg_log1 > (avg_log2 * 6)) {
                Serial.printf("[Tiếng Nổ] Đột biến! Log1(Tức thời): %d | Log2(Nền): %d\n", avg_log1, avg_log2);
                
                triggerVibration("TIẾNG NỔ / VA ĐẬP MẠNH", 0, 5); 
                
                memset(energy_history, 0, sizeof(energy_history));
                history_index = 0;
                history_filled = false;
            }
        }
        // ================= KẾT THÚC LOGIC 3 =================

        float peakFreq = 0;
        bool isFireSound = false;
        bool isCo2Sound = false;
        bool isHornFrame = false;
        float bandRatio = 0.0f;
        float fireToneRatio = 0.0f;
        float co2PurityRatio = 0.0f;
        double frameCo2PeakEnergy = 0;

        // Thực thi FFT khi năng lượng vượt ngưỡng cắt đuôi tiếng vang (> 500)
        if (avg_energy > 500) { 
            FFT.windowing(FFTWindow::Hamming, FFTDirection::Forward); 
            FFT.compute(FFTDirection::Forward);                       
            FFT.complexToMagnitude();                        
            
            double binWidth = samplingFrequency / (double)samples; // 16000 / 512 = 31.25 Hz/Bin

            // ================= LOGIC 1: QUÉT ĐỈNH TRỰC TIẾP DẢI BÁO CHÁY (4.8k - 5.3kHz) =================
            int startFireBin = (int)(4800.0f / binWidth); // Bin ~153
            int endFireBin   = (int)(5300.0f / binWidth); // Bin ~169
            
            int targetFireBin = -1;
            double maxFireMag = 0;

            // Tìm đỉnh biên độ cao nhất nằm TRONG CHÍNH DẢI 4.8k - 5.3kHz
            for (int b = startFireBin; b <= endFireBin; b++) {
                if (b < samples / 2) {
                    if (vReal[b] > maxFireMag) {
                        maxFireMag = vReal[b];
                        targetFireBin = b;
                    }
                }
            }

            // Nếu tìm thấy đỉnh còi có biên độ đủ lớn
            if (targetFireBin != -1 && maxFireMag > 80.0) { 
                // Gom 3 Bin xung quanh đỉnh này
                double firePeakEnergy = 0;
                for (int b = targetFireBin - 1; b <= targetFireBin + 1; b++) {
                    if (b >= 0 && b < samples / 2) {
                        firePeakEnergy += vReal[b];
                    }
                }
                
                // Gom tổng năng lượng dải rộng 4000Hz - 6000Hz
                double highBandEnergy = 0;
                int startHighBin = (int)(4000.0f / binWidth);
                int endHighBin   = (int)(6000.0f / binWidth);
                for (int b = startHighBin; b <= endHighBin; b++) {
                    if (b < samples / 2) {
                        highBandEnergy += vReal[b];
                    }
                }
                
                if (highBandEnergy > 0) {
                    fireToneRatio = (float)(firePeakEnergy / highBandEnergy);
                }
                
                if (fireToneRatio >= MIN_FIRE_TONE_RATIO) { 
                    isFireSound = true;
                    peakFreq = targetFireBin * binWidth; // Tần số thực tế của còi
                }
            }

            // Dùng majorPeak() cho các logic khác
            float globalPeakFreq = FFT.majorPeak();

            // ================= LOGIC 2: ĐỈNH BÁO CO2 (3.3k - 3.4kHz) =================
            if (globalPeakFreq >= 3300.0f && globalPeakFreq <= 3400.0f) {
                int f0_Bin = (int)(globalPeakFreq / binWidth);
                
                double f0_PeakEnergy = 0;
                for (int b = f0_Bin - 1; b <= f0_Bin + 1; b++) {
                    if (b >= 0 && b < samples / 2) {
                        f0_PeakEnergy += vReal[b];
                    }
                }

                double co2BandEnergy = 0;
                int startCo2Bin = (int)(3000.0f / binWidth);
                int endCo2Bin   = (int)(4000.0f / binWidth);
                for (int b = startCo2Bin; b <= endCo2Bin; b++) {
                    if (b < samples / 2) {
                        co2BandEnergy += vReal[b];
                    }
                }

                if (co2BandEnergy > 0) {
                    co2PurityRatio = (float)(f0_PeakEnergy / co2BandEnergy);
                }

                if (co2PurityRatio >= MIN_CO2_PURITY_RATIO) { 
                    isCo2Sound = true;
                    frameCo2PeakEnergy = f0_PeakEnergy;
                }
            }

            // ================= LOGIC 4: CÒI XE GIAO THÔNG PHẠM VI < 5M =================
            if (avg_energy >= HORN_ENERGY_5M && globalPeakFreq >= MIN_HORN_FREQ && globalPeakFreq <= MAX_HORN_FREQ) {
                double totalMagnitude = 0;
                double hornBandMagnitude = 0;

                int startHornBin = (int)(MIN_HORN_FREQ / binWidth); 
                int endHornBin   = (int)(MAX_HORN_FREQ / binWidth); 
                int halfSamples  = samples / 2;

                for (int i = 1; i < halfSamples; i++) { 
                    double mag = vReal[i];
                    totalMagnitude += mag;
                    if (i >= startHornBin && i <= endHornBin) {
                        hornBandMagnitude += mag;
                    }
                }

                if (totalMagnitude > 0) {
                    bandRatio = (float)(hornBandMagnitude / totalMagnitude);
                }

                if (bandRatio >= MIN_BAND_RATIO) {
                    isHornFrame = true;
                }
            }
        }

        // ================= XỬ LÝ QUYẾT ĐỊNH LOGIC THỜI GIAN =================

        // LUỒNG 1: BÁO CHÁY (CONFIDENCE SCORE + GRACE PERIOD 300MS)
        if (isFireSound) {
            lastFireFrameTime = millis(); // Lưu lại thời điểm gần nhất có frame chuẩn
            
            fireConfidence += 2; // Tăng 2 điểm mỗi frame hợp lệ
            if (fireConfidence > FIRE_MAX_SCORE) fireConfidence = FIRE_MAX_SCORE;

            Serial.printf("[Báo cháy] Frame đạt (%.0f Hz | Ratio: %.2f) -> Điểm tin cậy: %d/%d\n", 
                          peakFreq, fireToneRatio, fireConfidence, FIRE_TRIGGER_SCORE);

            // Khi điểm tích lũy chạm ngưỡng (>= 8 điểm) -> XUẤT LỆNH RUNG
            if (fireConfidence >= FIRE_TRIGGER_SCORE) {
                triggerVibration("CÒI BÁO CHÁY CHUẨN (HỌA ÂM TONE)", peakFreq, 3);
                fireConfidence = 0; 
            }
        } else {
            // CHỈ TRỪ ĐIỂM KHI MẤT TÍN HIỆU CÒI LIÊN TỤC QUÁ 300MS (GRACE PERIOD)
            if (fireConfidence > 0 && (millis() - lastFireFrameTime > FIRE_GRACE_MS)) {
                fireConfidence--;
                if (fireConfidence == 0) {
                    Serial.println("   -> Điểm tin cậy về 0 (Đứt tiếng còi quá 300ms)");
                }
            }
        }

        // LUỒNG 2: BÁO CO2 (Peak Hold trong nhịp + Đối soát 3 đỉnh lệch < 5%)
        if (isCo2Sound) {
            lastCo2DetectTime = millis();
            
            if (!isCo2Beeping) {
                isCo2Beeping = true; 
                
                if (co2BeepCount == 0) {
                    co2WindowStartTime = millis(); 
                }
                
                co2BeepPeakEnergy[co2BeepCount] = frameCo2PeakEnergy;
                co2BeepCount++;
                
                Serial.printf("[Báo CO2] Bắt đầu nhịp %d/3 (Purity: %.2f)\n", 
                              co2BeepCount, co2PurityRatio);
            } else {
                int currentIdx = co2BeepCount - 1;
                if (currentIdx >= 0 && currentIdx < 3) {
                    if (frameCo2PeakEnergy > co2BeepPeakEnergy[currentIdx]) {
                        co2BeepPeakEnergy[currentIdx] = frameCo2PeakEnergy;
                    }
                }
            }
        } else {
            if (isCo2Beeping && (millis() - lastCo2DetectTime > 80)) {
                isCo2Beeping = false;
                Serial.printf("   -> Hoàn tất nhịp %d/3 (Đỉnh năng lượng: %.0f)\n", 
                              co2BeepCount, co2BeepPeakEnergy[co2BeepCount - 1]);

                if (co2BeepCount == 3) {
                    double eMin = co2BeepPeakEnergy[0];
                    double eMax = co2BeepPeakEnergy[0];
                    for (int k = 1; k < 3; k++) {
                        if (co2BeepPeakEnergy[k] < eMin) eMin = co2BeepPeakEnergy[k];
                        if (co2BeepPeakEnergy[k] > eMax) eMax = co2BeepPeakEnergy[k];
                    }

                    float energyDiff = (eMax > 0) ? (float)((eMax - eMin) / eMax) : 1.0f;

                    if (energyDiff <= MAX_CO2_ENERGY_DIFF) { 
                        Serial.printf("\n[!!!] XÁC NHẬN CÒI CO2 THẬT: 3 đỉnh lệch %.2f%% (Đạt chuẩn < 5%%)\n", energyDiff * 100.0f);
                        triggerVibration("CÒI BÁO KHÍ CO2 (3 ĐỈNH CHUẨN LỆCH < 5%)", 3350, 3);
                    } else {
                        Serial.printf("   -> HỦY BÁO CO2: Năng lượng 3 đỉnh lệch %.1f%% (> 5%%)\n", energyDiff * 100.0f);
                    }

                    co2BeepCount = 0; 
                }
            }
        }

        if (co2BeepCount > 0 && (millis() - co2WindowStartTime > 5000)) {
            co2BeepCount = 0;
            isCo2Beeping = false;
            Serial.println("   -> Hủy Báo CO2 (Quá 5 giây không gom đủ 3 nhịp)");
        }

        // LUỒNG 4: CÒI XE GIAO THÔNG < 5M
        if (isHornFrame) {
            lastHornDetectTime = millis();

            if (hornSustainStart == 0) {
                hornSustainStart = millis();
                Serial.printf("[Còi xe < 5m] Phát hiện (Ratio: %.2f | Energy: %d) -> Đang theo dõi...\n", 
                              bandRatio, avg_energy);
            } else {
                unsigned long elapsed = millis() - hornSustainStart;
                
                if (elapsed >= MIN_SUSTAIN_MS) {
                    triggerVibration("CÒI XE NGUY HIỂM (< 5M)", 1000, 6);
                    hornSustainStart = 0; 
                }
            }
        } else {
            if (hornSustainStart > 0 && (millis() - lastHornDetectTime > 100)) {
                hornSustainStart = 0;
                Serial.println("   -> Bỏ qua Còi xe (Tiếng ồn thoáng qua / Âm thanh ngắt đoạn)");
            }
        }
    }
    
    delay(5);
}