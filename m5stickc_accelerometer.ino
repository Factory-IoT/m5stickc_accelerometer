// m5stickc_accelerometer
// M5StickCで振動測定を行い、BTでシリアル送信する。
// Read data by Vanalysis ver 0.1.0 or higher.
// ver 1.0.0


#include <M5StickC.h>
#include "BluetoothSerial.h"

//測定条件設定項目
#define SAMPLE_RATE 333             // サンプリング間隔(μsec) SAMPLE_RATE = 1000000(μsec)/サンプリング周波数(Hz)
#define SAMPLE_SIZE 1024            // 一回の測定のサンプリング数。　※FFTを行う際は1024推奨（2のべき乗にすること）。m5stackcの画面でグラフ確認するだけなら150推奨。
#define SAMPLE_NUM 5                // 最大測定回数。10推奨。大きくしすぎるとメモリが不足します。

//その他設定
#define I2C_SPEED 500000             // デバイス内のI2C通信速度。仕様上は400000が最大だが、500000でも動く。
#define X0 5                        // 横軸の描画開始座標
#define MINZ 0                      // 縦軸の最小値
#define MAXZ 2000                   // 縦軸の最大値
#define MAXX 150                    // 横軸の最大値

//M5StickC IO設定
#define BTN_A_PIN 37
#define BTN_B_PIN 39
#define LED_PIN 10
#define LED_ON LOW
#define LED_OFF HIGH
#define BTN_ON LOW
#define BTN_OFF HIGH

BluetoothSerial bts;
float ax[SAMPLE_SIZE][SAMPLE_NUM], ay[SAMPLE_SIZE][SAMPLE_NUM], az[SAMPLE_SIZE][SAMPLE_NUM];  // 加速度データの保存先
long samplingTime[SAMPLE_SIZE][SAMPLE_NUM];         //サンプリング周期の保存先
int mesureNum = 0;                                  //測定済み回数
boolean serialMoniter = 0;                          //シリアルモニターへ出力する場合は1

//レジスタ書き込み用関数　ヘッダファイル変更できる方は mpu6886.hの同じ関数をpublic化して使っても可。
void I2C_Write_NBytes(uint8_t driver_Addr, uint8_t start_Addr, uint8_t number_Bytes, uint8_t *write_Buffer){
  Wire1.beginTransmission(driver_Addr);
  Wire1.write(start_Addr);
  Wire1.write(*write_Buffer);
  Wire1.endTransmission();
}

//加速度データ取得
void getAccel(int j){
    int i = 0;  
    long t = 0;
    long dT;

    M5.Lcd.fillScreen(BLACK);  // 画面をクリア
    M5.Lcd.setCursor(0,0);
    M5.Lcd.print("3 ");
    delay(500);
    M5.Lcd.print("> ");
    delay(500);
    M5.Lcd.print("2 ");
    delay(500);
    M5.Lcd.print("> ");
    delay(500);
    M5.Lcd.print("1 ");
    delay(500);
    M5.Lcd.println("> ");
    delay(500);
    
    t = micros();
    while (i < SAMPLE_SIZE) {
        dT = micros() - t;
        if (dT >= SAMPLE_RATE){
            t = micros();
            M5.MPU6886.getAccelData(&ax[i][j],&ay[i][j],&az[i][j]);  // MPU6886から加速度を取得

            ax[i][j] *= 1000;
            ay[i][j] *= 1000;
            az[i][j] *= 1000;

            if (i % MAXX > 1) {
                int y0 = map((int)(az[i - 1][j]), MINZ, MAXZ, M5.Lcd.height(), 0);
                int y1 = map((int)(az[i][j]), MINZ, MAXZ, M5.Lcd.height(), 0);
                M5.Lcd.drawLine(i % MAXX - 1 + X0, y0, i % MAXX + X0, y1, GREEN);
            }
            samplingTime[i][j] = dT;
            i += 1;
        }
    }
    M5.Lcd.print(SAMPLE_SIZE);
    M5.Lcd.print(" cycles completed.");
    M5.Lcd.setCursor(0,73);
    M5.Lcd.print("Button A to return menu");
    while(digitalRead(BTN_A_PIN) == BTN_OFF){
       delay(100);
    }
}

//シリアルモニターへデータ出力
void Serialout(){
    digitalWrite(LED_PIN, LED_ON);

    M5.Lcd.setCursor(0,70);
    M5.Lcd.print("Data sending... ");
    for (int j = 0;j< mesureNum; j++){
        for (int i = 0; i < SAMPLE_SIZE; i++){
            Serial.print("測定周期(μsec)：");
            Serial.print(samplingTime[i][j]);
            Serial.print(" , x軸方向加速度(mG)：");
            Serial.print(ax[i][j]);
            Serial.print(" , y軸方向加速度(mG)：");
            Serial.print(ay[i][j]);
            Serial.print(" , Z軸方向加速度(mG)：");
            Serial.print(az[i][j]);
            Serial.print(" , サンプル番号：");
            Serial.println(i);  
        }
    }
    digitalWrite(LED_PIN, LED_OFF);
    M5.Lcd.print("completed.");
}

//Bluetoothでデータ出力
void BTsend(){
    int BTcheck = 0;
    M5.Lcd.fillScreen(BLACK);
    M5.Lcd.setCursor(0,0);
    M5.Lcd.println("Bluetooth check.");
    digitalWrite(LED_PIN, LED_ON);

    bts.println(255);
    delay(1000);
    if(bts.available()){
        BTcheck = bts.read();
        M5.Lcd.println(BTcheck);
        if (BTcheck == 255){
            bts.println(SAMPLE_SIZE);
            bts.println(mesureNum);
            M5.Lcd.println("Data sending");
            for (int j = 0; j < mesureNum; j++){
                for (int i = 0; i < SAMPLE_SIZE; i++){
                    bts.println(samplingTime[i][j]);
                    bts.println(ax[i][j]);
                    bts.println(ay[i][j]);
                    bts.println(az[i][j]);
                }
            }
        } else {
            M5.Lcd.println("failure");
        }
        delay(2000);

    } else {
        M5.Lcd.println("PC not found.");
        delay(2000);
    }
    digitalWrite(LED_PIN, LED_OFF);
}

//メニュー画面表示
void menu(){
    M5.Lcd.fillScreen(BLACK);  // 画面をクリア
    M5.Lcd.setCursor(0,2);
    M5.Lcd.print("Number of mesurements : ");
    M5.Lcd.println(mesureNum);
    M5.Lcd.println("");
    M5.Lcd.println("Button A to mesure.");
    M5.Lcd.println("Button B to data sending.");
}


void setup() {
    M5.begin();

//通信回りの初期化、設定
    Serial.begin(115200);

    bts.begin("M5StickC_01");
    Wire1.setClock(I2C_SPEED);                              // オンボードデバイスのI2C通信速度設定

//M5StickCの入出力定義
    M5.Lcd.setRotation(3);
    pinMode(BTN_A_PIN, INPUT_PULLUP);                       //ボタン、LED設定
    pinMode(BTN_B_PIN, INPUT_PULLUP);
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LED_OFF);

//加速度センサ　MPU6886の測定モード設定
    M5.MPU6886.Init();                                      // MPU6886を初期化

    unsigned char buf;
    unsigned char regdata;

    regdata = 0x00;                                         //サンプルの間引きを0に変更。
    I2C_Write_NBytes(MPU6886_ADDRESS,0x19,1,&regdata);      //SAMPLE LATE DIVIDER,デフォルトはregdata = 0x05

    regdata = 0x07;                                         //サンプリングレートアップ
    I2C_Write_NBytes(MPU6886_ADDRESS,0x1A,1,&regdata);      //CONFIG,デフォルトは 0x01

    regdata = 0x08;                                         //加速度のLPFをバイパスしてサンプリングレートアップ
    I2C_Write_NBytes(MPU6886_ADDRESS,0x1D,1,&regdata);      //ACCEL_CONFIG_2,デフォルトは0x00

    menu();
}

void loop() {
    if (digitalRead(BTN_A_PIN) == BTN_ON){
        if( mesureNum < SAMPLE_NUM ){
            getAccel(mesureNum);
            mesureNum += 1;
        }else{
            M5.Lcd.println("Data is full.");
            M5.Lcd.println("Output the data pls.");
            delay(3000);
        }
        menu();
        delay(1000);
    }
    
    if (digitalRead(BTN_B_PIN) == BTN_ON){
        if (serialMoniter == true){
            Serialout();                      //arduino IDEのシリアルモニター出力
        }
        BTsend();                             //bluetooth出力

        menu();
        delay(1000);
    }
}
