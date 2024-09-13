#include <Arduino.h>
#include<timer.h>
#include<move_ave/MA.h>
#include<Vector/myVector.h>

int ballPin[16] = {10,2,14,15,16,17,18,19,11,3,4,5,6,7,8,9};

double ang;

int ball_num[16];
int ball_g[2];
int ball_down[4];

uint8_t ball_8bit[16];
uint8_t ball_get_8[2];
uint8_t ball_down_8bit[4];

Vector2D ele[16];

int16_t x,y;
int ball_get;
timer Timer_ball;
timer Timer;
int Time;
MA ma;
int A;

void ball();
void ball_print();
void led();
int LED = 13;
int Serial_flag = 0; //0だったらメインに通信　1だったらデバッグ

// B6 B5 D7 D6 D4 D5 D0 B7 B3 B2 B1 F0 F1 F4 F5 F7 これは普通のボールのやつ
// B4 D1 B0 F6 これは下向きのやつ

void setup() {
  Serial1.begin(115200);
  Serial.begin(9600);
  DDRB &= ~(_BV(6) | _BV(5) | _BV(7) | _BV(3) | _BV(2) | _BV(1) | _BV(4) | _BV(0));
  DDRD &= ~(_BV(7) | _BV(6) | _BV(4) | _BV(5) | _BV(0) | _BV(1));
  DDRF &= ~(_BV(0) | _BV(1) | _BV(4) | _BV(5) | _BV(7) | _BV(6));
  for(int i = 0; i < 16; i++){
    ele[i].set_polar(22.5 * i, 1.0);
  }
}

void loop() {
  int sendBuf_int[3];
  byte sendBuf_byte[12] = {0xFF,0,0,0,0,0,0,0,0,0,0,0xAA};
  //データを格納
  sendBuf_int[1] = x;
  sendBuf_int[2] = y;
  //データをバイトに直す
  sendBuf_byte[0] = 0xFF;
  sendBuf_byte[1] = byte( sendBuf_int[1] >> 8 ); //ビットシフトで上位側の８Bitを取り出し、バイト型に型変換をする。
  sendBuf_byte[2] = byte( sendBuf_int[1] & 0xFF ); //論理和で下位側の８Bitを取り出し、バイト型に型変換をする。
  sendBuf_byte[3] = byte( sendBuf_int[2] >> 8 ); //ビットシフトで上位側の８Bitを取り出し、バイト型に型変換をする。
  sendBuf_byte[4] = byte( sendBuf_int[2] & 0xFF ); //論理和で下位側の８Bitを取り出し、バイト型に型変換をする。
  sendBuf_byte[5] = ball_g[0];
  sendBuf_byte[6] = ball_g[1];
  sendBuf_byte[7] = ball_down[0];
  sendBuf_byte[8] = ball_down[1];
  sendBuf_byte[9] = ball_down[2];
  sendBuf_byte[10] = ball_down[3];
  sendBuf_byte[11] = 0xAA;
  Serial1.write(sendBuf_byte,12);
  ball();
  // ball_print();
}

void ball_print(){
  // Time = Timer.read_us();
  // Serial.print(" x : ");
  // Serial.print(x);
  // Serial.print(" y : ");
  // Serial.print(y);
  // Serial.print(" time : ");
  // Serial.print(Time);
  // Serial.print(" ang : ");
  // Serial.print(degrees(atan2(y,x)));
  // Serial.print(" 0 : ");
  // Serial.print(ball_g[0]);
  // Serial.print(" 1 : ");
  // Serial.print(ball_g[1]);
  // Serial.print(" A : ");
  // Serial.print(A);
  // Serial.print(" get : ");
  // Serial.print(ball_get);
  // for(int i = 0; i < 16; i++){
  //   Serial.print(" ");
  //   Serial.print(ball_num[i]);
  // }

  // for(int i = 0; i < 4; i++){
  //   Serial.print(" ");
  //   Serial.print(ball_down[i]);
  // }
  Serial.println();
  Timer.reset();
}

void ball() {
  Vector2D ball_;
  ball_g[0] = 0;
  ball_g[1] = 0;
  for(int i = 0; i < 16; i++){
    ball_num[i] = 0;
  }
  for(int i = 0; i < 4; i++){
    ball_down[i] = 0;
  }
  int best_num;
  int best_val = 0;

  Timer_ball.reset();

// B6 B5 D7 D6 D4 D5 D0 B7 B3 B2 B1 F0 F1 F4 F5 F7 これは普通のボールのやつ
// B4 D1 B0 F6 これは下向きのやつ

  for(int i = 0; i < 80; i++){
    ball_8bit[0] = PINB & _BV(6);
    ball_8bit[1] = PINB & _BV(5);
    ball_8bit[2] = PIND & _BV(7);
    ball_8bit[3] = PIND & _BV(6);
    ball_8bit[4] = PIND & _BV(4);
    ball_8bit[5] = PIND & _BV(5);
    ball_8bit[6] = PIND & _BV(0);
    ball_8bit[7] = PINB & _BV(7);
    ball_8bit[8] = PINB & _BV(3);
    ball_8bit[9] = PINB & _BV(2);
    ball_8bit[10] = PINB & _BV(1);
    ball_8bit[11] = PINF & _BV(0);
    ball_8bit[12] = PINF & _BV(1);
    ball_8bit[13] = PINF & _BV(4);
    ball_8bit[14] = PINF & _BV(5);
    ball_8bit[15] = PINF & _BV(7);

    ball_down_8bit[0] = PINB & _BV(4);
    ball_down_8bit[1] = PIND & _BV(1);
    ball_down_8bit[2] = PINB & _BV(0);
    ball_down_8bit[3] = PINF & _BV(6);

    // ball_get_8[0] = PINB & _BV(5);
    // ball_get_8[1] = PINB & _BV(4);

    for(int i = 0; i < 16; i++){
      if(ball_8bit[i] == 0){
        ball_num[i]++;
      }
    }

    for(int i = 0; i < 4; i++){
      if(ball_down_8bit[i] == 0){
        ball_down[i]++;
      }
    }

    if(ball_get_8[0] == 0){
      ball_g[0]++;
    }
    if(ball_get_8[1] == 0){
      ball_g[1]++;
    }
  }

  for(int i = 0; i < 16; i++){
    if(ball_num[i] == 80){
      ball_num[i] = 0;
    }
    if(best_val < ball_num[i]){
      best_val = ball_num[i];
      best_num = i;
    }
  }

  for(int i = -4; i <= 4; i++){
    int num = best_num + i;
    if(num < 0){
      num += 16;
    }
    else if(15 < num){
      num -= 16;
    }
    if(ball_num[num] == 250){
      ball_num[num] = 1000;
    }
    ball_ = ball_ + ele[num] * ball_num[num];
  }

  // A = ma.demandAve(ball_g[0] + ball_g[1]);
  A = ball_g[0] + ball_g[1];
  if(70 < A){
    ball_get = 1;
  }
  else if(60 < A){
    ball_get = 2;
  }
  else{
    ball_get = 0;
  }
  x = ball_.return_x();
  y = ball_.return_y();
}