#include "Car.hpp"

PIDParam turnPIDParam  = { 3, 0, 0 };
PIDParam splitPIDParam = { 100, 0, 1000 };
PIDParam rollPIDParam  = { 100, 0, 1000 };

Car::Car() {
  this->legL   = new Leg(LEFT);
  this->legR   = new Leg(RIGHT);
  this->legVir = new Leg(LEFT);
  this->sensor = new Sensor();
  this->turnPID.init(PIDPOSMODE, turnPIDParam, 0.03, 0.1);
  this->splitPID.init(PIDPOSMODE, splitPIDParam, 0.01, 1);
  this->rollPID.init(PIDPOSMODE, turnPIDParam, 0.01, 0.1);
  this->rollPID.setTarget(0);
  this->turnPID.setTarget(0);
  this->splitPID.setTarget(0);

  this->SplitKCoeff << -54.1021, 90.6528, -84.5271, -10.4081,
    -1.7042, -0.2075, -12.5042, -1.8305,
    -15.8197, 25.0691, -14.1572, -2.4042,
    -16.6639, 26.7676, -17.0888, -4.2684,
    -18.8416, 36.4423, -27.835, 10.3184,
    -9.0931, 16.8012, -12.2652, 4.5087,
    -2.048, 26.753, -40.9553, 30.9467,
    -4.448, 9.3187, -7.2037, 6.1261,
    -20.6399, 39.9205, -30.4917, 11.3032,
    -33.8231, 62.237, -45.0024, 16.2261,
    57.7654, -91.5392, 51.6946, 8.7788,
    27.9553, -43.9464, 24.6129, 1.5981;

  this->WBCKCoeff << -1.5782, -2.3503, 0.2456, 2.8044, -0.5538, -0.3792,
    -2.3012, -2.4472, 0.6088, 3.0067, -0.9434, -0.696,
    -1.7025, -1.7614, -0.1611, 1.1078, 0.4748, 0.1353,
    -0.6718, -0.8143, 0.0887, 0.5051, 0.089, 0.0446,
    -5.6233, -33.9127, 11.197, 25.7004, -12.3731, 12.9587,
    -2.1591, -2.472, 1.4636, 0.6844, -2.3293, 2.2648,
    -4.7678, 13.5134, -19.3277, -9.8997, 12.2251, -8.7767,
    -0.1143, -0.251, -4.5483, -0.1645, 2.2075, -0.8787,
    5.7111, -10.6213, -1.6627, 6.8939, -1.6006, 3.6871,
    -1.5782, 0.2456, -2.3503, -0.5538, 2.8044, -0.3792,
    1.5132, -0.2118, -0.983, -0.518, 0.6601, 0.4045,
    -2.3012, 0.6088, -2.4472, -0.9434, 3.0067, -0.696,
    1.7025, 0.1611, 1.7614, -0.4748, -1.1078, -0.1353,
    0.6718, -0.0887, 0.8143, -0.089, -0.5051, -0.0446,
    -4.7678, -19.3277, 13.5134, 12.2251, -9.8997, -8.7767,
    -0.1143, -4.5483, -0.251, 2.2075, -0.1645, -0.8787,
    -5.6233, 11.197, -33.9127, -12.3731, 25.7004, 12.9587,
    -2.1591, 1.4636, -2.472, -2.3293, 0.6844, 2.2648,
    5.7111, -1.6627, -10.6213, -1.6006, 6.8939, 3.6871,
    1.1684, -0.6895, -1.8163, -0.0233, 0.9931, 0.9174,
    3.4869, 5.0171, -12.4427, -7.1564, 3.2676, 9.4463,
    4.6614, 5.0502, -15.4945, -7.5485, 3.907, 11.7847,
    -3.0347, 4.4495, 1.1615, -3.7765, -0.9687, 4.441,
    -1.2444, 1.8184, 0.2787, -1.4727, -0.1415, 1.9262,
    9.2991, 46.4478, -2.144, -48.64, -13.5728, 74.2038,
    5.3844, -5.3733, 0.8031, 5.7666, -3.9449, 6.7014,
    8.8266, -16.0393, -60.4417, 20.639, 17.617, -28.6534,
    -0.0412, 2.2207, -7.3757, -0.258, -1.1245, -0.1112,
    9.8365, 28.2392, -14.3279, -21.4837, 5.5521, 3.5966,
    1.3797, 4.4538, -1.8205, -2.8596, 0.5427, -0.0419,
    3.4869, -12.4427, 5.0171, 3.2676, -7.1564, 9.4463,
    4.6614, -15.4945, 5.0502, 3.907, -7.5485, 11.7847,
    3.0347, -1.1615, -4.4495, 0.9687, 3.7765, -4.441,
    1.2444, -0.2787, -1.8184, 0.1415, 1.4727, -1.9262,
    8.8266, -60.4417, -16.0393, 17.617, 20.639, -28.6534,
    -0.0412, -7.3757, 2.2207, -1.1245, -0.258, -0.1112,
    9.2991, -2.144, 46.4478, -13.5728, -48.64, 74.2038,
    5.3844, 0.8031, -5.3733, -3.9449, 5.7666, 6.7014,
    9.8365, -14.3279, 28.2392, 5.5521, -21.4837, 3.5966,
    1.3797, -1.8205, 4.4538, 0.5427, -2.8596, -0.0419;

  this->jumpPhase = OFF;
  this->flyflag   = false;
}

Car::~Car() {
  delete this->legL;
  delete this->legR;
  delete this->sensor;
}

void Car::update() {
  this->sensor->update();
  this->legL->update();
  this->legR->update();
  this->legL->zjie(this->sensor->pitch.now);
  this->legR->zjie(this->sensor->pitch.now);
  this->legVir->zjie(this->sensor->pitch.now, (this->legL->angle1 + this->legR->angle1) / 2.f, (this->legL->angle4 + this->legR->angle4) / 2.f);
  this->legVir->dis.now = (this->legL->dis.now + this->legR->dis.now) / 2.f;
  this->legVir->dis.dot = (this->legL->dis.dot + this->legR->dis.dot) / 2.f;
  this->flyCheck();
  this->JumpControl();
}

void Car::flyCheck() {
  float lp          = this->legL->Fnow * cos(this->legL->theta.now) + this->legL->Tbnow * sin(this->legL->theta.now) / this->legL->L0.now;
  float rp          = this->legR->Fnow * cos(this->legR->theta.now) + this->legR->Tbnow * sin(this->legR->theta.now) / this->legR->L0.now;

  float zmdd        = this->sensor->accelz * cos(this->sensor->pitch.now) - this->sensor->accelx * sin(this->sensor->pitch.now);

  float lzwdd       = zmdd - this->legL->L0.ddot * cos(this->legL->theta.now) + 2 * this->legL->L0.dot + this->legL->theta.dot * sin(this->legL->theta.now) + this->legL->L0.now * this->legL->theta.ddot * sin(this->legL->theta.now) + this->legL->L0.now * this->legL->theta.dot * this->legL->theta.dot * cos(this->legL->theta.now);
  float rzwdd       = zmdd - this->legR->L0.ddot * cos(this->legR->theta.now) + 2 * this->legR->L0.dot + this->legR->theta.dot * sin(this->legR->theta.now) + this->legR->L0.now * this->legR->theta.ddot * sin(this->legR->theta.now) + this->legR->L0.now * this->legR->theta.dot * this->legR->theta.dot * cos(this->legR->theta.now);

  this->legR->force = rp + MASSWHEEL * (GRAVITY + rzwdd);
  this->legL->force = lp + MASSWHEEL * (GRAVITY + lzwdd);

  this->force       = (this->legL->force + this->legR->force) / 2;

  if (this->force < 20)
    this->flyflag = true;
  else
    this->flyflag = false;
}
void Car::JumpControl() {
  switch (this->jumpPhase) {
    case ON:
      this->legL->L0PID.setTarget(MINROBOTLEGLEN);
      this->legR->L0PID.setTarget(MINROBOTLEGLEN);
      if (this->legL->L0.now + this->legR->L0.now <= 0.55)
        this->jumpPhase = KICK;
      break;
    case KICK:
      this->legL->L0PID.setTarget(MAXROBOTLEGLEN);
      this->legR->L0PID.setTarget(MAXROBOTLEGLEN);
      if (this->legL->L0.now + this->legR->L0.now >= 1.05)
        this->jumpPhase = SHRINK;
      break;
    case SHRINK:
      this->legL->L0PID.setTarget(0.32);
      this->legR->L0PID.setTarget(0.32);
      if (this->force > 100)
        this->jumpPhase = BUFFER;
      break;
    case BUFFER:
      this->legL->L0PID.setTarget(0.32);
      this->legR->L0PID.setTarget(0.32);
      this->jumpPhase = OFF;
      break;
    case OFF:
      break;
  }
}

void Car::SplitLQRControl() {
  float L01L = this->legL->L0.now;
  float L02L = L01L * L01L;
  float L03L = L02L * L01L;

  float L01R = this->legR->L0.now;
  float L02R = L01R * L01R;
  float L03R = L02R * L01R;

  if (this->flyflag) {
    for (int row = 0; row < 2; row++) {
      for (int col = 0; col < 6; col++) {
        int num = (row * 6) + col;
        if (row == 1 && (col == 0 || col == 1)) {
          this->legL->KSplit(row, col) = this->SplitKCoeff(num, 0) * L03L + this->SplitKCoeff(num, 1) * L02L + this->SplitKCoeff(num, 2) * L01L + this->SplitKCoeff(num, 3);
          this->legR->KSplit(row, col) = this->SplitKCoeff(num, 0) * L03R + this->SplitKCoeff(num, 1) * L02R + this->SplitKCoeff(num, 2) * L01R + this->SplitKCoeff(num, 3);
        } else {
          this->legL->KSplit(row, col) = 0;
          this->legR->KSplit(row, col) = 0;
        }
      }
    }
  } else {
    for (int row = 0; row < 2; row++) {
      for (int col = 0; col < 6; col++) {
        int num                      = (row * 6) + col;
        this->legL->KSplit(row, col) = this->SplitKCoeff(num, 0) * L03L + this->SplitKCoeff(num, 1) * L02L + this->SplitKCoeff(num, 2) * L01L + this->SplitKCoeff(num, 3);
        this->legR->KSplit(row, col) = this->SplitKCoeff(num, 0) * L03R + this->SplitKCoeff(num, 1) * L02R + this->SplitKCoeff(num, 2) * L01R + this->SplitKCoeff(num, 3);
      }
    }
  }
  this->legL->StateSplit << this->legL->theta.now, this->legL->theta.dot, this->legL->dis.now, this->legL->dis.dot, this->sensor->pitch.now, this->sensor->pitch.dot;
  this->ExpectSplit << 0, 0, 0, 0, 0, 0;
  this->legL->InputSplit = this->legL->KSplit * (this->ExpectSplit - this->legL->StateSplit);

  this->legR->StateSplit << this->legR->theta.now, this->legR->theta.dot, this->legR->dis.now, this->legR->dis.dot, this->sensor->pitch.now, this->sensor->pitch.dot;
  this->legL->InputSplit = this->legL->KSplit * (this->ExpectSplit - this->legL->StateSplit);

  this->legL->Twset      = this->legL->InputSplit(0, 0);
  this->legR->Twset      = this->legR->InputSplit(0, 0);

  this->legL->Tbset      = this->legL->InputSplit(1, 0);
  this->legR->Tbset      = this->legR->InputSplit(1, 0);

  this->legL->Fset       = FFEEDFORWARD;
  this->legR->Fset       = FFEEDFORWARD;

  // Ðý×ª²¹³¥
  float yawCompensate    = this->turnPID.compute(this->sensor->yaw.dot);
  this->turnPID.setTarget(0);
  this->legL->Twset     -= yawCompensate;
  this->legR->Twset     += yawCompensate;
  // ÐéÄâÁ¦
  float lfCompensate     = this->legL->L0PID.compute(this->legL->L0.now);
  float rfCompensate     = this->legR->L0PID.compute(this->legR->L0.now);
  this->legL->Fset      += lfCompensate;
  this->legR->Fset      += rfCompensate;
  // ÅüÍÈ²¹³¥
  float splitCompensate  = this->splitPID.compute(this->legL->theta.now - this->legR->theta.now);
  this->legL->Tbset     += splitCompensate;
  this->legR->Tbset     -= splitCompensate;
  // ·­¹ö½Ç²¹³¥
  float rollCompensate   = this->rollPID.compute(this->sensor->roll.now);
  this->legL->Fset      += rollCompensate;
  this->legR->Fset      -= rollCompensate;

  legL->VMC();
  legR->VMC();

  legL->run();
  legR->run();
}

void Car::WbcLQRControl() {
  float L_l    = this->legL->L0.now;
  float L_r    = this->legR->L0.now;
  float L_l2   = L_l * L_l;
  float L_r2   = L_r * L_r;
  float L_lL_r = L_l * L_r;

  if (this->flyflag) {
    for (int row = 0; row < 4; row++) {
      for (int col = 0; col < 10; col++) {
        int num = (row * 10) + col;
        if ((row != 2 && row != 3) || (col != 4 && col != 5 && col != 6 && col != 7)) {
          this->KWBC(row, col) = 0;
          continue;
        }
        this->KWBC(row, col) = this->WBCKCoeff(num, 0) +
                               this->WBCKCoeff(num, 1) * L_l +
                               this->WBCKCoeff(num, 2) * L_r +
                               this->WBCKCoeff(num, 3) * L_l2 +
                               this->WBCKCoeff(num, 4) * L_r2 +
                               this->WBCKCoeff(num, 5) * L_lL_r;
      }
    }
  } else {
    for (int row = 0; row < 4; row++) {
      for (int col = 0; col < 10; col++) {
        int num              = (row * 10) + col;
        this->KWBC(row, col) = this->WBCKCoeff(num, 0) +
                               this->WBCKCoeff(num, 1) * L_l +
                               this->WBCKCoeff(num, 2) * L_r +
                               this->WBCKCoeff(num, 3) * L_l2 +
                               this->WBCKCoeff(num, 4) * L_r2 +
                               this->WBCKCoeff(num, 5) * L_lL_r;
      }
    }
  }

  this->StateWBC << this->legVir->dis.now,
    this->legVir->dis.dot,
    this->sensor->yaw.now,
    this->sensor->yaw.dot,
    this->legL->theta.now,
    this->legL->theta.dot,
    this->legR->theta.now,
    this->legR->theta.dot,
    this->sensor->pitch.now,
    this->sensor->pitch.dot;
  this->ExpectWBC << 1, 0, 0, 0, 0, 0, 0, 0, 0, 0;
  this->InputWBC         = this->KWBC * (this->ExpectWBC - this->StateWBC);

  this->legL->Twset      = this->InputWBC(0, 0);
  this->legR->Twset      = this->InputWBC(1, 0);

  this->legL->Tbset      = this->InputWBC(2, 0);
  this->legR->Tbset      = this->InputWBC(3, 0);

  this->legL->Fset       = FFEEDFORWARD;
  this->legR->Fset       = FFEEDFORWARD;
  // ÐéÄâÁ¦
  float lfCompensate     = this->legL->L0PID.compute(this->legL->L0.now);
  float rfCompensate     = this->legR->L0PID.compute(this->legR->L0.now);
  this->legL->Fset      += lfCompensate;
  this->legR->Fset      += rfCompensate;
  // ÅüÍÈ²¹³¥
  float splitCompensate  = this->splitPID.compute(this->legL->theta.now - this->legR->theta.now);
  this->legL->Tbset     += splitCompensate;
  this->legR->Tbset     -= splitCompensate;
  // ·­¹ö½Ç²¹³¥
  float rollCompensate   = this->rollPID.compute(this->sensor->roll.now);
  this->legL->Fset      += rollCompensate;
  this->legR->Fset      -= rollCompensate;

  legL->VMC();
  legR->VMC();

  legL->run();
  legR->run();
}

void Car::SplitMPCControl() {
}
