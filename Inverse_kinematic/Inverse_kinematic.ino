
// --------------------Robot configuration----------------------
// Leg configuration
float LENGTH_1 = 75; // mm
float LENGTH_2 = 70; // mm
float LENGTH_3 = 90; // mm

int n = -1;
float default_pos_r[3] = {0, -90, -120};
float default_pos_l[3] = {0, 90, -120};

struct leg {
  float pos[3];
  int jointAngles[3];
  bool legState = 0; // 0 for contact 1 for swing.
  bool isLeftLeg = 0; // 0 for no 1 for yes
  float currentPhase = 0.0;
  float phaseOffset = 0.0;
  float switchingPhase = 0.6;
  float periodTimeNominal = 0.6; // seconds
  float xRange[2];
  float yRange[2];
};

int mode = 0;

void calculate_ik_l(float *coordinate, int *jointAngle) {
  float x = coordinate[0];
  float y = coordinate[1];
  float z = coordinate[2];

  float theta1 = 0;
  if (y >= 0) {
    theta1 = acos(LENGTH_1 / (sqrt(sq(y) + sq(z)))) - acos(abs(y) / (sqrt(sq(y) + sq(z))));
  }
  else {
    theta1 = acos(LENGTH_1 / (sqrt(sq(y) + sq(z)))) - acos(abs(z) / (sqrt(sq(y) + sq(z)))) - 1.5708;
  }

  float theta2 = 0;
  float theta3 = 0;
  //  Serial.println("Original Z: " + String(z) + " and theta_1: " + String(theta1));
  z = z - LENGTH_1 * sin(theta1);
  float length_ = sqrt(sq(x) + sq(z));
  //  Serial.println("calculate new Z: " + String(z) + " and length " + String(length_));
  if (x >= 0) {
    theta2 = -( (acos(abs(z) / length_) - acos(-(sq(LENGTH_3) - sq(length_) - sq(LENGTH_2)) / (2 * length_ * LENGTH_2))));
    theta3 = acos(-(sq(length_) - sq(LENGTH_3) - sq(LENGTH_2)) / (2 * LENGTH_3 * LENGTH_2)) - 1.5708;
  }
  else {
    theta2 = acos(abs(z) / (sqrt(sq(x) + sq(z)))) + acos(-(sq(LENGTH_3) - sq(LENGTH_2) - sq(length_)) / (2 * LENGTH_2 * length_));
    theta3 = acos(-(sq(length_) - sq(LENGTH_2) - sq(LENGTH_3)) / (2 * LENGTH_2 * LENGTH_3)) - 1.5708;
  }
  //  Serial.println("calculate_ik_l - 1: " + String(theta1) + "--- 2: " + String(theta2) + "--- 3: " + String(theta3));

  jointAngle[0] = int(theta1 * (180 / 3.14));
  jointAngle[1] = int(theta2 * (180 / 3.14));
  jointAngle[2] = int(theta3 * (180 / 3.14));
}

void calculate_ik_r(float *coordinate, int *jointAngle) {
  float x = coordinate[0];
  float y = coordinate[1];
  float z = coordinate[2];

  float theta1 = 0;
  if (y >= 0) {
    theta1 = 1.5708 - acos(LENGTH_1 / (sqrt(sq(y) + sq(z)))) - acos(abs(z) / (sqrt(sq(y) + sq(z))));
  }
  else {
    theta1 = -( acos(LENGTH_1 / (sqrt(sq(y) + sq(z)))) - acos(abs(y) / (sqrt(sq(y) + sq(z)))) );
  }

  float theta2 = 0;
  float theta3 = 0;
  //  Serial.println("calculate_ik_r Original Z: " + String(z) + " and theta_1: " + String(theta1));
  z = z + LENGTH_1 * sin(theta1);
  float length_ = sqrt(sq(x) + sq(z));
  //  Serial.println("calculate_ik_r calculate new Z: " + String(z) + " and length " + String(length_));

  if (x >= 0) {
    theta2 = -( (acos(abs(z) / length_) - acos(-(sq(LENGTH_3) - sq(length_) - sq(LENGTH_2)) / (2 * length_ * LENGTH_2))));
    theta3 = acos(-(sq(length_) - sq(LENGTH_3) - sq(LENGTH_2)) / (2 * LENGTH_3 * LENGTH_2)) - 1.5708;
  }
  else {
    theta2 = acos(abs(z) / (sqrt(sq(x) + sq(z)))) + acos(-(sq(LENGTH_3) - sq(LENGTH_2) - sq(length_)) / (2 * LENGTH_2 * length_));
    theta3 = acos(-(sq(length_) - sq(LENGTH_2) - sq(LENGTH_3)) / (2 * LENGTH_2 * LENGTH_3)) - 1.5708;
  }
  //  Serial.println("calculate_ik_r - 1: " + String(theta1) + " --- 2: " + String(theta2) + " --- 3: " + String(theta3));
  jointAngle[0] = int(theta1 * (180 / 3.14));
  jointAngle[1] = int(theta2 * (180 / 3.14));
  jointAngle[2] = int(theta3 * (180 / 3.14));
}

float setZHeight(float currX, float strideLength, float walkHeight) {
  float actualZ = -(4 * walkHeight) / (sq(abs(strideLength))) * sq(currX) + walkHeight;
  return actualZ;
}

int calculate_servo_delay(const int* theta_fr, const int* theta_fr_before) {
  int delay_ms = 0;
  float rot_ve = 300;
  for (int i = 0; i < 2; i++) {
    int delta_theta = abs(theta_fr[i] - theta_fr_before[i]);
    float delta_curr_ms = (float(delta_theta) / rot_ve) * 1000;
    if (delta_curr_ms > delay_ms)
      delay_ms = delta_curr_ms;
    Serial.println("delta_theta: " + String(delta_theta) + " delta_curr_ms: " + String(delta_curr_ms) + " delay_ms: " + String(delay_ms));
  }
  return delay_ms;
}

float calculate_new_x(const float x, float velocity, const float delta_t) {
  float newX = x + velocity * (delta_t / 1000);
  return newX;
}

float calculateNewY(const float y, float velocity, const float delta_t) {
  float newY = y + velocity * (delta_t / 1000);
  return newY;
}

void initLeg(leg* currLeg, bool is_left, bool contactState) {
  if (is_left) {
    memcpy(currLeg->pos, default_pos_l, sizeof(currLeg->pos));
    calculate_ik_l(currLeg->pos, currLeg->jointAngles);
  } else {
    memcpy(currLeg->pos, default_pos_r, sizeof(currLeg->pos));
    calculate_ik_r(currLeg->pos, currLeg->jointAngles);
  }
  currLeg->isLeftLeg = is_left;
  currLeg->legState = contactState;
}

float calculateNewPos(float xMin, float xMax, float currPhasePercent){
  float x = xMin + (xMax - xMin) * currPhasePercent;
  return x;
}

void contactLegXYZupdate(leg* currLeg, float z_contact_offset, float z_walk_height, bool is_left) {
  currLeg->pos[0] = calculateNewPos(currLeg->xRange[1], currLeg->xRange[0], currLeg->currentPhase / currLeg->switchingPhase);
  currLeg->pos[1] = calculateNewPos(currLeg->yRange[1], currLeg->yRange[0], currLeg->currentPhase / currLeg->switchingPhase);
  currLeg->pos[2] = z_contact_offset;
  if (is_left)
    calculate_ik_l(currLeg->pos, currLeg->jointAngles);
  else
    calculate_ik_r(currLeg->pos, currLeg->jointAngles);
}

void swingLegXYZupdate(leg* currLeg, float walkStride, float z_contact_offset, float z_walk_height, bool is_left) {
  currLeg->pos[0] = calculateNewPos(currLeg->xRange[0], currLeg->xRange[1], (currLeg->currentPhase-currLeg->switchingPhase) / (1-currLeg->switchingPhase));
  currLeg->pos[1] = calculateNewPos(currLeg->yRange[0], currLeg->yRange[1], (currLeg->currentPhase-currLeg->switchingPhase) / (1-currLeg->switchingPhase));
  currLeg->pos[2] = z_contact_offset + setZHeight(currLeg->pos[0], walkStride, z_walk_height);
  if (is_left)
    calculate_ik_l(currLeg->pos, currLeg->jointAngles);
  else
    calculate_ik_r(currLeg->pos, currLeg->jointAngles);
}

void headingControl(leg *flLeg, leg *frLeg, leg *blLeg, leg *br) {
  return 0;
}

void gaitScheduler(leg* currLeg, float dt) {
  // contact state
  float dtPhase = dt / 1000 / currLeg->periodTimeNominal;
  currLeg->currentPhase = currLeg->currentPhase + dtPhase;
  Serial.println("currLeg currentPhase: " + String(currLeg->currentPhase, 4) + " dtPhase: " + String(dtPhase, 4) + " dt: " + String(dt, 4));
  if (currLeg->currentPhase >= currLeg->switchingPhase && currLeg->currentPhase < 1) {
    currLeg->legState = 1;
  } else if (currLeg->currentPhase >= 1) {
    currLeg->legState = 0;
    currLeg->currentPhase = 0;
  } else {
    currLeg->legState = 0;
  }
}

void updateLegXRange(leg* legs, float rangeMin, float rangeMax){
  legs[0].xRange[0] = rangeMin, legs[0].xRange[1] = rangeMax;
  legs[1].xRange[0] = rangeMin, legs[1].xRange[1] = rangeMax;
  legs[2].xRange[0] = rangeMin, legs[2].xRange[1] = rangeMax;
  legs[3].xRange[0] = rangeMin, legs[3].xRange[1] = rangeMax;
}

void updateLegYRange(leg* legs, float rangeMin, float rangeMax){
  legs[0].yRange[0] = default_pos_l[1] + rangeMin, legs[0].yRange[1] = default_pos_l[1] + rangeMax;
  legs[1].yRange[0] = default_pos_r[1] + rangeMin, legs[1].yRange[1] = default_pos_r[1] + rangeMax;
  legs[2].yRange[0] = default_pos_l[1] + rangeMin, legs[2].yRange[1] = default_pos_l[1] + rangeMax;
  legs[3].yRange[0] = default_pos_r[1] + rangeMin, legs[3].yRange[1] = default_pos_r[1] + rangeMax;
}

void velocityControl() {
  leg flLeg, frLeg, blLeg, brLeg;
  initLeg(&flLeg, true, 0);
  initLeg(&frLeg, false, 1);
  initLeg(&blLeg, true, 1);
  initLeg(&brLeg, false, 0);

  flLeg.currentPhase = 0.5;
  frLeg.currentPhase = 0.0;
  blLeg.currentPhase = 0.0;
  brLeg.currentPhase = 0.5;
  
  leg legs[4] = {flLeg, frLeg, blLeg, brLeg};

  float z_contact_offset = -120;
  float z_walk_height = 40;
  float walkStride = 80;

  // Contact phase
  int serialIntOutput = 0;
  float headingRad = (float(serialIntOutput) / 180 * 3.14);
  
  float xStride = walkStride * cos(headingRad);
  float xmin = -xStride / 2, xmax = xStride / 2;
  updateLegXRange(legs, xmin, xmax);

  float yStride = walkStride * sin(headingRad);
  float ymin = -yStride / 2, ymax = yStride / 2;
  updateLegYRange(legs, ymin, ymax);
  
  float oldTime = millis(), currentTime = millis(), deltaTime = 0;
  while (true) {
    if (Serial.available()) {
      Serial.println("Pasrsing output");
      serialIntOutput = Serial.parseInt();
      Serial.println("Pasrsing output: " + String(serialIntOutput));
    }
    if (int(serialIntOutput) == -1) {
      return 0;
    }
    if (serialIntOutput != 0) {
      headingRad = (float(serialIntOutput) / 180 * 3.14);
      xStride = walkStride * cos(headingRad);
      xmin = -xStride / 2, xmax = xStride / 2;
      updateLegXRange(legs, xmin, xmax);

      yStride = walkStride * sin(headingRad);
      ymin = -yStride / 2, ymax = yStride / 2;
      updateLegYRange(legs, ymin, ymax);
    }

    deltaTime = currentTime - oldTime; // deltaTime is in milliseconds
    oldTime = currentTime;
    for (int i = 0; i < 4; i++) {
      gaitScheduler(&legs[i], deltaTime);
      if (legs[i].legState == 0) {
        contactLegXYZupdate(&legs[i], z_contact_offset, z_walk_height, legs[i].isLeftLeg);
        Serial.println("contact leg " + String(i) + " x: " + String(legs[i].pos[0]) + " y: " + String(legs[i].pos[1]) + " z: " + String(legs[i].pos[2]) + " currentPhase: " + String(legs[i].currentPhase));
      } else {
        swingLegXYZupdate(&legs[i], walkStride, z_contact_offset, z_walk_height, legs[i].isLeftLeg);
        Serial.println("swing leg " + String(i) + " x: " + String(legs[i].pos[0]) + " y: " + String(legs[i].pos[1]) + " z: " + String(legs[i].pos[2]) + " currentPhase: " + String(legs[i].currentPhase));
      }
    }
    set_join_array_leg(legs[0].jointAngles, legs[1].jointAngles, legs[2].jointAngles, legs[3].jointAngles);
    currentTime = millis();
  }
}

void setup() {
  Serial.begin(115200);
  Serial.setTimeout(10);
  initServos();
  yield();

  int theta_l[3] = {0, 0, 0};
  int theta_r[3] = {0, 0, 0};
  calculate_ik_l(default_pos_l, theta_l);
  calculate_ik_r(default_pos_r, theta_r);
  float *pos_l = default_pos_l;
  float *pos_r = default_pos_r;
  set_join_array_leg(theta_l, theta_r, theta_l, theta_r);
  delay(100);
}

void loop() {
  int theta_r[3] = {0, 0, 0};
  int theta_l[3] = {0, 0, 0};
  int serialIntOutput = 0;

  if (Serial.available()) {
    Serial.println("Pasrsing output");
    serialIntOutput = Serial.parseInt();
    Serial.println("Pasrsing output" + String(serialIntOutput));
  }
  //  Serial.println("After Serial.available");
  if (serialIntOutput != 0) {
    mode = serialIntOutput;
  }
  switch (mode) {
    case -1:
      float pos_r[3];
      float pos_l[3];
      memcpy(pos_r, default_pos_r, sizeof(pos_r));
      memcpy(pos_l, default_pos_l, sizeof(pos_l));
      calculate_ik_l(pos_l, theta_l);
      calculate_ik_r(pos_r, theta_r);
      set_join_array_leg(theta_l, theta_r, theta_l, theta_r);
      delay(100);
      break;
    case 3:
      serialSetZ(theta_l, theta_r);
      mode = -1;
      break;
    case 4:
      serialSetBodyRotX(theta_l, theta_r);
      mode = -1;
      break;
    case 5:
      serialSetBodyRotY(theta_l, theta_r);
      mode = -1;
      break;
    case 6:
      serialSetBodyRotZ(theta_l, theta_r);
      mode = -1;
      break;
    case 7:
      serialSetY(theta_l, theta_r);
      mode = -1;
      break;
    case 8:
      serialSetX(theta_l, theta_r);
      mode = -1;
      break;
    case 11:
      velocityControl();
      mode = -1;
      break;
  }
}
