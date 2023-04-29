void stand_cycle(int *theta_l, int *theta_r) {
  float pos_r[3], pos_l[3];
  memcpy(pos_r, default_pos_r, sizeof(pos_r));
  memcpy(pos_l, default_pos_l, sizeof(pos_l));
  float standOffSet = 40;
  float standInterval = 5;
  float z_offset = -90;
  int delay_ms = 50;
  for (float z = standOffSet; z > -standOffSet; z -= standInterval) {
    float actual_z = z + z_offset;
    pos_r[2] = actual_z;
    pos_l[2] = actual_z;
    //    Serial.println("Position_r - x " + String(pos_r[0]) + "--- y " + String(pos_r[1]) + "--- z " + String(pos_r[2]));
    //    Serial.println("Position_l - x " + String(pos_l[0]) + "--- y " + String(pos_l[1]) + "--- z " + String(pos_l[2]));
    calculate_ik_r(pos_r, theta_r);
    calculate_ik_l(pos_l, theta_l);
    set_join_array_leg(theta_l, theta_r, theta_l, theta_r);
    //    Serial.println("joint_angle_r - 1 " + String(theta_r[0]) + "--- 2 " + String(theta_r[1]) + "--- 3 " + String(theta_r[2]));
    //    Serial.println("joint_angle_l - 1 " + String(theta_l[0]) + "--- 2 " + String(theta_l[1]) + "--- 3 " + String(theta_l[2]));
    delay(delay_ms);
  }
  for (float z = -standOffSet; z < standOffSet; z += standInterval) {
    float actual_z = z + z_offset;
    pos_r[2] = actual_z;
    pos_l[2] = actual_z;
    //    Serial.println("Position_r - x " + String(pos_r[0]) + "--- y " + String(pos_r[1]) + "--- z " + String(pos_r[2]));
    //    Serial.println("Position_l - x " + String(pos_l[0]) + "--- y " + String(pos_l[1]) + "--- z " + String(pos_l[2]));
    calculate_ik_r(pos_r, theta_r);
    calculate_ik_l(pos_l, theta_l);
    set_join_array_leg(theta_l, theta_r, theta_l, theta_r);
    //    Serial.println("joint_angle_r - 1 " + String(theta_r[0]) + "--- 2 " + String(theta_r[1]) + "--- 3 " + String(theta_r[2]));
    //    Serial.println("joint_angle_l - 1 " + String(theta_l[0]) + "--- 2 " + String(theta_l[1]) + "--- 3 " + String(theta_l[2]));
    delay(delay_ms);
  }
}

int serialSetX(int *theta_l, int *theta_r) {
  float pos_r[3], pos_l[3];
  memcpy(pos_r, default_pos_r, sizeof(pos_r));
  memcpy(pos_l, default_pos_l, sizeof(pos_l));
  int delay_ms = 50;
  int serialIntOutput = 0;
  while (true) {
    if (Serial.available()) {
      Serial.println("Pasrsing output");
      serialIntOutput = Serial.parseInt();
      Serial.println("Pasrsing output" + String(serialIntOutput));
    }
    if (serialIntOutput != 0) {
      pos_r[0] = serialIntOutput;
      pos_l[0] = serialIntOutput;
      calculate_ik_r(pos_r, theta_r);
      calculate_ik_l(pos_l, theta_l);
      set_join_array_leg(theta_l, theta_r, theta_l, theta_r);
    }
    if (serialIntOutput == -1) {
      return 0;
    }
  }
}

int serialSetY(int *theta_l, int *theta_r) {
  float pos_bl[3];
  float pos_br[3];
  float pos_fl[3];
  float pos_fr[3];
  int theta_bl[3];
  int theta_br[3];
  int theta_fl[3];
  int theta_fr[3];
  int delay_ms = 50;
  int serialIntOutput = 0;
  while (true) {
    if (Serial.available()) {
      Serial.println("Pasrsing output");
      serialIntOutput = Serial.parseInt();
      Serial.println("Pasrsing output" + String(serialIntOutput));
    }
    if (serialIntOutput != 0) {
      memcpy(pos_bl, default_pos_l, sizeof(pos_bl));
      memcpy(pos_br, default_pos_r, sizeof(pos_br));
      memcpy(pos_fl, default_pos_l, sizeof(pos_fl));
      memcpy(pos_fr, default_pos_r, sizeof(pos_fr));
      pos_bl[1] = pos_bl[1] + serialIntOutput;
      pos_br[1] = pos_br[1] + serialIntOutput;
      pos_fl[1] = pos_fl[1] + serialIntOutput;
      pos_fr[1] = pos_fr[1] + serialIntOutput;
      calculate_ik_l(pos_bl, theta_bl);
      calculate_ik_r(pos_br, theta_br);
      calculate_ik_l(pos_fl, theta_fl);
      calculate_ik_r(pos_fr, theta_fr);
      Serial.println("pos_fl - 1 " + String(pos_fl[0]) + "--- 2 " + String(pos_fl[1]) + "--- 3 " + String(pos_fl[2]));
      Serial.println("pos_fr - 1 " + String(pos_fr[0]) + "--- 2 " + String(pos_fr[1]) + "--- 3 " + String(pos_fr[2]));
      Serial.println("pos_bl - 1 " + String(pos_bl[0]) + "--- 2 " + String(pos_bl[1]) + "--- 3 " + String(pos_bl[2]));
      Serial.println("pos_br - 1 " + String(pos_br[0]) + "--- 2 " + String(pos_br[1]) + "--- 3 " + String(pos_br[2]));
      Serial.println("theta_fl - 1 " + String(theta_fl[0]) + "--- 2 " + String(theta_fl[1]) + "--- 3 " + String(theta_fl[2]));
      Serial.println("theta_fr - 1 " + String(theta_fr[0]) + "--- 2 " + String(theta_fr[1]) + "--- 3 " + String(theta_fr[2]));
      Serial.println("theta_bl - 1 " + String(theta_bl[0]) + "--- 2 " + String(theta_bl[1]) + "--- 3 " + String(theta_bl[2]));
      Serial.println("theta_br - 1 " + String(theta_br[0]) + "--- 2 " + String(theta_br[1]) + "--- 3 " + String(theta_br[2]));
      set_join_array_leg(theta_fl, theta_fr, theta_bl, theta_br);
    }
    if (serialIntOutput == -1) {
      return 0;
    }
  }
}

int serialSetZ(int *theta_l, int *theta_r) {
  float pos_r[3], pos_l[3];
  memcpy(pos_r, default_pos_r, sizeof(pos_r));
  memcpy(pos_l, default_pos_l, sizeof(pos_l));
  int delay_ms = 50;
  int serialIntOutput = 0;
  while (true) {
    if (Serial.available()) {
      Serial.println("Pasrsing output");
      serialIntOutput = Serial.parseInt();
      Serial.println("Pasrsing output" + String(serialIntOutput));
    }
    if (serialIntOutput < 0) {
      pos_r[2] = serialIntOutput;
      pos_l[2] = serialIntOutput;
      calculate_ik_r(pos_r, theta_r);
      calculate_ik_l(pos_l, theta_l);
      set_join_array_leg(theta_l, theta_r, theta_l, theta_r);
    }
    if (serialIntOutput == -1) {
      return 0;
    }
  }
}

void serialSetBodyRotX(int *theta_l, int *theta_r) {
  float pos_r[3], pos_l[3];
  int delay_ms = 50;
  int serialIntOutput = 0;
  while (true) {
    if (Serial.available()) {
      Serial.println("Pasrsing output");
      serialIntOutput = Serial.parseInt();
      Serial.println("Pasrsing output" + String(serialIntOutput));
    }
    if (serialIntOutput != 0) {
      memcpy(pos_r, default_pos_r, sizeof(pos_r));
      memcpy(pos_l, default_pos_l, sizeof(pos_l));
      float rad = float(serialIntOutput) * (3.14 / 180);
      float dz = 232 / 2 * float(sin(rad)); // 232 is the body length in y direction
      pos_r[2] = pos_r[2] + dz;
      pos_l[2] = pos_l[2] - dz;
      calculate_ik_r(pos_r, theta_r);
      calculate_ik_l(pos_l, theta_l);
      set_join_array_leg(theta_l, theta_r, theta_l, theta_r);
    }
    if (serialIntOutput == -1) {
      return 0;
    }
  }
}

void serialSetBodyRotY(int *theta_l, int *theta_r) {
  float pos_bl[3];
  float pos_br[3];
  float pos_fl[3];
  float pos_fr[3];
  int theta_bl[3];
  int theta_br[3];
  int theta_fl[3];
  int theta_fr[3];
  int delay_ms = 50;
  int serialIntOutput = 0;
  while (true) {
    if (Serial.available()) {
      Serial.println("Pasrsing output");
      serialIntOutput = Serial.parseInt();
      Serial.println("Pasrsing output" + String(serialIntOutput));
    }
    if (serialIntOutput != 0) {
      memcpy(pos_bl, default_pos_l, sizeof(pos_bl));
      memcpy(pos_br, default_pos_r, sizeof(pos_br));
      memcpy(pos_fl, default_pos_l, sizeof(pos_fl));
      memcpy(pos_fr, default_pos_r, sizeof(pos_fr));
      float rad = float(serialIntOutput) * (3.14 / 180);
      float dz = 256 / 2 * float(sin(rad)); // 232 is the body length in x direction
      pos_bl[2] = pos_bl[2] + dz;
      pos_br[2] = pos_br[2] + dz;
      pos_fl[2] = pos_fl[2] - dz;
      pos_fr[2] = pos_fr[2] - dz;
      calculate_ik_l(pos_bl, theta_bl);
      calculate_ik_r(pos_br, theta_br);
      calculate_ik_l(pos_fl, theta_fl);
      calculate_ik_r(pos_fr, theta_fr);
      set_join_array_leg(theta_fl, theta_fr, theta_bl, theta_br);
    }
    if (serialIntOutput == -1) {
      return 0;
    }
  }
}

void serialSetBodyRotZ(int *theta_l, int *theta_r) {
  float pos_bl[3];
  float pos_br[3];
  float pos_fl[3];
  float pos_fr[3];
  int theta_bl[3];
  int theta_br[3];
  int theta_fl[3];
  int theta_fr[3];
  int delay_ms = 50;
  int serialIntOutput = 0;
  while (true) {
    if (Serial.available()) {
      Serial.println("Pasrsing output");
      serialIntOutput = Serial.parseInt();
      Serial.println("Pasrsing output" + String(serialIntOutput));
    }
    if (serialIntOutput != 0) {
      memcpy(pos_bl, default_pos_l, sizeof(pos_bl));
      memcpy(pos_br, default_pos_r, sizeof(pos_br));
      memcpy(pos_fl, default_pos_l, sizeof(pos_fl));
      memcpy(pos_fr, default_pos_r, sizeof(pos_fr));
      float rad = float(serialIntOutput) * (3.14 / 180);
      float dy = 256 / 2 * float(sin(rad)); // 256 is the body length in x direction
      pos_fl[1] = pos_fl[1] + dy;
      pos_fr[1] = pos_fr[1] - dy;
      pos_bl[1] = pos_bl[1] + dy;
      pos_br[1] = pos_br[1] - dy;
      calculate_ik_l(pos_fl, theta_fl);
      calculate_ik_r(pos_fr, theta_fr);
      calculate_ik_l(pos_bl, theta_bl);
      calculate_ik_r(pos_br, theta_br);
      Serial.println("pos_fl - 1 " + String(pos_fl[0]) + "--- 2 " + String(pos_fl[1]) + "--- 3 " + String(pos_fl[2]));
      Serial.println("pos_fr - 1 " + String(pos_fr[0]) + "--- 2 " + String(pos_fr[1]) + "--- 3 " + String(pos_fr[2]));
      Serial.println("pos_bl - 1 " + String(pos_bl[0]) + "--- 2 " + String(pos_bl[1]) + "--- 3 " + String(pos_bl[2]));
      Serial.println("pos_br - 1 " + String(pos_br[0]) + "--- 2 " + String(pos_br[1]) + "--- 3 " + String(pos_br[2]));
      Serial.println("theta_fl - 1 " + String(theta_fl[0]) + "--- 2 " + String(theta_fl[1]) + "--- 3 " + String(theta_fl[2]));
      Serial.println("theta_fr - 1 " + String(theta_fr[0]) + "--- 2 " + String(theta_fr[1]) + "--- 3 " + String(theta_fr[2]));
      Serial.println("theta_bl - 1 " + String(theta_bl[0]) + "--- 2 " + String(theta_bl[1]) + "--- 3 " + String(theta_bl[2]));
      Serial.println("theta_br - 1 " + String(theta_br[0]) + "--- 2 " + String(theta_br[1]) + "--- 3 " + String(theta_br[2]));
      set_join_array_leg(theta_fl, theta_fr, theta_bl, theta_br);
    }
    if (serialIntOutput == -1) {
      return 0;
    }
  }
}
