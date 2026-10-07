// #include <cstdio>
// #include <cstdlib>
// #include <cstring>
// #include <cmath>
// #include <algorithm>
// #include <fstream>

// #include <GLFW/glfw3.h>
// #include <mujoco/mujoco.h>
// #include "imgui.h"
// #include "imgui_impl_glfw.h"
// #include "imgui_impl_opengl3.h"

// // ----- Bien toan cuc MuJoCo -----
// mjModel* m = NULL;
// mjData*  d = NULL;
// mjvCamera cam;
// mjvOption opt;
// mjvScene  scn;
// mjrContext con;

// // ----- Tuong tac chuot de xoay camera (giu nguyen tu sample goc) -----
// bool button_left = false, button_middle = false, button_right = false;
// double lastx = 0, lasty = 0;

// void mouse_button(GLFWwindow* window, int button, int act, int mods) {
//     ImGui_ImplGlfw_MouseButtonCallback(window, button, act, mods);
//     if (ImGui::GetIO().WantCaptureMouse) return;
//     button_left   = glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT)  == GLFW_PRESS;
//     button_middle = glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE)== GLFW_PRESS;
//     button_right  = glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS;
//     glfwGetCursorPos(window, &lastx, &lasty);
// }

// void mouse_move(GLFWwindow* window, double xpos, double ypos) {
//     ImGui_ImplGlfw_CursorPosCallback(window, xpos, ypos);
//     if (ImGui::GetIO().WantCaptureMouse) return;
//     if (!button_left && !button_middle && !button_right) return;
//     double dx = xpos - lastx, dy = ypos - lasty;
//     lastx = xpos; lasty = ypos;
//     int width, height;
//     glfwGetWindowSize(window, &width, &height);
//     bool mod_shift = glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS;
//     mjtMouse action;
//     if (button_right)       action = mod_shift ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V;
//     else if (button_left)   action = mod_shift ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V;
//     else                    action = mjMOUSE_ZOOM;
//     mjv_moveCamera(m, action, dx / height, dy / height, &cam);
// }

// void scroll(GLFWwindow* window, double xoffset, double yoffset) {
//     ImGui_ImplGlfw_ScrollCallback(window, xoffset, yoffset);
//     if (ImGui::GetIO().WantCaptureMouse) return;
//     mjv_moveCamera(m, mjMOUSE_ZOOM, 0, -0.05 * yoffset, &cam);
// }

// void key_cb(GLFWwindow* window, int key, int scancode, int action, int mods) {
//     ImGui_ImplGlfw_KeyCallback(window, key, scancode, action, mods);
// }

// void char_cb(GLFWwindow* window, unsigned int c) {
//     ImGui_ImplGlfw_CharCallback(window, c);
// }

// // ----- Chuyen quaternion [w,x,y,z] sang roll/pitch/yaw (radian) -----
// void quat_to_euler(const mjtNum q[4], double& roll, double& pitch, double& yaw) {
//     double w = q[0], x = q[1], y = q[2], z = q[3];
//     double sinr = 2 * (w * x + y * z);
//     double cosr = 1 - 2 * (x * x + y * y);
//     roll = std::atan2(sinr, cosr);

//     double sinp = std::clamp(2 * (w * y - z * x), -1.0, 1.0);
//     pitch = std::asin(sinp);

//     double siny = 2 * (w * z + x * y);
//     double cosy = 1 - 2 * (y * y + z * z);
//     yaw = std::atan2(siny, cosy);
// }

// // ----- Goc lech giua truc than (Z) va phuong thang dung, dung o moi muc do nghieng -----
// double tilt_from_vertical(const mjtNum q[4]) {
//     double x = q[1], y = q[2];
//     double R22 = std::clamp(1.0 - 2.0 * (x * x + y * y), -1.0, 1.0);
//     return std::acos(R22);
// }

// // ----- Vector huong truc Z cua than, dung de bu luc day khi nghieng -----
// void body_z_axis(const mjtNum q[4], double bz[3]) {
//     double w = q[0], x = q[1], y = q[2], z = q[3];
//     bz[0] = 2 * (x * z + w * y);
//     bz[1] = 2 * (y * z - w * x);
//     bz[2] = 1 - 2 * (x * x + y * y);
// }

// // ----- Class PID don gian, co anti-windup -----
// struct PID {
//     double kp, ki, kd;
//     double integral = 0.0;
//     double integral_limit;

//     PID(double kp_, double ki_, double kd_, double limit = 2.0)
//         : kp(kp_), ki(ki_), kd(kd_), integral_limit(limit) {}

//     double update(double error, double rate_measured, double dt) {
//         integral = std::clamp(integral + error * dt, -integral_limit, integral_limit);
//         return kp * error + ki * integral - kd * rate_measured;
//     }

//     void reset() { integral = 0.0; }
// };

// int main(int argc, char** argv) {
//     if (argc < 2) {
//         std::printf("Cach dung: %s duong_dan_toi_quad.xml\n", argv[0]);
//         return 1;
//     }

//     char error[1000] = "";
//     m = mj_loadXML(argv[1], nullptr, error, sizeof(error));
//     if (!m) {
//         std::printf("Loi nap model: %s\n", error);
//         return 1;
//     }
//     d = mj_makeData(m);

//     double mass = 0;
//     for (int i = 0; i < m->nbody; i++) mass += m->body_mass[i];
//     double g = 9.81;
//     int id_motor1 = mj_name2id(m, mjOBJ_SITE, "motor1");
//     double a = m->site_pos[3 * id_motor1 + 1];
//     double b = m->site_pos[3 * id_motor1 + 0];
//     int id_thrust1 = mj_name2id(m, mjOBJ_ACTUATOR, "thrust1");
//     double k = m->actuator_gear[6 * id_thrust1 + 5];
//     double ctrl_max = m->actuator_ctrlrange[2 * id_thrust1 + 1];
//     double mass_check = 0; for (int i = 0; i < m->nbody; i++) mass_check += m->body_mass[i];
//     double hover_per_motor = mass_check * g / 4.0;
//     if (ctrl_max < hover_per_motor * 2.0) {
//         std::printf("CANH BAO: ctrlrange toi da (%.3f N) chua gap doi luc day hover (%.3f N).\n", ctrl_max, hover_per_motor);
//         std::printf("Co the khong du luc de phuc hoi tu goc lat lon. Nen dat ctrlrange >= %.3f N.\n\n", hover_per_motor * 2.0);
//     }

//     double Marr[4][4] = {
//         { 1,  1,  1,  1},
//         { a, -a,  a, -a},
//         {-b,  b,  b, -b},
//         { k,  k, -k, -k},
//     };
//     double Minv[4][4];
//     {
//         double A[4][8];
//         for (int i = 0; i < 4; i++) {
//             for (int j = 0; j < 4; j++) A[i][j] = Marr[i][j];
//             for (int j = 0; j < 4; j++) A[i][j + 4] = (i == j) ? 1.0 : 0.0;
//         }
//         for (int col = 0; col < 4; col++) {
//             int piv = col;
//             for (int r = col + 1; r < 4; r++)
//                 if (std::fabs(A[r][col]) > std::fabs(A[piv][col])) piv = r;
//             std::swap(A[col], A[piv]);
//             double pivval = A[col][col];
//             for (int j = 0; j < 8; j++) A[col][j] /= pivval;
//             for (int r = 0; r < 4; r++) {
//                 if (r == col) continue;
//                 double factor = A[r][col];
//                 for (int j = 0; j < 8; j++) A[r][j] -= factor * A[col][j];
//             }
//         }
//         for (int i = 0; i < 4; i++)
//             for (int j = 0; j < 4; j++)
//                 Minv[i][j] = A[i][j + 4];
//     }

//     // ----- Tham so PID (da kiem chung: hoi phuc <2s tu goc lat 85-89 do) -----
//     // ----- Tang Position (ngoai cung): sai so vi tri -> goc nghieng mong muon -----
//     PID pid_z(1.5, 1.0, 0.6, 0.4);
//     PID pid_x(4.0, 0.0, 3.3, 0.5);
//     PID pid_y(4.0, 0.0, 3.3, 0.5);

//     // ----- Tang Angle (giua): sai so goc -> TOC DO GOC mong muon (rate setpoint) -----
//     // Day la tang bi "gop tat" trong ban truoc; gio tach rieng that su, dung
//     // sai so goc tinh qua quaternion (on dinh moi goc) thay vi Euler cho roll/pitch
//     PID pid_angle_roll(8.0, 0.0, 0.0, 100.0);
//     PID pid_angle_pitch(8.0, 0.0, 0.0, 100.0);
//     PID pid_angle_yaw(6.0, 0.0, 0.0, 10.0);
//     const double RATE_MAX = 400.0 * 3.14159265358979323846 / 180.0;   // gioi han toc do quay dat (rad/s)

//     // ----- Tang Rate (trong cung, nhanh nhat): sai so toc do goc (tu gyro) -> mo-men -----
//     // them kd (so hang D) de giam dao dong/vot lo khi phuc hoi tu goc rat lon
//     PID pid_rate_roll(0.0010, 0.0, 0.0003, 1.0);
//     PID pid_rate_pitch(0.0010, 0.0, 0.0003, 1.0);
//     PID pid_rate_yaw(0.0010, 0.0, 0.0, 1.0);

//     const double max_tilt = 45.0 * 3.14159265358979323846 / 180.0;

//     float equilibrium[3] = {0.0f, 0.0f, 1.5f};
//     bool recovery_active = false;
//     bool recovery_ever_started = false;
//     double recovery_start_time = 0.0;
//     int log_counter = 0;
//     std::ofstream recovery_log;

//     auto start_recovery_test = [&]() {
//         mj_resetData(m, d);
//         d->qpos[0] = equilibrium[0];
//         d->qpos[1] = equilibrium[1];
//         d->qpos[2] = equilibrium[2];
//         // Lat dong thoi: roll = 85 do VA pitch = 85 do (quaternion ZYX, yaw=0)
//         // Tong goc lech thuc te so voi phuong thang dung ~89.6 do (gan nhu lat up hoan toan)
//         double cr = std::cos(85.0 * 3.14159265358979323846 / 360.0);
//         double sr = std::sin(85.0 * 3.14159265358979323846 / 360.0);
//         d->qpos[3] = cr * cr;  d->qpos[4] = sr * cr;
//         d->qpos[5] = cr * sr;  d->qpos[6] = -sr * sr;
//         mj_forward(m, d);

//         pid_z.reset(); pid_x.reset(); pid_y.reset();
//         pid_angle_roll.reset(); pid_angle_pitch.reset(); pid_angle_yaw.reset();
//         pid_rate_roll.reset(); pid_rate_pitch.reset(); pid_rate_yaw.reset();

//         recovery_start_time = d->time;
//         recovery_active = true;
//         recovery_ever_started = true;

//         recovery_log.close();
//         recovery_log.open("recovery_log.csv");
//         recovery_log << "time,x_error_m,y_error_m,z_error_m,tilt_error_deg,motor1_N,motor2_N,motor3_N,motor4_N\n";
//     };

//     if (!glfwInit()) { std::printf("Khong khoi tao duoc GLFW\n"); return 1; }
//     GLFWwindow* window = glfwCreateWindow(1200, 900, "Quadcopter PID - Phuc hoi tu the", nullptr, nullptr);
//     glfwMakeContextCurrent(window);
//     glfwSwapInterval(1);
//     glfwSetMouseButtonCallback(window, mouse_button);
//     glfwSetCursorPosCallback(window, mouse_move);
//     glfwSetScrollCallback(window, scroll);
//     glfwSetKeyCallback(window, key_cb);
//     glfwSetCharCallback(window, char_cb);

//     IMGUI_CHECKVERSION();
//     ImGui::CreateContext();
//     ImGui_ImplGlfw_InitForOpenGL(window, false);
//     ImGui_ImplOpenGL3_Init("#version 130");

//     mjv_defaultCamera(&cam);
//     mjv_defaultOption(&opt);
//     mjv_defaultScene(&scn);
//     mjr_defaultContext(&con);
//     mjv_makeScene(m, &scn, 2000);
//     mjr_makeContext(m, &con, mjFONTSCALE_150);
//     cam.distance = 3.0;

//     while (!glfwWindowShouldClose(window)) {
//         mjtNum simstart = d->time;

//         if (recovery_active) {
//         while (d->time - simstart < 1.0 / 60.0) {
//             double dt = m->opt.timestep;

//             double x = d->qpos[0], y = d->qpos[1], z = d->qpos[2];
//             double vx = d->qvel[0], vy = d->qvel[1], vz = d->qvel[2];

//             double bz[3]; body_z_axis(&d->qpos[3], bz);
//             double roll, pitch, yaw;
//             quat_to_euler(&d->qpos[3], roll, pitch, yaw);

//             double z_pid_out = pid_z.update(equilibrium[2] - z, vz, dt);
//             double tilt_loss = std::max(bz[2], 0.35);
//             double T = std::max((mass * g + z_pid_out) / tilt_loss, 0.0);

//             double pitch_des =  std::clamp(pid_x.update(equilibrium[0] - x, vx, dt), -max_tilt, max_tilt);
//             double roll_des  = -std::clamp(pid_y.update(equilibrium[1] - y, vy, dt), -max_tilt, max_tilt);

//             double cr = std::cos(roll_des * 0.5), sr = std::sin(roll_des * 0.5);
//             double cp = std::cos(pitch_des * 0.5), sp = std::sin(pitch_des * 0.5);
//             double q_des[4] = { cr * cp, sr * cp, cr * sp, -sr * sp };

//             const mjtNum* q = &d->qpos[3];
//             double q_des_conj[4] = { q_des[0], -q_des[1], -q_des[2], -q_des[3] };
//             double q_err[4] = {
//                 q_des_conj[0]*q[0] - q_des_conj[1]*q[1] - q_des_conj[2]*q[2] - q_des_conj[3]*q[3],
//                 q_des_conj[0]*q[1] + q_des_conj[1]*q[0] + q_des_conj[2]*q[3] - q_des_conj[3]*q[2],
//                 q_des_conj[0]*q[2] - q_des_conj[1]*q[3] + q_des_conj[2]*q[0] + q_des_conj[3]*q[1],
//                 q_des_conj[0]*q[3] + q_des_conj[1]*q[2] - q_des_conj[2]*q[1] + q_des_conj[3]*q[0]
//             };
//             if (q_err[0] < 0) { q_err[0]=-q_err[0]; q_err[1]=-q_err[1]; q_err[2]=-q_err[2]; q_err[3]=-q_err[3]; }

//             // ----- Tang Angle: sai so goc (vector quaternion, on dinh moi goc) -> rate dat -----
//             double e_roll  = 2.0 * q_err[1];
//             double e_pitch = 2.0 * q_err[2];
//             double rate_des_x = std::clamp(pid_angle_roll.update(-e_roll, 0.0, dt), -RATE_MAX, RATE_MAX);
//             double rate_des_y = std::clamp(pid_angle_pitch.update(-e_pitch, 0.0, dt), -RATE_MAX, RATE_MAX);
//             double rate_des_z = std::clamp(pid_angle_yaw.update(0 - yaw, 0.0, dt), -RATE_MAX, RATE_MAX);

//             // ----- Tang Rate (gyro): sai so toc do goc -> mo-men, day moi la tang nhanh nhat -----
//             double wx = d->qvel[3], wy = d->qvel[4], wz = d->qvel[5];
//             double taux = pid_rate_roll.update(rate_des_x - wx, wx, dt);
//             double tauy = pid_rate_pitch.update(rate_des_y - wy, wy, dt);
//             double tauz = pid_rate_yaw.update(rate_des_z - wz, 0.0, dt);

//             double des[4] = {T, taux, tauy, tauz};
//             for (int i = 0; i < 4; i++) {
//                 double F = 0;
//                 for (int j = 0; j < 4; j++) F += Minv[i][j] * des[j];
//                 d->ctrl[i] = std::clamp(F, 0.0, ctrl_max);
//             }

//             mj_step(m, d);

//             log_counter++;
//             if (log_counter % 5 == 0) {
//                 // Do lech (co dau) so voi diem can bang theo tung truc Ox, Oy, Oz
//                 double x_error = x - equilibrium[0];
//                 double y_error = y - equilibrium[1];
//                 double z_error = z - equilibrium[2];
//                 double tilt_deg = tilt_from_vertical(&d->qpos[3]) * 180.0 / 3.14159265358979323846;
//                 double elapsed = d->time - recovery_start_time;

//                 recovery_log << elapsed << "," << x_error << "," << y_error << ","
//                              << z_error << "," << tilt_deg << ","
//                              << d->ctrl[0] << "," << d->ctrl[1] << "," << d->ctrl[2] << "," << d->ctrl[3] << "\n";
//                 recovery_log.flush();
//             }
//         }
//         }

//         int width, height;
//         glfwGetFramebufferSize(window, &width, &height);
//         mjrRect viewport = {0, 0, width, height};
//         mjv_updateScene(m, d, &opt, nullptr, &cam, mjCAT_ALL, &scn);
//         mjr_render(viewport, &scn, &con);

//         ImGui_ImplOpenGL3_NewFrame();
//         ImGui_ImplGlfw_NewFrame();
//         ImGui::NewFrame();

//         ImGui::Begin("Phuc hoi tu the (roll+pitch 85 do)");
//         ImGui::Text("Vi tri hien tai: (%.2f, %.2f, %.2f)", d->qpos[0], d->qpos[1], d->qpos[2]);
//         ImGui::Separator();
//         ImGui::Text("Vi tri/do cao can bang (muc tieu phuc hoi):");
//         ImGui::SliderFloat("X can bang (m)", &equilibrium[0], -3.0f, 3.0f);
//         ImGui::SliderFloat("Y can bang (m)", &equilibrium[1], -3.0f, 3.0f);
//         ImGui::SliderFloat("Z can bang (m)", &equilibrium[2], 0.2f, 3.0f);
//         ImGui::Spacing();
//         if (ImGui::Button("Bat dau thu nghiem (lat roll+pitch 85 do)", ImVec2(260, 30))) {
//             start_recovery_test();
//         }

//         if (!recovery_ever_started) {
//             ImGui::TextDisabled("Bam nut tren de bat dau: drone se bi dat lat nghieng");
//             ImGui::TextDisabled("85 do, sau do bo dieu khien PID se tu phuc hoi ve");
//             ImGui::TextDisabled("dung tu the va giu dung vi tri/do cao can bang o tren.");
//         } else {
//             double cur_tilt = tilt_from_vertical(&d->qpos[3]) * 180.0 / 3.14159265358979323846;
//             double cur_z_err = d->qpos[2] - equilibrium[2];
//             ImGui::Text("Goc lech hien tai: %.1f do", cur_tilt);
//             ImGui::Text("Do lech truc Oz hien tai: %.3f m", cur_z_err);
//             ImGui::Text(recovery_active ? "Trang thai: DANG CHAY" : "Trang thai: DA DUNG (bam nut de chay lai)");
//             ImGui::Separator();
//             ImGui::TextDisabled("Bieu do se hien o 1 cua so rieng ngay khi ban");
//             ImGui::TextDisabled("dong cua so mo phong nay.");
//         }
//         ImGui::End();

//         ImGui::Render();
//         ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

//         glfwSwapBuffers(window);
//         glfwPollEvents();
//     }

//     if (recovery_ever_started) {
//         recovery_log.close();
//         std::printf("Da luu du lieu phuc hoi vao recovery_log.csv\n");
//         std::printf("Dang tu dong mo bieu do do lech theo thoi gian...\n");
//         int plot_ret = std::system("python Plot_trajectory.py recovery_log.csv");
//         if (plot_ret != 0) {
//             std::printf("Khong tu chay duoc script Python (ma loi %d).\n", plot_ret);
//             std::printf("Chay tay bang lenh: python Plot_trajectory.py recovery_log.csv\n");
//         }
//     }

//     ImGui_ImplOpenGL3_Shutdown();
//     ImGui_ImplGlfw_Shutdown();
//     ImGui::DestroyContext();

//     mjv_freeScene(&scn);
//     mjr_freeContext(&con);
//     mj_deleteData(d);
//     mj_deleteModel(m);
//     glfwTerminate();
//     return 0;
// }