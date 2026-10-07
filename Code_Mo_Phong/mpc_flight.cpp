#define NSTATES 12
#define NINPUTS 4
#define NHORIZON 30

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cmath>
#include <algorithm>
#include <fstream>

#include <GLFW/glfw3.h>
#include <mujoco/mujoco.h>
#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"
#include <tinympc/tiny_api.hpp>

// ----- Bien toan cuc MuJoCo -----
mjModel* m = NULL;
mjData*  d = NULL;
mjvCamera cam;
mjvOption opt;
mjvScene  scn;
mjrContext con;

bool button_left = false, button_middle = false, button_right = false;
double lastx = 0, lasty = 0;

void mouse_button(GLFWwindow* window, int button, int act, int mods) {
    ImGui_ImplGlfw_MouseButtonCallback(window, button, act, mods);
    if (ImGui::GetIO().WantCaptureMouse) return;
    button_left   = glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT)  == GLFW_PRESS;
    button_middle = glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE)== GLFW_PRESS;
    button_right  = glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS;
    glfwGetCursorPos(window, &lastx, &lasty);
}

void mouse_move(GLFWwindow* window, double xpos, double ypos) {
    ImGui_ImplGlfw_CursorPosCallback(window, xpos, ypos);
    if (ImGui::GetIO().WantCaptureMouse) return;
    if (!button_left && !button_middle && !button_right) return;
    double dx = xpos - lastx, dy = ypos - lasty;
    lastx = xpos; lasty = ypos;
    int width, height;
    glfwGetWindowSize(window, &width, &height);
    bool mod_shift = glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS;
    mjtMouse action;
    if (button_right)       action = mod_shift ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V;
    else if (button_left)   action = mod_shift ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V;
    else                    action = mjMOUSE_ZOOM;
    mjv_moveCamera(m, action, dx / height, dy / height, &cam);
}

void scroll(GLFWwindow* window, double xoffset, double yoffset) {
    ImGui_ImplGlfw_ScrollCallback(window, xoffset, yoffset);
    if (ImGui::GetIO().WantCaptureMouse) return;
    mjv_moveCamera(m, mjMOUSE_ZOOM, 0, -0.05 * yoffset, &cam);
}

void key_cb(GLFWwindow* window, int key, int scancode, int action, int mods) {
    ImGui_ImplGlfw_KeyCallback(window, key, scancode, action, mods);
}

void char_cb(GLFWwindow* window, unsigned int c) {
    ImGui_ImplGlfw_CharCallback(window, c);
}

// ----- Goc lech giua truc than (Z) va phuong thang dung, on dinh moi goc -----
double tilt_from_vertical(const mjtNum q[4]) {
    double x = q[1], y = q[2];
    double R22 = std::clamp(1.0 - 2.0 * (x * x + y * y), -1.0, 1.0);
    return std::acos(R22);
}

// ----- Quaternion [w,x,y,z] -> Rodrigues parameters p = 2*(x,y,z)/w, chon duong quay ngan nhat -----
void quat_to_rodrigues(const mjtNum q[4], double rod[3]) {
    double w = q[0], x = q[1], y = q[2], z = q[3];
    if (w < 0) { w = -w; x = -x; y = -y; z = -z; }
    w = std::max(w, 1e-6);
    rod[0] = 2.0 * x / w;
    rod[1] = 2.0 * y / w;
    rod[2] = 2.0 * z / w;
}

int main(int argc, char** argv) {
    if (argc < 2) {
        std::printf("Cach dung: %s duong_dan_toi_quad.xml\n", argv[0]);
        return 1;
    }

    char error[1000] = "";
    m = mj_loadXML(argv[1], nullptr, error, sizeof(error));
    if (!m) {
        std::printf("Loi nap model: %s\n", error);
        return 1;
    }
    d = mj_makeData(m);

    double mass = 0;
    for (int i = 0; i < m->nbody; i++) mass += m->body_mass[i];
    double g = 9.81;
    int id_motor1 = mj_name2id(m, mjOBJ_SITE, "motor1");
    double a = m->site_pos[3 * id_motor1 + 1];
    double b = m->site_pos[3 * id_motor1 + 0];
    int id_thrust1 = mj_name2id(m, mjOBJ_ACTUATOR, "thrust1");
    double k = m->actuator_gear[6 * id_thrust1 + 5];
    double ctrl_max = m->actuator_ctrlrange[2 * id_thrust1 + 1];
    double hover_per_motor = mass * g / 4.0;
    if (ctrl_max < hover_per_motor * 2.0) {
        std::printf("CANH BAO: ctrlrange toi da (%.3f N) chua gap doi luc day hover (%.3f N).\n", ctrl_max, hover_per_motor);
    }
    // Doc DUNG quan tinh tu chinh file XML (khong hard-code) -- neu quan tinh that
    // trong XML khac voi gia tri gia dinh, Ox/Oy se bi dieu khien sai (lech qua
    // goc nghieng) trong khi Oz van dung (Oz khong phu thuoc quan tinh, chi phu
    // thuoc khoi luong) -- day la nguyen nhan hay gap nhat cho trieu chung
    // "Oz on dinh nhanh nhung Ox/Oy dao dong/cham on dinh".
    int id_quad_body = mj_name2id(m, mjOBJ_BODY, "quad");
    if (id_quad_body < 0) {
        std::printf("CANH BAO: khong tim thay body ten \"quad\" trong XML, dung body dau tien co quan tinh.\n");
        id_quad_body = 1;
    }
    const double Ixx = m->body_inertia[3*id_quad_body + 0];
    const double Iyy = m->body_inertia[3*id_quad_body + 1];
    const double Izz = m->body_inertia[3*id_quad_body + 2];
    std::printf("Quan tinh doc tu XML: Ixx=%.3e Iyy=%.3e Izz=%.3e\n", Ixx, Iyy, Izz);

    // ----- Dung model tuyen tinh hoa quanh hover (Ac,Bc,fc) roi roi rac hoa Euler -----
    const double dt_mpc = 0.01;   // TinyMPC giai o 100Hz
    tinyMatrix Adyn = tinyMatrix::Identity(NSTATES, NSTATES);
    tinyMatrix Bdyn = tinyMatrix::Zero(NSTATES, NINPUTS);
    tinyMatrix fdyn = tinyMatrix::Zero(NSTATES, 1);

    Adyn(0,6) += dt_mpc; Adyn(1,7) += dt_mpc; Adyn(2,8) += dt_mpc;      // pos_dot = vel
    Adyn(3,9) += dt_mpc; Adyn(4,10) += dt_mpc; Adyn(5,11) += dt_mpc;    // rod_dot = omega (xap xi goc nho)
    Adyn(6,4) += g * dt_mpc;    // vx_dot =  g*theta
    Adyn(7,3) += -g * dt_mpc;   // vy_dot = -g*phi
    Bdyn(8,0) = dt_mpc/mass; Bdyn(8,1) = dt_mpc/mass; Bdyn(8,2) = dt_mpc/mass; Bdyn(8,3) = dt_mpc/mass;  // vz_dot = T/m
    fdyn(8,0) = -g * dt_mpc;    // vz_dot -= g
    // w_dot = torque/I, dung dung ma tran mixer: taux=a(F1-F2+F3-F4); tauy=b(-F1+F2+F3-F4); tauz=k(F1+F2-F3-F4)
    Bdyn(9,0)=a/Ixx*dt_mpc;   Bdyn(9,1)=-a/Ixx*dt_mpc;  Bdyn(9,2)=a/Ixx*dt_mpc;   Bdyn(9,3)=-a/Ixx*dt_mpc;
    Bdyn(10,0)=-b/Iyy*dt_mpc; Bdyn(10,1)=b/Iyy*dt_mpc;  Bdyn(10,2)=b/Iyy*dt_mpc;  Bdyn(10,3)=-b/Iyy*dt_mpc;
    Bdyn(11,0)=k/Izz*dt_mpc;  Bdyn(11,1)=k/Izz*dt_mpc;  Bdyn(11,2)=-k/Izz*dt_mpc; Bdyn(11,3)=-k/Izz*dt_mpc;

    // ----- Trong so chi phi: uu tien vi tri/do cao, nhe hon cho goc/van toc, phat nhe lenh dieu khien -----
    tinyMatrix Qdiag(NSTATES, 1);
    Qdiag << 1500, 1500, 400,  35, 35, 12,  120, 120, 25,  12, 12, 3;
    tinyMatrix Rdiag(NINPUTS, 1);
    Rdiag << 0.008, 0.008, 0.008, 0.008;

    TinySolver* solver;
    tiny_setup(&solver, Adyn, Bdyn, fdyn, Qdiag.asDiagonal(), Rdiag.asDiagonal(),
               5.0, NSTATES, NINPUTS, NHORIZON, 0);

    tinyMatrix x_min = tinyMatrix::Constant(NSTATES, NHORIZON, -1000.0);
    tinyMatrix x_max = tinyMatrix::Constant(NSTATES, NHORIZON, 1000.0);
    tinyMatrix u_min = tinyMatrix::Constant(NINPUTS, NHORIZON - 1, 0.0);
    tinyMatrix u_max = tinyMatrix::Constant(NINPUTS, NHORIZON - 1, ctrl_max);   // dung gioi han luc day that
    tiny_set_bound_constraints(solver, x_min, x_max, u_min, u_max);
    solver->settings->max_iter = 50;

    float equilibrium[3] = {0.0f, 0.0f, 1.5f};
    bool recovery_active = false;
    bool recovery_ever_started = false;
    double recovery_start_time = 0.0;
    int log_counter = 0;
    int mpc_counter = 0;
    const int mpc_every_n_steps = (int)std::round(dt_mpc / m->opt.timestep);
    double last_u[4] = {0, 0, 0, 0};
    std::ofstream recovery_log;

    auto start_recovery_test = [&]() {
        mj_resetData(m, d);
        d->qpos[0] = equilibrium[0];
        d->qpos[1] = equilibrium[1];
        d->qpos[2] = equilibrium[2];
        double cr = std::cos(85.0 * 3.14159265358979323846 / 360.0);
        double sr = std::sin(85.0 * 3.14159265358979323846 / 360.0);
        d->qpos[3] = cr * cr;  d->qpos[4] = sr * cr;
        d->qpos[5] = cr * sr;  d->qpos[6] = -sr * sr;
        mj_forward(m, d);

        tinyVector Xref = tinyVector::Zero(NSTATES);
        Xref(0) = equilibrium[0]; Xref(1) = equilibrium[1]; Xref(2) = equilibrium[2];
        solver->work->Xref = Xref.replicate<1, NHORIZON>();
        last_u[0] = last_u[1] = last_u[2] = last_u[3] = mass * g / 4.0;
        mpc_counter = 0;

        recovery_start_time = d->time;
        recovery_active = true;
        recovery_ever_started = true;

        recovery_log.close();
        recovery_log.open("recovery_log.csv");
        recovery_log << "time,x_error_m,y_error_m,z_error_m,tilt_error_deg,motor1_N,motor2_N,motor3_N,motor4_N\n";
    };

    if (!glfwInit()) { std::printf("Khong khoi tao duoc GLFW\n"); return 1; }
    GLFWwindow* window = glfwCreateWindow(1200, 900, "Quadcopter TinyMPC - Phuc hoi tu the", nullptr, nullptr);
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);
    glfwSetMouseButtonCallback(window, mouse_button);
    glfwSetCursorPosCallback(window, mouse_move);
    glfwSetScrollCallback(window, scroll);
    glfwSetKeyCallback(window, key_cb);
    glfwSetCharCallback(window, char_cb);

    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGui_ImplGlfw_InitForOpenGL(window, false);
    ImGui_ImplOpenGL3_Init("#version 130");

    mjv_defaultCamera(&cam);
    mjv_defaultOption(&opt);
    mjv_defaultScene(&scn);
    mjr_defaultContext(&con);
    mjv_makeScene(m, &scn, 2000);
    mjr_makeContext(m, &con, mjFONTSCALE_150);
    cam.distance = 3.0;

    while (!glfwWindowShouldClose(window)) {
        mjtNum simstart = d->time;

        if (recovery_active) {
        while (d->time - simstart < 1.0 / 60.0) {

            // ----- Giai MPC moi dt_mpc (100Hz), giu nguyen lenh giua 2 lan giai (zero-order hold) -----
            if (mpc_counter % mpc_every_n_steps == 0) {
                double rod[3]; quat_to_rodrigues(&d->qpos[3], rod);
                tinyVector x0(NSTATES);
                x0 << d->qpos[0], d->qpos[1], d->qpos[2],
                      rod[0], rod[1], rod[2],
                      d->qvel[0], d->qvel[1], d->qvel[2],
                      d->qvel[3], d->qvel[4], d->qvel[5];

                tiny_set_x0(solver, x0);
                tiny_solve(solver);
                for (int j = 0; j < 4; j++)
                    last_u[j] = std::clamp((double)solver->work->u(j, 0), 0.0, ctrl_max);
            }
            mpc_counter++;

            for (int j = 0; j < 4; j++) d->ctrl[j] = last_u[j];
            mj_step(m, d);

            log_counter++;
            if (log_counter % 5 == 0) {
                double x_error = d->qpos[0] - equilibrium[0];
                double y_error = d->qpos[1] - equilibrium[1];
                double z_error = d->qpos[2] - equilibrium[2];
                double tilt_deg = tilt_from_vertical(&d->qpos[3]) * 180.0 / 3.14159265358979323846;
                double elapsed = d->time - recovery_start_time;
                recovery_log << elapsed << "," << x_error << "," << y_error << "," << z_error << "," << tilt_deg << ","
                             << d->ctrl[0] << "," << d->ctrl[1] << "," << d->ctrl[2] << "," << d->ctrl[3] << "\n";
                recovery_log.flush();
            }
        }
        }

        int width, height;
        glfwGetFramebufferSize(window, &width, &height);
        mjrRect viewport = {0, 0, width, height};
        mjv_updateScene(m, d, &opt, nullptr, &cam, mjCAT_ALL, &scn);
        mjr_render(viewport, &scn, &con);

        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        ImGui::Begin("Phuc hoi tu the - TinyMPC (roll+pitch 85 do)");
        ImGui::Text("Vi tri hien tai: (%.2f, %.2f, %.2f)", d->qpos[0], d->qpos[1], d->qpos[2]);
        ImGui::Separator();
        ImGui::Text("Vi tri/do cao can bang (muc tieu phuc hoi):");
        ImGui::SliderFloat("X can bang (m)", &equilibrium[0], -3.0f, 3.0f);
        ImGui::SliderFloat("Y can bang (m)", &equilibrium[1], -3.0f, 3.0f);
        ImGui::SliderFloat("Z can bang (m)", &equilibrium[2], 0.2f, 3.0f);
        ImGui::Spacing();
        if (ImGui::Button("Bat dau thu nghiem (lat roll+pitch 85 do)", ImVec2(260, 30))) {
            start_recovery_test();
        }

        if (!recovery_ever_started) {
            ImGui::TextDisabled("Bam nut tren de bat dau: drone se bi dat lat nghieng");
            ImGui::TextDisabled("85 do, sau do TinyMPC se tu phuc hoi ve dung tu the");
            ImGui::TextDisabled("va giu dung vi tri/do cao can bang o tren.");
        } else {
            double cur_tilt = tilt_from_vertical(&d->qpos[3]) * 180.0 / 3.14159265358979323846;
            double cur_z_err = d->qpos[2] - equilibrium[2];
            ImGui::Text("Goc lech hien tai: %.1f do", cur_tilt);
            ImGui::Text("Do lech truc Oz hien tai: %.3f m", cur_z_err);
            ImGui::Text(recovery_active ? "Trang thai: DANG CHAY" : "Trang thai: DA DUNG (bam nut de chay lai)");
            ImGui::Separator();
            ImGui::TextDisabled("Bieu do se hien o 1 cua so rieng ngay khi ban");
            ImGui::TextDisabled("dong cua so mo phong nay.");
        }
        ImGui::End();

        ImGui::Render();
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

        glfwSwapBuffers(window);
        glfwPollEvents();
    }

    if (recovery_ever_started) {
        recovery_log.close();
        std::printf("Da luu du lieu phuc hoi vao recovery_log.csv\n");
        std::printf("Dang tu dong mo bieu do do lech theo thoi gian...\n");
        int plot_ret = std::system("python Plot_trajectory.py recovery_log.csv");
        if (plot_ret != 0) {
            std::printf("Khong tu chay duoc script Python (ma loi %d).\n", plot_ret);
            std::printf("Chay tay bang lenh: python Plot_trajectory.py recovery_log.csv\n");
        }
    }

    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();

    mjv_freeScene(&scn);
    mjr_freeContext(&con);
    mj_deleteData(d);
    mj_deleteModel(m);
    glfwTerminate();
    return 0;
}