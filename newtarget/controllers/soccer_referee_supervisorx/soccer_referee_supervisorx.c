#include <stdio.h>
#include <string.h>
#include <webots/emitter.h>
#include <webots/robot.h>
#include <webots/supervisor.h>
 
#define ROBOTS 6
#define TIME_STEP 64
#define MATCH_DURATION 60000 *2 // ms
#define GOAL_X_LIMIT 0.76     // position X où la balle est considérée comme dans le but
 
static void set_scores(int b, int y) {
  char score[16];
 
  sprintf(score, "%d", b);
  wb_supervisor_set_label(0, score, 0.92, 0.01, 0.1, 0xff0000, 0.0, "Arial");  // rouge (équipe bleue)
 
  sprintf(score, "%d", y);
  wb_supervisor_set_label(1, score, 0.05, 0.01, 0.1, 0x0000ff, 0.0, "Arial");  // bleu (équipe jaune)
}
 
static void set_time(double time) {
  char time_string[16];
 
  if (time < 0) time = 0;
  sprintf(time_string, "%02d:%02d", (int)(time / 60), (int)time % 60);
  wb_supervisor_set_label(2, time_string, 0.45, 0.01, 0.1, 0x000000, 0.0, "Arial");
}
 
int main() {
  const char *robot_name[ROBOTS] = {"B1", "B2", "B3", "Y1", "Y2", "Y3"};
  WbNodeRef robot_node[ROBOTS], ball_node;
  WbFieldRef robot_translation_field[ROBOTS], robot_rotation_field[ROBOTS], ball_translation_field;
  WbDeviceTag emitter;
 
  double packet[ROBOTS * 4 + 3];
  double robot_initial_translation[ROBOTS][3], robot_initial_rotation[ROBOTS][4];
  double ball_initial_translation[3] = {0, 0.0, 0};
  double ball_reset_timer = 0.0;
 
  double initial_positions[ROBOTS][3] = {
    {0.3, -0.3, 0.0375}, {0.3, 0.3, 0.0375}, {0.75, 0.0, 0.0375},
    {-0.3, -0.3, 0.0375}, {-0.3, 0.3, 0.0375}, {-0.75, 0.0, 0.0375}
  };
  double initial_rotations[ROBOTS][4] = {
    {0, 1, 0, 0}, {0, 1, 0, 0}, {-0.00021, 0.00021, 1, 1.604},
    {0, 1, 0, 0}, {0, 1, 0, 0}, {-0.00021, 0.00021, 1, 1.604}
  };
 
  wb_robot_init();
  emitter = wb_robot_get_device("emitter");
 
  // Init robots
  for (int i = 0; i < ROBOTS; i++) {
    robot_node[i] = wb_supervisor_node_get_from_def(robot_name[i]);
    robot_translation_field[i] = wb_supervisor_node_get_field(robot_node[i], "translation");
    robot_rotation_field[i] = wb_supervisor_node_get_field(robot_node[i], "rotation");
 
    wb_supervisor_field_set_sf_vec3f(robot_translation_field[i], initial_positions[i]);
    wb_supervisor_field_set_sf_rotation(robot_rotation_field[i], initial_rotations[i]);
 
    // Sauvegarde pour reset
    memcpy(robot_initial_translation[i], initial_positions[i], sizeof(double) * 3);
    memcpy(robot_initial_rotation[i], initial_rotations[i], sizeof(double) * 4);
  }
 
  // Balle
  ball_node = wb_supervisor_node_get_from_def("BALL");
  ball_translation_field = wb_supervisor_node_get_field(ball_node, "translation");
  wb_supervisor_field_set_sf_vec3f(ball_translation_field, ball_initial_translation);
 
  int score_blue = 0, score_yellow = 0;
  double time = MATCH_DURATION / 1000.0;  // en secondes
 
  set_scores(score_blue, score_yellow);
  set_time(time);
 
  while (wb_robot_step(TIME_STEP) != -1 && time > 0) {
    time -= (double)TIME_STEP / 1000.0;
 
    const double *ball_translation = wb_supervisor_field_get_sf_vec3f(ball_translation_field);
 
    // Détection but et temporisation de reset
    if (ball_reset_timer == 0) {
      if (ball_translation[0] > GOAL_X_LIMIT) {
        set_scores(++score_blue, score_yellow);
        ball_reset_timer = 1.0;  // 3 secondes d'attente
      } else if (ball_translation[0] < -GOAL_X_LIMIT) {
        set_scores(score_blue, ++score_yellow);
        ball_reset_timer = 1.0;
      }
    } else {
      ball_reset_timer -= (double)TIME_STEP / 1000.0;
      if (ball_reset_timer <= 0) {
        ball_reset_timer = 0;
        wb_supervisor_field_set_sf_vec3f(ball_translation_field, ball_initial_translation);
        for (int i = 0; i < ROBOTS; i++) {
          wb_supervisor_field_set_sf_vec3f(robot_translation_field[i], robot_initial_translation[i]);
          wb_supervisor_field_set_sf_rotation(robot_rotation_field[i], robot_initial_rotation[i]);
        }
      }
    }
 
    // Mise à jour positions des robots dans le packet
    for (int i = 0; i < ROBOTS; i++) {
      const double *pos = wb_supervisor_field_get_sf_vec3f(robot_translation_field[i]);
      const double *rot = wb_supervisor_field_get_sf_rotation(robot_rotation_field[i]);
      packet[4 * i]     = pos[0];
      packet[4 * i + 1] = pos[1];
      packet[4 * i + 2] = pos[2];
      packet[4 * i + 3] = rot[3] * (rot[2] >= 0 ? 1 : -1);
    }
 
    packet[ROBOTS * 4 + 0] = ball_translation[0];
    packet[ROBOTS * 4 + 1] = ball_translation[1];
    packet[ROBOTS * 4 + 2] = ball_translation[2];
 
    set_time(time);
    wb_emitter_send(emitter, packet, sizeof(packet));
  }
 
  set_time(0);
  wb_robot_cleanup();
  return 0;
}