#define _POSIX_C_SOURCE 200809L

/*
 * Persistent IH01 EtherCAT controller for the local OpenCV dashboard.
 * One process owns SOEM, keeps the verified 2 ms/DC/vendor command sequence,
 * accepts line commands on stdin, and publishes JSONL feedback on stdout.
 * Author: haoming
 */
#include <errno.h>
#include <inttypes.h>
#include <poll.h>
#include <signal.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "ethercat.h"

#define VENDOR_ID 0x00041101U
#define PRODUCT_CODE 0x08080808U
#define OUTPUT_BYTES 16
#define INPUT_BYTES 64
#define CONTROL_OFFSET 12
#define MODE_COMMAND 0x01U
#define SPEED_COMMAND 0x02U
#define POSITION_COMMAND 0x04U
#define CYCLE_US 2000U
#define DC_CYCLE_NS 2000000U
#define DC_SHIFT_NS 700
#define STAGE_CYCLES 10
#define STARTUP_CYCLES 100
#define FEEDBACK_DIVIDER 25
#define MAX_BAD_WKC 5
#define CONTACT_CURRENT_MA 1000
#define CONTACT_PRETRIP_CURRENT_MA 600
#define CONTACT_PRETRIP_STALL_CYCLES 3 /* 6 ms at the 2 ms PDO cycle */
#define CONTACT_FORCE_RAW 5000
#define CONTACT_STALL_CYCLES 100 /* 200 ms at the verified 2 ms PDO cycle */
#define CONTACT_ENDPOINT_MARGIN 100
#define CONTACT_STALL_DELTA_STEPS 10

enum { CONTACT_MODE_VISUAL = 0, CONTACT_MODE_QUEST = 1 };

typedef struct {
  int16_t position[6];
  int16_t current[6];
  int16_t force[6];
  int16_t temperature[6];
  int16_t fault[6];
  uint8_t status;
} snapshot_t;

typedef struct {
  const char *side;
  int slave;
  int enabled;
  int command_active;
  int speed_update_cycles;
  int fault_reset_cycles;
  int fault_latched;
  int16_t requested[6];
  int16_t target[6];
  /* Snapshot captured once when contact is detected.  Keeping this fixed
   * avoids chasing a moving feedback position and repeatedly correcting the
   * motors while the trigger remains held. */
  int16_t hold_target[6];
  int16_t speed[6];
  int16_t previous_position[6];
  uint8_t stall_cycles[6];
  uint16_t contact_stall_limit;
  int contact_mode;
  uint8_t contact_hold[6];
  uint8_t grasp_hold;
  int soft_limit_enabled;
  int soft_limit_active;
  int soft_limit_ch5_max;
  snapshot_t feedback;
} hand_t;

static int channel_limit(int channel);

static void update_contact_guard_visual(hand_t *hand) {
  int opening = 1;
  for (int channel = 0; channel < 6; ++channel)
    if (hand->requested[channel] > 20) opening = 0;
  if (opening) {
    memset(hand->contact_hold, 0, sizeof(hand->contact_hold));
    memset(hand->stall_cycles, 0, sizeof(hand->stall_cycles));
    return;
  }
  /* Keep the original visual-teleop behavior: each channel is evaluated and
   * released independently.  This path deliberately does not use the Quest
   * grasp latch or its force/end-stop heuristics. */
  for (int channel = 0; channel < 6; ++channel) {
    const int position = hand->feedback.position[channel];
    const int delta = position - hand->previous_position[channel];
    const int remaining = hand->requested[channel] - position;
    if (hand->contact_hold[channel]) {
      /* A target moving back toward open releases this channel immediately. */
      if (abs(remaining) <= 20 ||
          (hand->requested[channel] < position - 20 &&
           position >= hand->target[channel]))
        hand->contact_hold[channel] = 0;
    } else if (abs(remaining) > 25 && abs(delta) <= 2) {
      if (hand->stall_cycles[channel] < hand->contact_stall_limit)
        ++hand->stall_cycles[channel];
      if (hand->stall_cycles[channel] >= hand->contact_stall_limit ||
          abs(hand->feedback.current[channel]) >= CONTACT_CURRENT_MA) {
        hand->contact_hold[channel] = 1;
      }
    } else {
      hand->stall_cycles[channel] = 0;
    }
    hand->previous_position[channel] = (int16_t)position;
  }
  /* Keep this field false for visual teleop.  It is a Quest-only latch and
   * must not be allowed to alter the visual command lifecycle. */
  hand->grasp_hold = 0;
}

/* Measured coupled envelope derived from:
 * calibration/thumb-index-soft-limit/left-20260814-084315.json
 * plus the dedicated pinch contact target in ih01-pinch-mapping.json.
 * Rows are CH6={500,800,1100}; columns are CH4={800,900,1200,1600}.
 * Values subtract exactly 10 steps from measured contact boundaries. */
static const int soft_limit_ch4_axis[4] = {800, 900, 1200, 1600};
static const int soft_limit_ch6_axis[3] = {500, 800, 1100};
static const int ch5_max_grid[3][4] = {
    {1590, 1590, 1590, 840},
    {1590, 1490, 1190, 490},
    {1590, 1190, 950, 550},
};

static uint8_t io_map[8192] = {0};
static volatile sig_atomic_t stop_requested = 0;

static int parse_speed(const char *text, int *result);
static void print_six(const int16_t values[6]);

static void print_six_to(FILE *stream, const int16_t values[6]) {
  fputc('[', stream);
  for (int channel = 0; channel < 6; ++channel) {
    if (channel) fputc(',', stream);
    fprintf(stream, "%" PRId16, values[channel]);
  }
  fputc(']', stream);
}

static int interval_index(const int *axis, int length, int value) {
  for (int index = 0; index < length - 2; ++index)
    if (value <= axis[index + 1]) return index;
  return length - 2;
}

static int interpolate_floor(int low_value, int high_value, int coordinate,
                             int low_axis, int high_axis) {
  if (coordinate <= low_axis) return low_value;
  if (coordinate >= high_axis) return high_value;
  const int span = high_axis - low_axis;
  const int offset = coordinate - low_axis;
  return (low_value * (span - offset) + high_value * offset) / span;
}

static int thumb_flex_max(int index_flex, int thumb_rotate) {
  const int x_index = interval_index(soft_limit_ch4_axis, 4, index_flex);
  const int y_index = interval_index(soft_limit_ch6_axis, 3, thumb_rotate);
  const int x0 = soft_limit_ch4_axis[x_index];
  const int x1 = soft_limit_ch4_axis[x_index + 1];
  const int y0 = soft_limit_ch6_axis[y_index];
  const int y1 = soft_limit_ch6_axis[y_index + 1];
  const int low_row = interpolate_floor(
      ch5_max_grid[y_index][x_index], ch5_max_grid[y_index][x_index + 1],
      index_flex, x0, x1);
  const int high_row = interpolate_floor(
      ch5_max_grid[y_index + 1][x_index],
      ch5_max_grid[y_index + 1][x_index + 1], index_flex, x0, x1);
  return interpolate_floor(low_row, high_row, thumb_rotate, y0, y1);
}

static void apply_coupled_soft_limit(hand_t *hand) {
  memcpy(hand->target, hand->requested, sizeof(hand->target));
  hand->soft_limit_active = 0;
  hand->soft_limit_ch5_max = 1700;
  if (!hand->soft_limit_enabled) return;
  const int requested_limit = thumb_flex_max(hand->requested[3], hand->requested[5]);
  const int feedback_limit = thumb_flex_max(
      hand->feedback.position[3], hand->feedback.position[5]);
  hand->soft_limit_ch5_max =
      requested_limit < feedback_limit ? requested_limit : feedback_limit;
  if (hand->requested[4] > hand->soft_limit_ch5_max) {
    hand->target[4] = (int16_t)hand->soft_limit_ch5_max;
    hand->soft_limit_active = 1;
  }
}

static void update_contact_guard(hand_t *hand) {
  if (hand->contact_mode == CONTACT_MODE_VISUAL) {
    update_contact_guard_visual(hand);
    return;
  }
  int opening = 1;
  for (int channel = 0; channel < 6; ++channel)
    if (hand->requested[channel] > 20) opening = 0;
  if (opening) {
    hand->grasp_hold = 0;
    memset(hand->contact_hold, 0, sizeof(hand->contact_hold));
    memset(hand->stall_cycles, 0, sizeof(hand->stall_cycles));
    memset(hand->hold_target, 0, sizeof(hand->hold_target));
    /* Do not reuse the previous contact/force sample while opening. */
    return;
  }
  if (hand->grasp_hold) {
    /* Keep the trigger latch. Channels are frozen as soon as the first
     * contact/end-stop is detected; OPEN is the only release path. */
    for (int channel = 0; channel < 6; ++channel) {
      if (hand->contact_hold[channel]) {
        hand->previous_position[channel] = hand->feedback.position[channel];
        continue;
      }
      const int position = hand->feedback.position[channel];
      const int delta = position - hand->previous_position[channel];
      const int requested = hand->requested[channel];
      const int remaining = hand->requested[channel] - position;
      if (abs(remaining) > 25 && abs(delta) <= CONTACT_STALL_DELTA_STEPS) {
        if (hand->stall_cycles[channel] < hand->contact_stall_limit)
          ++hand->stall_cycles[channel];
      } else {
        hand->stall_cycles[channel] = 0;
      }
      if (abs(remaining) <= 25 ||
          (requested >= channel_limit(channel) - CONTACT_ENDPOINT_MARGIN &&
           position >= channel_limit(channel) - CONTACT_ENDPOINT_MARGIN &&
           abs(delta) <= CONTACT_STALL_DELTA_STEPS && hand->stall_cycles[channel] >= CONTACT_PRETRIP_STALL_CYCLES) ||
          (abs(delta) <= CONTACT_STALL_DELTA_STEPS && hand->stall_cycles[channel] >= CONTACT_PRETRIP_STALL_CYCLES &&
           (abs(hand->feedback.current[channel]) >= CONTACT_CURRENT_MA ||
            (abs(hand->feedback.current[channel]) >= CONTACT_PRETRIP_CURRENT_MA &&
             hand->stall_cycles[channel] >= CONTACT_PRETRIP_STALL_CYCLES) ||
           abs(hand->feedback.force[channel]) >= CONTACT_FORCE_RAW))) {
        hand->contact_hold[channel] = 1;
        hand->hold_target[channel] = (int16_t)position;
      }
      hand->previous_position[channel] = (int16_t)position;
    }
    return;
  }
  int contact_detected = 0;
  for (int channel = 0; channel < 6; ++channel) {
    const int position = hand->feedback.position[channel];
    const int requested = hand->requested[channel];
    const int delta = position - hand->previous_position[channel];
    const int remaining = requested - position;
    if (hand->contact_hold[channel]) {
      contact_detected = 1;
    } else if (abs(remaining) > 25 && abs(delta) <= CONTACT_STALL_DELTA_STEPS) {
      /* Some IH01 firmware reports zero current just before it latches an
       * overload.  Use sustained position stall as the fallback contact
       * signal; the short current threshold remains useful for fast contact. */
      if (hand->stall_cycles[channel] < hand->contact_stall_limit)
        ++hand->stall_cycles[channel];
      if (hand->stall_cycles[channel] >= hand->contact_stall_limit ||
          (requested >= channel_limit(channel) - CONTACT_ENDPOINT_MARGIN &&
           position >= channel_limit(channel) - CONTACT_ENDPOINT_MARGIN &&
           abs(delta) <= CONTACT_STALL_DELTA_STEPS && hand->stall_cycles[channel] >= CONTACT_PRETRIP_STALL_CYCLES) ||
          abs(hand->feedback.current[channel]) >= CONTACT_CURRENT_MA ||
          (hand->stall_cycles[channel] >= CONTACT_PRETRIP_STALL_CYCLES &&
           abs(hand->feedback.current[channel]) >= CONTACT_PRETRIP_CURRENT_MA) ||
          abs(hand->feedback.force[channel]) >= CONTACT_FORCE_RAW) {
        hand->contact_hold[channel] = 1;
        contact_detected = 1;
      }
    } else {
      hand->stall_cycles[channel] = 0;
    }
    hand->previous_position[channel] = (int16_t)position;
  }
  if (contact_detected) {
    hand->grasp_hold = 1;
    for (int channel = 0; channel < 6; ++channel) {
      /* Quest uses independent finger contact: freeze only channels that
       * actually reached contact/stall. Other fingers continue closing until
       * their own condition is met. OPEN explicitly releases all channels. */
      if (hand->contact_hold[channel]) {
        hand->hold_target[channel] = hand->feedback.position[channel];
        hand->target[channel] = hand->hold_target[channel];
      }
    }
  }
}

static void apply_contact_guard(hand_t *hand) {
  if (hand->contact_mode == CONTACT_MODE_QUEST && hand->grasp_hold) {
    for (int channel = 0; channel < 6; ++channel)
      if (hand->contact_hold[channel])
        hand->target[channel] = hand->hold_target[channel];
    return;
  }
  for (int channel = 0; channel < 6; ++channel)
    if (hand->contact_hold[channel])
      hand->target[channel] = hand->feedback.position[channel];
}

static void request_stop(int signal_number) {
  (void)signal_number;
  stop_requested = 1;
}

static int16_t read_i16(const uint8_t *base, int offset) {
  uint16_t raw = 0;
  memcpy(&raw, base + offset, sizeof(raw));
  return (int16_t)etohs(raw);
}

static void write_i16(uint8_t *base, int offset, int16_t value) {
  const uint16_t raw = htoes((uint16_t)value);
  memcpy(base + offset, &raw, sizeof(raw));
}

static void capture(hand_t *hand) {
  const uint8_t *inputs = ec_slave[hand->slave].inputs;
  for (int channel = 0; channel < 6; ++channel) {
    hand->feedback.position[channel] = read_i16(inputs, channel * 2);
    hand->feedback.current[channel] = read_i16(inputs, 12 + channel * 2);
    hand->feedback.force[channel] = read_i16(inputs, 24 + channel * 2);
    hand->feedback.temperature[channel] = read_i16(inputs, 36 + channel * 2);
    hand->feedback.fault[channel] = read_i16(inputs, 48 + channel * 2);
  }
  hand->feedback.status = inputs[60];
}

static void set_output(hand_t *hand, const int16_t values[6], uint8_t cw) {
  uint8_t *outputs = ec_slave[hand->slave].outputs;
  memset(outputs, 0, OUTPUT_BYTES);
  for (int channel = 0; channel < 6; ++channel)
    write_i16(outputs, channel * 2, values[channel]);
  outputs[CONTROL_OFFSET] = cw;
}

static void apply_frame(hand_t hands[2], const int16_t values[6], uint8_t cw) {
  for (int index = 0; index < 2; ++index)
    if (hands[index].enabled) set_output(&hands[index], values, cw);
}

static int exchange(hand_t hands[2], int expected_wkc) {
  ec_send_processdata();
  const int wkc = ec_receive_processdata(EC_TIMEOUTRET);
  for (int index = 0; index < 2; ++index)
    if (hands[index].enabled) capture(&hands[index]);
  (void)expected_wkc;
  return wkc;
}

static int hard_fault_present(const hand_t hands[2]) {
  for (int index = 0; index < 2; ++index) {
    if (!hands[index].enabled) continue;
    for (int channel = 0; channel < 6; ++channel)
      if (hands[index].feedback.fault[channel] != 0) return 1;
  }
  return 0;
}

static void print_startup_feedback(const hand_t hands[2], int wkc,
                                   int expected_wkc) {
  fprintf(stderr, "Startup PDO/feedback gate failed: WKC=%d/%d\n", wkc,
          expected_wkc);
  for (int index = 0; index < 2; ++index) {
    if (!hands[index].enabled) continue;
    fprintf(stderr, "%s slave %d status=%u position=", hands[index].side,
            hands[index].slave, hands[index].feedback.status);
    print_six_to(stderr, hands[index].feedback.position);
    fputs(" current=", stderr);
    print_six_to(stderr, hands[index].feedback.current);
    fputs(" fault=", stderr);
    print_six_to(stderr, hands[index].feedback.fault);
    fputc('\n', stderr);
  }
}

static int hold_shared(hand_t hands[2], const int16_t values[6], uint8_t cw,
                       int cycles, int expected_wkc) {
  for (int cycle = 0; cycle < cycles && !stop_requested; ++cycle) {
    apply_frame(hands, values, cw);
    if (exchange(hands, expected_wkc) < expected_wkc ||
        hard_fault_present(hands))
      return 0;
    osal_usleep(CYCLE_US);
  }
  return !stop_requested;
}

static void print_six(const int16_t values[6]) {
  print_six_to(stdout, values);
}

static void print_hand(const hand_t *hand) {
  fprintf(stdout, "{\"side\":\"%s\",\"slave\":%d,\"target_steps\":",
          hand->side, hand->slave);
  print_six(hand->target);
  fputs(",\"requested_steps\":", stdout);
  print_six(hand->requested);
  fputs(",\"contact_hold\":", stdout);
  fputc('[', stdout);
  for (int channel = 0; channel < 6; ++channel) {
    if (channel) fputc(',', stdout);
    fprintf(stdout, "%s", hand->contact_hold[channel] ? "true" : "false");
  }
  fputc(']', stdout);
  fprintf(stdout, ",\"grasp_hold\":%s", hand->grasp_hold ? "true" : "false");
  fputs(",\"position_steps\":", stdout);
  print_six(hand->feedback.position);
  fputs(",\"current_ma\":", stdout);
  print_six(hand->feedback.current);
  fputs(",\"force_raw\":", stdout);
  print_six(hand->feedback.force);
  fputs(",\"temperature_c\":", stdout);
  print_six(hand->feedback.temperature);
  fputs(",\"fault_code\":", stdout);
  print_six(hand->feedback.fault);
  fprintf(stdout,
          ",\"status_word\":%u,\"command_active\":%s,"
          "\"fault_latched\":%s,\"fault_reset_cycles\":%d,"
          "\"soft_limit_enabled\":%s,\"soft_limit_active\":%s,"
          "\"soft_limit_ch5_max\":%d}",
          hand->feedback.status, hand->command_active ? "true" : "false",
          hand->fault_latched ? "true" : "false", hand->fault_reset_cycles,
          hand->soft_limit_enabled ? "true" : "false",
          hand->soft_limit_active ? "true" : "false",
          hand->soft_limit_ch5_max);
}

static void print_feedback(const hand_t hands[2], uint64_t cycle, int wkc,
                           int expected_wkc) {
  fprintf(stdout,
          "{\"format\":\"ih01_dual_hand_control_v1\",\"cycle\":%" PRIu64
          ",\"state\":\"OP\",\"wkc\":%d,\"expected_wkc\":%d,\"hands\":[",
          cycle, wkc, expected_wkc);
  int printed = 0;
  for (int index = 0; index < 2; ++index) {
    if (!hands[index].enabled) continue;
    if (printed++) fputc(',', stdout);
    print_hand(&hands[index]);
  }
  fputs("]}\n", stdout);
  fflush(stdout);
}

static hand_t *find_hand(hand_t hands[2], char side) {
  if (side == 'L' || side == 'l') return hands[0].enabled ? &hands[0] : NULL;
  if (side == 'R' || side == 'r') return hands[1].enabled ? &hands[1] : NULL;
  return NULL;
}

static int channel_limit(int channel) {
  return channel == 5 ? 1300 : 1700;
}

static int clamp_target(int channel, int value) {
  if (value < 0) return 0;
  const int limit = channel_limit(channel);
  return value > limit ? limit : value;
}

static void handle_command(hand_t hands[2], const char *line) {
  char side = 0;
  int channel = 0;
  int value = 0;
  char speed_text[16] = {0};
  if (strncmp(line, "RESET", 5) == 0) {
    for (int index = 0; index < 2; ++index) {
      if (!hands[index].enabled) continue;
      /* Vendor procedure: CW5 pulse, then CW0. Keep the bus in a quiet
       * state long enough for the driver to clear its alarm. */
      hands[index].fault_reset_cycles = 25;
      hands[index].fault_latched = 0;
      hands[index].command_active = 0;
      memset(hands[index].contact_hold, 0, sizeof(hands[index].contact_hold));
      memset(hands[index].stall_cycles, 0, sizeof(hands[index].stall_cycles));
      hands[index].grasp_hold = 0;
      /* Do not leave the previous max-grasp request armed.  Freeze the
       * software target at the last safe feedback position until the next
       * explicit GRASP/OPEN command arrives. */
      for (int channel = 0; channel < 6; ++channel) {
        hands[index].requested[channel] = hands[index].feedback.position[channel];
        hands[index].target[channel] = hands[index].feedback.position[channel];
        hands[index].hold_target[channel] = hands[index].feedback.position[channel];
      }
    }
    fprintf(stderr, "Explicit fault reset requested: CW5 for 50 ms, then CW0; targets frozen until next command\n");
    return;
  }
  if (sscanf(line, "SPEED %15s", speed_text) == 1 &&
      parse_speed(speed_text, &value)) {
    for (int index = 0; index < 2; ++index) {
      if (!hands[index].enabled) continue;
      for (int channel = 0; channel < 6; ++channel)
        hands[index].speed[channel] = (int16_t)value;
      hands[index].speed_update_cycles = STAGE_CYCLES;
    }
    return;
  }
  if (sscanf(line, "SET %c %d %d", &side, &channel, &value) == 3) {
    hand_t *hand = find_hand(hands, side);
    if (hand && channel >= 1 && channel <= 6) {
      hand->requested[channel - 1] =
          (int16_t)clamp_target(channel - 1, value);
      apply_coupled_soft_limit(hand);
      hand->command_active = 1;
    }
    return;
  }

  int values[6] = {0};
  if (sscanf(line, "SETALL %c %d %d %d %d %d %d", &side, &values[0],
             &values[1], &values[2], &values[3], &values[4], &values[5]) == 7) {
    hand_t *hand = find_hand(hands, side);
    if (hand) {
      int opening = 1;
      for (int index = 0; index < 6; ++index)
        if ((hand->requested[index] = (int16_t)clamp_target(index, values[index])) > 20)
          opening = 0;
      if (opening) {
        /* OPEN is an explicit release command: clear the latch immediately,
         * before the next PDO cycle can inspect stale force feedback. */
        hand->grasp_hold = 0;
        memset(hand->contact_hold, 0, sizeof(hand->contact_hold));
        memset(hand->stall_cycles, 0, sizeof(hand->stall_cycles));
        memset(hand->hold_target, 0, sizeof(hand->hold_target));
        /* If the previous grasp reached a driver end-stop, give the vendor
         * fault-reset pulse once, then continue with the explicit OPEN
         * target. This prevents a stale software latch from making OPEN
         * appear unresponsive. */
        if (hand->contact_mode == CONTACT_MODE_QUEST) {
          hand->fault_latched = 0;
          hand->fault_reset_cycles = 10;
        }
      }
      apply_coupled_soft_limit(hand);
      hand->command_active = 1;
    }
    return;
  }

  if (sscanf(line, "FLEX %c %d", &side, &value) == 2) {
    hand_t *hand = find_hand(hands, side);
    if (hand) {
      for (int index = 0; index < 5; ++index)
        hand->requested[index] = (int16_t)clamp_target(index, value);
      apply_coupled_soft_limit(hand);
      hand->command_active = 1;
    }
    return;
  }

  if (sscanf(line, "BOTH %d", &value) == 1) {
    for (int hand_index = 0; hand_index < 2; ++hand_index) {
      if (!hands[hand_index].enabled) continue;
      for (int channel_index = 0; channel_index < 5; ++channel_index)
        hands[hand_index].requested[channel_index] =
            (int16_t)clamp_target(channel_index, value);
      apply_coupled_soft_limit(&hands[hand_index]);
      hands[hand_index].command_active = 1;
    }
    return;
  }

  if (sscanf(line, "STOP %c", &side) == 1) {
    hand_t *hand = find_hand(hands, side);
    if (hand) hand->command_active = 0;
    return;
  }
  if (strncmp(line, "QUIT", 4) == 0) stop_requested = 1;
}

static int parse_int(const char *text, int *result) {
  char *end = NULL;
  errno = 0;
  const long value = strtol(text, &end, 10);
  if (errno || !end || *end != '\0' || value < 0 || value > 255) return 0;
  *result = (int)value;
  return 1;
}

static int parse_speed(const char *text, int *result) {
  char *end = NULL;
  errno = 0;
  const long value = strtol(text, &end, 10);
  if (errno || !end || *end != '\0' || value < 1 || value > 2000) return 0;
  *result = (int)value;
  return 1;
}

static int parse_hold_delay(const char *text, int *result) {
  char *end = NULL;
  errno = 0;
  const long value = strtol(text, &end, 10);
  if (errno || !end || *end != '\0' || value < 20 || value > 60000) return 0;
  *result = (int)value;
  return 1;
}

static void shutdown_bus(hand_t hands[2]) {
  for (int index = 0; index < 2; ++index) {
    if (!hands[index].enabled || !ec_slave[hands[index].slave].outputs) continue;
    set_output(&hands[index], hands[index].target, 0U);
    ec_dcsync0((uint16_t)hands[index].slave, FALSE, DC_CYCLE_NS, 0);
  }
  ec_send_processdata();
  (void)ec_receive_processdata(EC_TIMEOUTRET);
  ec_slave[0].state = EC_STATE_SAFE_OP;
  ec_writestate(0);
  (void)ec_statecheck(0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE);
  ec_slave[0].state = EC_STATE_PRE_OP;
  ec_writestate(0);
  (void)ec_statecheck(0, EC_STATE_PRE_OP, EC_TIMEOUTSTATE);
}

static int dry_run(void) {
  puts("{\"format\":\"ih01_dual_hand_control_plan_v1\","
       "\"hardware_access\":false,\"cycle_us\":2000,"
       "\"sync0_shift_ns\":700,\"mode\":2,\"speed_steps_s\":200,"
       "\"control_sequence\":[0,1,0,0,2,0,4],"
       "\"channel_limits\":[1700,1700,1700,1700,1700,1300],"
       "\"clear_fault_sequence\":[5,0],\"contact_hold_delay_ms\":200,"
       "\"contact_current_trip_ma\":1000,\"contact_current_pretrip_ma\":600,"
       "\"contact_force_raw_trip\":5000,\"contact_stall_delta_steps\":10,"
       "\"current_is_monitor_only\":false}");
  return 0;
}

static int soft_limit_check(int argc, char **argv) {
  if (argc != 5) return 64;
  char *end = NULL;
  const long index_flex = strtol(argv[2], &end, 10);
  if (!end || *end != '\0' || index_flex < 0 || index_flex > 1700) return 64;
  const long thumb_flex = strtol(argv[3], &end, 10);
  if (!end || *end != '\0' || thumb_flex < 0 || thumb_flex > 1700) return 64;
  const long thumb_rotate = strtol(argv[4], &end, 10);
  if (!end || *end != '\0' || thumb_rotate < 0 || thumb_rotate > 1300) return 64;
  const int maximum = thumb_flex_max((int)index_flex, (int)thumb_rotate);
  const int output = thumb_flex > maximum ? maximum : (int)thumb_flex;
  printf("{\"ch4\":%ld,\"ch5_requested\":%ld,\"ch6\":%ld,"
         "\"ch5_max\":%d,\"ch5_output\":%d,\"limited\":%s}\n",
         index_flex, thumb_flex, thumb_rotate, maximum, output,
         thumb_flex > maximum ? "true" : "false");
  return 0;
}

int main(int argc, char **argv) {
  if (argc == 2 && strcmp(argv[1], "--dry-run") == 0) return dry_run();
  if (argc > 1 &&
      (strcmp(argv[1], "--soft-limit-check") == 0 ||
       strcmp(argv[1], "--left-soft-limit-check") == 0))
    return soft_limit_check(argc, argv);
  if (argc < 7 || strcmp(argv[2], "--left-slave") != 0 ||
      strcmp(argv[4], "--right-slave") != 0)
    return fprintf(stderr,
                   "Usage: sudo %s <interface> --left-slave N "
                   "--right-slave N --interactive [--speed-steps-s N] "
                   "[--contact-hold-delay-ms N] "
                   "[--contact-hold-mode visual|quest] "
                   "[--enable-thumb-index-soft-limit] [--fault-reset]\n",
                   argv[0]),
           64;
  if (strcmp(argv[6], "--interactive") != 0) return 64;

  /* poll() observes the file descriptor, while fgets() normally reads ahead.
   * Disable stdin buffering so consecutive GUI commands cannot remain hidden
   * inside stdio after poll() reports the descriptor drained. */
  setvbuf(stdin, NULL, _IONBF, 0);

  int configured_speed = 200;
  int contact_hold_delay_ms = CONTACT_STALL_CYCLES * CYCLE_US / 1000;
  int enable_soft_limit = 0;
  int allow_fault_reset = 0;
  int contact_mode = CONTACT_MODE_VISUAL;
  for (int index = 7; index < argc;) {
    if (strcmp(argv[index], "--speed-steps-s") == 0 && index + 1 < argc) {
      if (!parse_speed(argv[index + 1], &configured_speed))
        return fprintf(stderr, "Invalid speed; expected 1..2000 steps/s\n"), 64;
      index += 2;
    } else if (strcmp(argv[index], "--contact-hold-delay-ms") == 0 &&
               index + 1 < argc) {
      if (!parse_hold_delay(argv[index + 1], &contact_hold_delay_ms))
        return fprintf(stderr, "Invalid contact hold delay; expected 20..60000 ms\n"), 64;
      index += 2;
    } else if (strcmp(argv[index], "--enable-thumb-index-soft-limit") == 0 ||
               strcmp(argv[index],
                      "--enable-left-thumb-index-soft-limit") == 0) {
      enable_soft_limit = 1;
      ++index;
    } else if (strcmp(argv[index], "--fault-reset") == 0) {
      allow_fault_reset = 1;
      ++index;
    } else if (strcmp(argv[index], "--contact-hold-mode") == 0 &&
               index + 1 < argc) {
      if (strcmp(argv[index + 1], "visual") == 0)
        contact_mode = CONTACT_MODE_VISUAL;
      else if (strcmp(argv[index + 1], "quest") == 0)
        contact_mode = CONTACT_MODE_QUEST;
      else
        return fprintf(stderr, "Invalid contact hold mode; expected visual or quest\n"), 64;
      index += 2;
    } else {
      return fprintf(stderr, "Unknown or incomplete backend option: %s\n",
                     argv[index]),
             64;
    }
  }

  int left_slave = 0;
  int right_slave = 0;
  if (!parse_int(argv[3], &left_slave) || !parse_int(argv[5], &right_slave) ||
      (left_slave == 0 && right_slave == 0) ||
      (left_slave != 0 && left_slave == right_slave))
    return fprintf(stderr, "Invalid left/right slave assignment\n"), 64;

  hand_t hands[2] = {
      {.side = "left",
       .slave = left_slave,
       .enabled = left_slave > 0,
       .soft_limit_enabled = left_slave > 0 && enable_soft_limit,
       .soft_limit_ch5_max = 1700,
       .contact_mode = contact_mode,
       .contact_stall_limit = (uint16_t)((contact_hold_delay_ms * 1000 + CYCLE_US - 1) / CYCLE_US)},
      {.side = "right",
       .slave = right_slave,
       .enabled = right_slave > 0,
       .soft_limit_enabled = right_slave > 0 && enable_soft_limit,
       .soft_limit_ch5_max = 1700,
       .contact_mode = contact_mode,
       .contact_stall_limit = (uint16_t)((contact_hold_delay_ms * 1000 + CYCLE_US - 1) / CYCLE_US)},
  };
  for (int index = 0; index < 2; ++index)
    for (int channel = 0; channel < 6; ++channel)
      hands[index].speed[channel] = (int16_t)configured_speed;
  signal(SIGINT, request_stop);
  signal(SIGTERM, request_stop);
  signal(SIGHUP, request_stop);

  if (!ec_init(argv[1]))
    return fprintf(stderr, "Cannot open EtherCAT socket on %s\n", argv[1]), 66;
  int configuration_attempt = 0;
configure_slaves:
  ;
  const int slave_count = ec_config_init(FALSE);
  const int highest = left_slave > right_slave ? left_slave : right_slave;
  if (slave_count < highest) {
    ec_close();
    return fprintf(stderr, "Found %d slaves, assignment requires slave %d\n",
                   slave_count, highest),
           67;
  }
  for (int index = 0; index < 2; ++index) {
    if (!hands[index].enabled) continue;
    const int slave = hands[index].slave;
    if (ec_slave[slave].eep_man != VENDOR_ID ||
        ec_slave[slave].eep_id != PRODUCT_CODE ||
        strcmp(ec_slave[slave].name, "IPE-IO-Device") != 0) {
      ec_close();
      return fprintf(stderr, "%s slave %d identity mismatch\n",
                     hands[index].side, slave),
             68;
    }
  }

  memset(io_map, 0, sizeof(io_map));
  const int mapped_bytes = ec_config_map(io_map);
  if (mapped_bytes <= 0) {
    ec_close();
    return fprintf(stderr, "PDO mapping failed\n"), 69;
  }
  int pdo_layout_ok = 1;
  for (int index = 0; index < 2; ++index) {
    if (!hands[index].enabled) continue;
    const int slave = hands[index].slave;
    if (ec_slave[slave].Obytes != OUTPUT_BYTES ||
        ec_slave[slave].Ibytes != INPUT_BYTES || !ec_slave[slave].outputs ||
        !ec_slave[slave].inputs)
      pdo_layout_ok = 0;
  }
  if (!pdo_layout_ok && configuration_attempt == 0) {
    fprintf(stderr,
            "PDO layout not settled after previous master; retrying from INIT\n");
    ec_slave[0].state = EC_STATE_INIT;
    ec_writestate(0);
    (void)ec_statecheck(0, EC_STATE_INIT, EC_TIMEOUTSTATE);
    ec_close();
    osal_usleep(250000U);
    if (!ec_init(argv[1]))
      return fprintf(stderr, "Cannot reopen EtherCAT socket on %s\n", argv[1]),
             66;
    ++configuration_attempt;
    memset(io_map, 0, sizeof(io_map));
    goto configure_slaves;
  }
  if (!pdo_layout_ok) {
    for (int index = 0; index < 2; ++index) {
      if (!hands[index].enabled) continue;
      const int slave = hands[index].slave;
      fprintf(stderr,
              "%s slave %d PDO size mismatch: O=%" PRIu32
              " bytes/%u bits I=%" PRIu32
              " bytes/%u bits pointers=%s/%s mapped=%d\n",
              hands[index].side, slave, ec_slave[slave].Obytes,
              ec_slave[slave].Obits, ec_slave[slave].Ibytes,
              ec_slave[slave].Ibits,
              ec_slave[slave].outputs ? "yes" : "no",
              ec_slave[slave].inputs ? "yes" : "no", mapped_bytes);
    }
    ec_close();
    return 70;
  }

  if (!ec_configdc()) {
    ec_close();
    return fprintf(stderr, "DC configuration failed\n"), 70;
  }
  osal_usleep(80000U);
  for (int index = 0; index < 2; ++index) {
    if (!hands[index].enabled) continue;
    if (!ec_slave[hands[index].slave].hasdc) {
      ec_close();
      return fprintf(stderr, "%s slave has no DC\n", hands[index].side), 70;
    }
    ec_dcsync0((uint16_t)hands[index].slave, TRUE, DC_CYCLE_NS, DC_SHIFT_NS);
  }

  const int16_t zeros[6] = {0};
  apply_frame(hands, zeros, 0U);
  (void)ec_statecheck(0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE * 4);
  ec_readstate();
  for (int index = 0; index < 2; ++index) {
    if (hands[index].enabled &&
        (ec_slave[hands[index].slave].state != EC_STATE_SAFE_OP ||
         ec_slave[hands[index].slave].ALstatuscode != 0U)) {
      shutdown_bus(hands);
      ec_close();
      return fprintf(stderr, "%s SAFE-OP gate failed\n", hands[index].side), 71;
    }
  }

  const int expected_wkc =
      (ec_group[0].outputsWKC * 2) + ec_group[0].inputsWKC;
  ec_send_processdata();
  (void)ec_receive_processdata(EC_TIMEOUTRET);
  ec_slave[0].state = EC_STATE_OPERATIONAL;
  ec_writestate(0);
  for (int attempt = 0; attempt < 50 && !stop_requested; ++attempt) {
    apply_frame(hands, zeros, 0U);
    ec_send_processdata();
    (void)ec_receive_processdata(EC_TIMEOUTRET);
    (void)ec_statecheck(0, EC_STATE_OPERATIONAL, 1000);
    if (ec_slave[0].state == EC_STATE_OPERATIONAL) break;
    osal_usleep(CYCLE_US);
  }
  ec_readstate();
  if (ec_slave[0].state != EC_STATE_OPERATIONAL) {
    shutdown_bus(hands);
    ec_close();
    return fprintf(stderr, "OP transition failed\n"), 72;
  }

  /* Learn current non-negative positions while sending no command. */
  for (int cycle = 0; cycle < STARTUP_CYCLES && !stop_requested; ++cycle) {
    apply_frame(hands, zeros, 0U);
    const int wkc = exchange(hands, expected_wkc);
    if (wkc < expected_wkc ||
        (hard_fault_present(hands) && !allow_fault_reset)) {
      print_startup_feedback(hands, wkc, expected_wkc);
      shutdown_bus(hands);
      ec_close();
      return 73;
    }
    for (int hand_index = 0; hand_index < 2; ++hand_index) {
      if (!hands[hand_index].enabled) continue;
      for (int channel = 0; channel < 6; ++channel) {
        const int value = hands[hand_index].feedback.position[channel];
        if (value > hands[hand_index].requested[channel])
          hands[hand_index].requested[channel] =
              (int16_t)clamp_target(channel, value);
      }
      apply_coupled_soft_limit(&hands[hand_index]);
    }
    osal_usleep(CYCLE_US);
  }

  int16_t mode[6] = {2, 2, 2, 2, 2, 2};
  int startup_ok = allow_fault_reset ||
                   (hold_shared(hands, mode, 0U, STAGE_CYCLES, expected_wkc) &&
                   hold_shared(hands, mode, MODE_COMMAND, STAGE_CYCLES,
                               expected_wkc) &&
                   hold_shared(hands, mode, 0U, STAGE_CYCLES, expected_wkc) &&
                   hold_shared(hands, hands[0].speed, 0U, STAGE_CYCLES, expected_wkc) &&
                   hold_shared(hands, hands[0].speed, SPEED_COMMAND, STAGE_CYCLES,
                               expected_wkc) &&
                   hold_shared(hands, hands[0].speed, 0U, STAGE_CYCLES, expected_wkc));
  if (!startup_ok) {
    shutdown_bus(hands);
    ec_close();
    return fprintf(stderr, "Mode/speed setup failed\n"), 74;
  }

  fprintf(stderr,
          "IH01 controller ready: interface=%s left=%d right=%d, "
          "2 ms PDO speed=%d steps/s\n",
          argv[1], left_slave, right_slave, configured_speed);
  uint64_t cycle = 0;
  int bad_wkc = 0;
  while (!stop_requested) {
    struct pollfd input = {.fd = STDIN_FILENO, .events = POLLIN};
    if (poll(&input, 1, 0) > 0) {
      if (input.revents & POLLIN) {
        char line[256] = {0};
        if (fgets(line, sizeof(line), stdin))
          handle_command(hands, line);
        else
          stop_requested = 1;
      }
      if (input.revents & (POLLERR | POLLHUP | POLLNVAL))
        stop_requested = 1;
    }

    for (int index = 0; index < 2; ++index) {
      if (!hands[index].enabled) continue;
      update_contact_guard(&hands[index]);
      apply_coupled_soft_limit(&hands[index]);
      apply_contact_guard(&hands[index]);
      if (hands[index].fault_reset_cycles > 0) {
        const int16_t reset_values[6] = {0};
        set_output(&hands[index], reset_values, 5U);
      } else if (hands[index].fault_latched) {
        const int16_t reset_values[6] = {0};
        set_output(&hands[index], reset_values, 0U);
      } else if (hands[index].speed_update_cycles > 0)
        set_output(&hands[index], hands[index].speed, SPEED_COMMAND);
      else
        set_output(&hands[index], hands[index].target,
                   hands[index].command_active ? POSITION_COMMAND : 0U);
    }
    const int wkc = exchange(hands, expected_wkc);
    if (wkc < expected_wkc)
      ++bad_wkc;
    else
      bad_wkc = 0;
    if (hard_fault_present(hands)) {
      int newly_latched = 0;
      for (int index = 0; index < 2; ++index)
        if (hands[index].enabled) {
          if (!hands[index].fault_latched) newly_latched = 1;
          hands[index].fault_latched = 1;
          hands[index].command_active = 0;
        }
      if (newly_latched)
        fprintf(stderr, "Fault reported; outputs held at CW0. Press RESET only after removing the cause.\n");
    }
    if (bad_wkc >= MAX_BAD_WKC) {
      fprintf(stderr, "Repeated WKC failure; stopping\n");
      break;
    }
    for (int index = 0; index < 2; ++index)
      if (hands[index].speed_update_cycles > 0) --hands[index].speed_update_cycles;
    for (int index = 0; index < 2; ++index)
      if (hands[index].fault_reset_cycles > 0) --hands[index].fault_reset_cycles;
    if (cycle % FEEDBACK_DIVIDER == 0)
      print_feedback(hands, cycle, wkc, expected_wkc);
    ++cycle;
    osal_usleep(CYCLE_US);
  }

  shutdown_bus(hands);
  ec_close();
  return 0;
}
