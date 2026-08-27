#ifndef TESLA_BATTERY_H
#define TESLA_BATTERY_H
#include "../datalayer/datalayer.h"
#include "TESLA-HTML.h"
#include "UdsCanBattery.h"

// HVC backup
// Reads the HVBMS (0x602/0x612) and HVP (0x610/0x611) fields.
// Multi-byte fields are big-endian.
static constexpr uint8_t TESLA_HVC_MAX_MODULES = 8;
static constexpr uint8_t TESLA_HVC_NUM_HISTOGRAMS = 8;  // HISTOGRAM_ID 1..8
static constexpr uint8_t TESLA_HVP_NUM_CAL = 8;         // HVP calibration records 0..7
static constexpr uint8_t TESLA_HVBMS_NUM_CAL = 3;       // HVBMS calibration records 0, 1, 6

// Raw responses collected from the ECUs. The present flags let the JSON emit "MISSING" for
// anything that NAKed or timed out, like ODIN does.
struct TeslaHvcBackup {
  uint8_t energy_counters[32] = {0};  // ENERGY_COUNTERS 0x420
  bool energy_counters_present = false;
  // Actual response length. The ODX declares 32, but 2022 packs send 28 with the Ah pair as two
  // uint16 - the length is what tells the two layouts apart.
  uint8_t energy_counters_len = 0;
  uint8_t pack_info[16] = {0};  // PACK_INFO 0x450
  bool pack_info_present = false;
  uint8_t hist_page_count[8] = {0};  // HISTOGRAM_PAGE_COUNT 0x436, for the Pack Info block
  bool hist_page_count_present = false;

  uint8_t assembly_id_hvbms = 0;  // 0xF01C, one byte
  uint16_t usage_id_hvbms = 0;    // 0xF01D, big-endian uint16 (a one-byte read returns the high half)
  bool assembly_id_hvbms_present = false, usage_id_hvbms_present = false;
  uint8_t assembly_id_hvp = 0;
  uint16_t usage_id_hvp = 0;
  bool assembly_id_hvp_present = false, usage_id_hvp_present = false;

  char package_pn[24] = {0};  // 0xF014 (ASCII)
  char package_sn[24] = {0};  // 0xF015
  char board_pn[24] = {0};    // 0xF012
  char board_sn[24] = {0};    // 0xF013
  bool package_pn_present = false, package_sn_present = false, board_pn_present = false, board_sn_present = false;

  uint8_t config_hvbms[64] = {0};  // CONFIGURATION_RECORD 0x439 (HVBMS)
  uint8_t config_hvbms_len = 0;
  uint8_t config_hvp[64] = {0};  // CONFIGURATION_RECORD 0x439 (HVP)
  uint8_t config_hvp_len = 0;

  // HVP calibration records 0..7, each a uint32. ODIN reads these via routine 0x438
  // CALIBRATION_RECORD getResults, so that is the primary path; from_did marks values that came
  // from the 0x0500+id DID fallback instead.
  uint32_t hvp_cal[TESLA_HVP_NUM_CAL] = {0};
  bool hvp_cal_present[TESLA_HVP_NUM_CAL] = {false};
  bool hvp_cal_from_did[TESLA_HVP_NUM_CAL] = {false};

  // HVBMS calibration records, ids 0/1/6 per ODIN's required-for-recovery set, via routine 0x439.
  uint32_t hvbms_cal[TESLA_HVBMS_NUM_CAL] = {0};
  bool hvbms_cal_present[TESLA_HVBMS_NUM_CAL] = {false};

  uint8_t num_bricks = 0, num_modules = 0;  // from PACK_INFO

  char readable_time[24] = {0};  // "YYYY-MM-DD HH:MM:SS" or "Not Available"

  // Read diagnostics for debugging...
  static constexpr uint8_t MAX_NRC_RECORDS = 10;
  uint16_t nrc_did[MAX_NRC_RECORDS] = {0};
  uint8_t nrc_code[MAX_NRC_RECORDS] = {0};
  uint8_t nrc_records = 0;
  uint16_t nrc_count = 0;       // total refusals, including repeats
  uint16_t short_read_did = 0;  // first response rejected for being shorter than its ODX layout
  uint8_t short_read_len = 0;
  uint16_t short_read_count = 0;

  void record_nrc(uint16_t did, uint8_t nrc) {
    ++nrc_count;
    for (uint8_t i = 0; i < nrc_records; i++) {
      if (nrc_did[i] == did && nrc_code[i] == nrc) {
        return;
      }
    }
    if (nrc_records < MAX_NRC_RECORDS) {
      nrc_did[nrc_records] = did;
      nrc_code[nrc_records] = nrc;
      ++nrc_records;
    }
  }
};

// Render the collected backup as ODIN-format JSON.
String build_hvc_backup_json(const TeslaHvcBackup& b);

// 0x7FF gateway config, "Gen3" vehicles only, not applicable to Gen2 "classic" Model S and Model X
// These are user configurable from the Webserver UI
extern bool user_selected_tesla_digital_HVIL;
extern uint16_t user_selected_tesla_GTW_packEnergy;
extern uint16_t user_selected_tesla_soc_source;

// GTW_carConfig mux3 GTW_packEnergy, VAL_ 2047 GTW_packEnergy 0 "SR" 1 "LR" 2 "MR"
static constexpr uint16_t TESLA_GTW_PACK_ENERGY_LR = 1;

class TeslaBattery : public UdsCanBattery {
 public:
  bool mandatory_charge_taper() { return true; }
  // Use the default constructor to create the first or single battery.
  TeslaBattery() {
    datalayer_battery = &datalayer.battery;
    allows_contactor_closing = &datalayer.system.status.battery_allows_contactor_closing;
    previous_max_percentage = datalayer.battery.settings.max_percentage;
  }
  // Use this constructor for the second or third battery.
  TeslaBattery(DATALAYER_BATTERY_TYPE* datalayer_ptr, CAN_Interface targetCan) : UdsCanBattery(targetCan) {
    datalayer_battery = datalayer_ptr;
    allows_contactor_closing = nullptr;
    previous_max_percentage = datalayer_ptr->settings.max_percentage;
  }
  virtual void setup();
  virtual void handle_incoming_can_frame(CAN_frame rx_frame);
  virtual void update_values();
  void set_balancing_status(balancing_status_enum new_status);

  /* Observed BMS_balanceState values. The names are ours, not Tesla's,
   * and both values are currently guessed from logged behaviour */
  static constexpr uint8_t BALANCE_STATE_IDLE = 0;
  static constexpr uint8_t BALANCE_STATE_BALANCING = 1;
  static constexpr uint8_t BALANCE_STATE_VSH_TEST = 12;

  //BMS_state (0x212) enum: SUPPORT is the accessory/standalone state that accepts the isolation-monitor disable.
  static constexpr uint8_t BMS_STATE_SUPPORT = 2;

  enum TeslaUdsState : uint16_t {
    TESLA_UDS_BMS_RESET = 1,
    TESLA_UDS_BMS_RESET_ECU,
    TESLA_UDS_BMS_RESET_DONE,
    TESLA_UDS_SOC_RESET,
    TESLA_UDS_SOC_RESET_SESSION,
    TESLA_UDS_SOC_RESET_KEY,
    TESLA_UDS_SOC_RESET_ROUTINE,
    TESLA_UDS_SOC_RESET_RESULT,
    TESLA_UDS_SOC_RESET_ECU,
    TESLA_UDS_SOC_RESET_DONE,
    TESLA_UDS_ISO_CLEAR,
    TESLA_UDS_ISO_CLEAR_SESSION,
    TESLA_UDS_ISO_CLEAR_KEY,
    TESLA_UDS_ISO_CLEAR_COUNTERS,
    TESLA_UDS_ISO_CLEAR_FLOOD,
    TESLA_UDS_ISO_CLEAR_INTERNAL,
    TESLA_UDS_ISO_CLEAR_DONE,
    //ODIN "Blocked Contactors - Attempt to Clear" (Gen3): VSH latch reset, then the isolation-condition
    //clear routines, then reset HVBMS, then reset HVP. All HVBMS steps run in one session on 0x602;
    //the HVP ECUReset is dispatched at the end via pending_ecu_reset_req_id.
    TESLA_UDS_CBC,           // entry: open extended session
    TESLA_UDS_CBC_SESSION,   // session ack -> request security seed
    TESLA_UDS_CBC_KEY,       // seed -> send key, then start VSH latch reset routine
    TESLA_UDS_CBC_VSH,       // VSH ack -> start isolation-counter reset
    TESLA_UDS_CBC_COUNTERS,  // -> clear battery coolant flood alert
    TESLA_UDS_CBC_FLOOD,     // -> clear internal isolation alert
    TESLA_UDS_CBC_INTERNAL,  // -> ECUReset the HVBMS
    TESLA_UDS_CBC_HVBMS,     // HVBMS reset ack -> queue the HVP reset, done
    TESLA_UDS_CBC_DONE,
    TESLA_UDS_QUERY,
    TESLA_UDS_QUERY_READ,
    TESLA_UDS_QUERY_PARSE,
    TESLA_UDS_BLEED,
    TESLA_UDS_BLEED_START,
    TESLA_UDS_BLEED_COUNT,
    TESLA_UDS_BLEED_RESULT,
    TESLA_UDS_CAC_RESET,
    TESLA_UDS_CAC_RESET_ROUTINE,
    TESLA_UDS_CAC_RESET_RESULT,
    TESLA_UDS_BRICKV,
    TESLA_UDS_BRICKV_READ,
    TESLA_UDS_BRICKV_PARSE,
    TESLA_UDS_ISO_MON,
    TESLA_UDS_ISO_MON_SESSION,
    TESLA_UDS_ISO_MON_KEY,
    TESLA_UDS_ISO_MON_ROUTINE,
    TESLA_UDS_ISO_MON_RESULT,
    // ECU version fingerprint: one sequence walking BMS -> HVP -> PCS, retargeting the UDS channel
    // between ECUs. A step that times out just moves the chain on (an ECU can be absent).
    TESLA_UDS_VERSION,          // entry: open extended session on the BMS
    TESLA_UDS_VERSION_SESSION,  // session ack -> request 0xF180
    TESLA_UDS_VERSION_F180,     // BMS identity block
    TESLA_UDS_VERSION_0F00,     // BMS app CRC, then retarget to the HVP
    TESLA_UDS_VERSION_HVP_F180,
    TESLA_UDS_VERSION_HVP_0F00,  // then retarget to the PCS
    TESLA_UDS_VERSION_PCS_F180,
    TESLA_UDS_VERSION_PCS_0F00,
    TESLA_UDS_VERSION_PCS_0F02,  // PCS CPU2 CRC
    TESLA_UDS_VERSION_PCS_F012,  // PCS board part number, then done
    // Pack identity straight off the HVBMS instead of waiting for the 0x72A broadcast. All DIDs are
    // security_level 0, so an extended session with no SecurityAccess is enough.
    // Ends with the PCS board part number; the HVP is not asked - it shares the
    // HVC board with the HVBMS, so its 0xF012 repeats what the HVBMS already gave.
    TESLA_UDS_IDENT,           // entry: open extended session
    TESLA_UDS_IDENT_SESSION,   // session ack -> request 0xF015 PACKAGE_SERIAL_NUMBER
    TESLA_UDS_IDENT_SERIAL,    // recv 0xF015 -> request 0x0450 PACK_INFO
    TESLA_UDS_IDENT_PACKINFO,  // recv 0x0450 -> request 0xF014 PACKAGE_PART_NUMBER
    TESLA_UDS_IDENT_PACK_PN,   // recv 0xF014 -> request 0xF012 BOARD_PART_NUMBER (the HVBMS board)
    TESLA_UDS_IDENT_BOARD_PN,  // recv 0xF012 -> retarget to the PCS for its board part number
    TESLA_UDS_IDENT_PCS_PN,    // recv PCS 0xF012 -> done
    // HVC backup: HVBMS scalar reads, HVBMS calibration records (routine 0x439), then HVP
    // (retargeted) with HVP calibration via routine 0x438 first and the DID as fallback.
    // hvc_bk_step indexes the current phase's table.
    TESLA_UDS_HVC,            // entry
    TESLA_UDS_HVC_HVBMS,      // response to HVC_SCALARS[hvc_bk_step]
    TESLA_UDS_HVC_HVBMS_CAL,  // response to HVBMS calibration record HVBMS_CAL_IDS[hvc_bk_step]
    TESLA_UDS_HVC_HVP,        // response to HVC_SCALARS_HVP[hvc_bk_step] as a DID read
    TESLA_UDS_HVC_CAL_RT,     // response to routine 0x438 for the current HVP calibration record
  };
  static constexpr uint16_t TESLA_MAX_BRICKS = 112;
  static constexpr unsigned long TESLA_BLEED_POLL_INTERVAL_MS = 30000;

  // HVBMS RoutineControl identifiers (ODJ names).
  static constexpr uint16_t RID_RESET_SOC = 0x0407;
  static constexpr uint16_t RID_RESET_CAC = 0x040C;
  static constexpr uint16_t RID_DISABLE_ISOLATION_MONITOR = 0x040E;
  static constexpr uint16_t RID_RESET_ISOLATION_COUNTERS = 0x0436;
  static constexpr uint16_t RID_CLEAR_BATTERY_COOLANT_FLOOD_ALERT = 0x0445;
  static constexpr uint16_t RID_CLEAR_INTERNAL_ISOLATION_ALERT = 0x0444;
  static constexpr uint16_t RID_RESET_VSH_FAULT = 0x041C;  //clears BMS_a119_Fast_Open_Vsh_Detected latch
  // SecurityAccess request-seed sub-functions; sendKey is seed sub-function + 1.
  static constexpr uint8_t SECURITY_LEVEL_5 = 0x05;
  static constexpr uint8_t SECURITY_LEVEL_19 = 0x13;
  virtual void transmit_can(unsigned long currentMillis);

  bool supports_clear_isolation() { return true; }
  bool supports_insulation_resistance() { return true; }
  void clear_isolation() { datalayer_battery->settings.user_requests_tesla_isolation_clear = true; }

  // The reset commands below require the pack contactors to be OPEN, which the user does manually
  // before running them - the command is refused if the BMS still reports the contactors closed.
  const char* contactor_open_warning() {
    return "This requires the pack contactors to be OPEN. Open them first (switch the car to a mode "
           "that drops the contactors); the command is refused if they are still closed. Continue?";
  }

  bool supports_reset_BMS() { return true; }
  void reset_BMS() { datalayer_battery->settings.user_requests_tesla_bms_reset = true; }

  bool supports_reset_HVP() { return true; }
  void reset_HVP() { datalayer_battery->settings.user_requests_tesla_hvp_reset = true; }

  bool supports_clear_blocked_contactors() { return true; }
  void clear_blocked_contactors() { datalayer_battery->settings.user_requests_tesla_clear_blocked_contactors = true; }

  bool supports_reset_PCS() { return true; }
  void reset_PCS() { datalayer_battery->settings.user_requests_tesla_pcs_reset = true; }

  // No "Read DTC" / "Erase DTC" buttons for Tesla: the pack ECUs broadcast their faults via the
  // alert matrix CAN IDs (0x320/0x3A4/0x31E) rather than serving UDS 0x19, and all three answer
  // 0x14 ClearDiagnosticInformation with NRC 0x11 (alert/fault display is not affected).
  bool supports_read_DTC() override { return false; }
  bool supports_reset_DTC() override { return false; }

  bool supports_hvc_backup() { return true; }
  void request_hvc_backup_partial() { datalayer_battery->settings.user_requests_tesla_hvc_backup_partial = true; }

  bool supports_reset_SOC() { return true; }
  void reset_SOC() { datalayer_battery->settings.user_requests_tesla_soc_reset = true; }

  bool supports_reset_CAC() { return true; }
  void reset_CAC() { datalayer_battery->settings.user_requests_tesla_cac_reset = true; }

  bool supports_disable_isolation_monitor() { return true; }
  void disable_isolation_monitor() { datalayer_battery->settings.user_requests_tesla_disable_isolation = true; }
  void enable_isolation_monitor() { datalayer_battery->settings.user_requests_tesla_enable_isolation = true; }

  bool supports_query_ecu_versions() { return true; }
  void query_ecu_versions() { ecu_version_query_requested = true; }

  bool supports_set_vehicle_mode() { return true; }
  void set_vehicle_mode_drive() { datalayer_battery->settings.user_requests_tesla_drive_mode = true; }
  void set_vehicle_mode_accessory() { datalayer_battery->settings.user_requests_tesla_accessory_mode = true; }
  void toggle_balance_at_rest() { datalayer_battery->settings.user_requests_tesla_toggle_balance_at_rest = true; }

 protected:
  void on_uds_sequence_step(uint16_t state, uint8_t sid, const uint8_t* data, uint16_t len) override;
  void on_uds_sequence_timeout(uint16_t state) override;

 private:
  void start_routine(uint16_t next_state, uint16_t routine_id);
  void request_bleed_results(uint16_t start_brick);
  void finalize_bleed_read();
  uint32_t brick_bleed_time_s[TESLA_MAX_BRICKS] = {0};
  uint32_t brick_bleed_prev_s[TESLA_MAX_BRICKS] = {0};
  uint16_t brick_bleed_pack_count = 0;
  bool brick_bleed_has_prev = false;
  unsigned long brick_bleed_last_poll = 0;
  bool isolation_monitor_disable_latched = false;
  bool isolation_monitor_disabling = false;
  bool iso_mon_needs_assert = false;
  bool battery_was_alive = false;
  bool brickv_requested = false;  // one-shot fast cell populate via UDS DID 0x410 at startup
  uint8_t brickv_attempts = 0;
  static constexpr uint8_t TESLA_BRICKV_MAX_ATTEMPTS = 3;
  static constexpr uint16_t TESLA_BRICKV_COUNT = 108;

  // ---- ECU firmware-version fingerprint: DID 0xF180 identity + 0x0F00/0x0F02 app CRC on
  // HVBMS/HVP/PCS, read as one framework sequence. The framework talks to one ECU at a time, so
  // the sequence retargets it (retarget_uds) between the three.
  static constexpr uint16_t BMS_UDS_REQUEST = 0x602;
  static constexpr uint16_t BMS_UDS_RESPONSE = 0x612;
  static constexpr uint16_t HVP_UDS_REQUEST = 0x610;
  static constexpr uint16_t HVP_UDS_RESPONSE = 0x611;
  static constexpr uint16_t PCS_UDS_REQUEST = 0x628;
  static constexpr uint16_t PCS_UDS_RESPONSE = 0x629;
  static constexpr uint16_t DID_BOOT_SW_ID = 0xF180;  // 19-byte identity block (hwid + boot CRC)
  static constexpr uint16_t DID_APP_CRC = 0x0F00;     // 4-byte running application CRC (CPU1)
  static constexpr uint16_t DID_APP_CRC_CPU2 = 0x0F02;
  static constexpr uint16_t DID_BOARD_PART_NUMBER = 0xF012;  // 20-byte ASCII, ODX security_level 0
  struct EcuVersion {
    uint8_t f180[19] = {0};
    uint8_t f180_len = 0;  // 0 = not read / no response
    uint8_t crc0f00[4] = {0};
    bool crc0f00_valid = false;
    uint8_t crc0f02[4] = {0};  // PCS CPU2 only
    bool crc0f02_valid = false;
  };
  EcuVersion ecu_ver_bms;
  EcuVersion ecu_ver_hvp;
  EcuVersion ecu_ver_pcs;
  bool ecu_version_query_requested = false;
  // Point the shared UDS channel at another ECU. Only between sequence steps, and always put back
  // to the BMS when the sequence ends, whichever way it ends.
  void retarget_uds(uint16_t req_id, uint16_t resp_id) { setup_uds(req_id, resp_id); }
  void start_ecu_version_query();
  void start_identity_query();
  // Pack build date from the year + Julian day embedded in the serial. Called from both places a
  // serial can arrive (0x72A broadcast, UDS 0xF015) - whichever lands first locks the other out.
  void derive_manufacture_date_from_serial();
  // Store a 0x62 response into the right EcuVersion field, send the chain's next read, and stamp
  // the fingerprint at the end. The timeout path calls ecu_version_continue directly, so an absent
  // ECU just moves the chain on.
  void ecu_version_store_did(EcuVersion& ev, const uint8_t* data, uint16_t len);
  void ecu_version_continue(uint16_t completed_state);
  bool ecu_ver_send(uint16_t next_state, uint16_t did);
  void finish_ecu_version_query();
  //User-requested UDS ECUReset (0x11 0x01) to the HVP (0x610) or PCS (0x628). Queued in
  //update_values, sent from transmit_can as a raw single frame - the positive response is
  //suppressed (0x81), so there is nothing to wait for.
  uint16_t pending_ecu_reset_req_id = 0;
  void send_ecu_reset(uint16_t req_id);

  // ---- HVC backup engine: a framework sequence of HVBMS scalar reads, then HVP (retargeted). ----
  enum HvcBackupPhase : uint8_t {
    HVC_BK_IDLE = 0,
    HVC_BK_HVBMS,      // HVBMS DID reads (energy, pack info, identity, config)
    HVC_BK_HVBMS_CAL,  // HVBMS calibration records 0/1/6 via routine 0x439
    HVC_BK_HVP,        // HVP reads (assembly, usage, config, calibration records)
  };
  TeslaHvcBackup* hvc_backup = nullptr;  // heap-allocated on first use, persists for the download
  bool hvc_backup_ready = false;
  bool hvc_backup_refused = false;  // last request rejected, for the UI
  uint16_t hvc_bk_reads_total = 0;  // reads attempted in the current/last run
  uint16_t hvc_bk_reads_ok = 0;     // of those, how many produced usable data
  uint8_t hvc_bk_phase = HVC_BK_IDLE;
  uint8_t hvc_bk_step = 0;     // index into the current phase's scalar table
  bool hvc_bk_queued = false;  // the TESLA_UDS_HVC sequence has been accepted by the framework
  uint16_t hvc_bk_runs = 0;    // completed runs; the web UI downloads when it sees this advance
  // Actual car reads the backup with the car awake. If BE was walking 0x221 down to OFF, hold ACCESSORY
  // for the run and put the mode back afterwards; the contactors are not touched either way.
  bool hvc_bk_accessory_held = false;
  bool hvc_bk_saved_accessory_forced = false;
  bool hvc_bk_saved_balance_at_rest = false;
  void release_hvc_backup_hold();
  bool hvc_send_current();  // request the current step's DID; false if the channel refused
  // Handle a step's response (data == nullptr for a timeout): store, record NRCs, try the
  // calibration routine fallback, then advance. hvc_store_scalar returns false for a response
  // that answered but was unusable (short), so "n of m reads" reflects data actually captured.
  void hvc_handle_response(uint16_t state, uint8_t sid, const uint8_t* data, uint16_t len);
  bool hvc_store_scalar(const uint8_t* rx, uint16_t len, bool via_routine);
  void hvc_score_and_advance(bool stored);  // count the read, move to the next step or finish
  void finish_hvc_backup();

 public:
  // Starts the backup read. False if the pack cannot be read right now (not on the bus, emulator
  // faulted/stopped, or the UDS channel is busy).
  bool start_hvc_backup();
  bool hvc_backup_available() { return hvc_backup_ready; }
  String hvc_backup_json();
  String hvc_backup_name();
  String hvc_backup_status_json();
  void refuse_hvc_backup() { hvc_backup_refused = true; }

  bool supports_charged_energy() { return true; }

  bool supports_manual_balancing() { return true; }

  BatteryHtmlRenderer& get_status_renderer() { return renderer; }

  static constexpr const char* NameSX = "Tesla Model S/X";
  static constexpr const char* Name3Y = "Tesla Model 3/Y";

 private:
  TeslaHtmlRenderer renderer;

 protected:
  /* Do not change anything below this line! */
  static const int MAX_PACK_VOLTAGE_SX_NCMA = 4600;   // V+1, if pack voltage goes over this, charge stops
  static const int MIN_PACK_VOLTAGE_SX_NCMA = 2850;   // V+1, if pack voltage goes over this, charge stops
  static const int MAX_PACK_VOLTAGE_3Y_NCMA = 4030;   // V+1, if pack voltage goes over this, charge stops
  static const int MIN_PACK_VOLTAGE_3Y_NCMA = 2850;   // V+1, if pack voltage goes below this, discharge stops
  static const int MAX_PACK_VOLTAGE_3Y_LFP = 3880;    // V+1, if pack voltage goes over this, charge stops
  static const int MIN_PACK_VOLTAGE_3Y_LFP = 2968;    // V+1, if pack voltage goes below this, discharge stops
  static const int MAX_CELL_DEVIATION_NCA_NCM = 500;  //LED turns yellow on the board if mv delta exceeds this value
  static const int MAX_CELL_DEVIATION_LFP = 400;      //LED turns yellow on the board if mv delta exceeds this value
  static const int MAX_CELL_VOLTAGE_NCA_NCM =
      4250;  //Battery is put into emergency stop if one cell goes over this value
  static const int MIN_CELL_VOLTAGE_NCA_NCM =
      2950;                                      //Battery is put into emergency stop if one cell goes below this value
  static const int MAX_CELL_VOLTAGE_LFP = 3650;  //Battery is put into emergency stop if one cell goes over this value
  static const int MIN_CELL_VOLTAGE_LFP = 2800;  //Battery is put into emergency stop if one cell goes below this value

  DATALAYER_BATTERY_TYPE* datalayer_battery;

  // Per-instance state for handle_incoming_can_frame.
  // These were previously static locals — static locals are shared across all
  // class instances, which breaks double/triple battery support.
  uint8_t mux = 0;
  uint16_t temp = 0;
  bool mux0_read = false;
  bool mux1_read = false;
  uint16_t brick_volts = 0;      // per-brick voltage scratch variable (0x401)
  uint8_t mux_zero_counter = 0;  // counts mux==0 frames to detect full cell scan
  uint8_t mux_max = 0;           // highest mux index seen so far

  // Per-instance state for transmit_can.
  int transmitPhase = -1;

  bool operate_contactors = false;

  // If not null, this battery decides when the contactor can be closed and writes the value here.
  bool* allows_contactor_closing;

  void printFaultCodesIfActive();
  void printFaultCodesPcsCp();  // PCS 0x3A4 + CP 0x31E alert matrices (tesla-m3-pack-findings)

  unsigned long previousMillis10 = 0;    // will store last time a 50ms CAN Message was sent
  unsigned long previousMillis50 = 0;    // will store last time a 50ms CAN Message was sent
  unsigned long previousMillis100 = 0;   // will store last time a 100ms CAN Message was sent
  unsigned long previousMillis500 = 0;   // will store last time a 500ms CAN Message was sent
  unsigned long previousMillis1000 = 0;  // will store last time a 1000ms CAN Message was sent

  //UDS session tracker
  //static bool uds_SessionInProgress = false; // Future use
  //0x221 VCFRONT_LVPowerState
  uint8_t alternateMux = 1;
  uint8_t frameCounter_TESLA_221 = 15;   // Start at 15 for Mux 0
  uint8_t vehicleState = 1;              // "OFF": 0, "DRIVE": 1, "ACCESSORY": 2, "GOING_DOWN": 3
  bool accessory_mode_forced = false;    // vehicleState = ACCESSORY (set by button or auto), otherwise DRIVE
  bool power_zeroed = false;             // ACCESSORY button zeroes charge/discharge; DRIVE button clears it
  bool balance_at_rest_enabled = false;  // experimental: auto DRIVE/ACCESSORY by load so the pack can balance at rest
  unsigned long rest_below_since_ms = 0;
  static constexpr int32_t TESLA_REST_THRESHOLD_dA = 10;  // 1 A in deci-amps
  static constexpr unsigned long TESLA_REST_DWELL_MS = 10000;
  static const uint8_t CAR_OFF = 0;
  static const uint8_t CAR_DRIVE = 1;
  static const uint8_t ACCESSORY = 2;
  static const uint8_t GOING_DOWN = 3;
  uint8_t powerDownSeconds = 9;  // Car power down (i.e. contactor open) tracking timer, 3 seconds per sendingState
  //0x2E1 VCFRONT_status, 6 mux tracker
  uint8_t muxNumber_TESLA_2E1 = 5;
  //0x334 UI
  bool TESLA_334_INITIAL_SENT = false;
  //Boot frames
  bool TESLA_313_INITIAL_SENT = false;
  bool TESLA_39D_INITIAL_SENT = false;
  bool TESLA_3C2_INITIAL_SENT = false;
  //0x3A1 VCFRONT_vehicleStatus, 15 frame counter (temporary)
  uint8_t frameCounter_TESLA_3A1 = 0;
  bool timeToMux3A1 = true;
  //Withhold 0x3A1 until the pack advertises drive power >=100kW (raw 7692 at
  //0.013 kW/bit on 0x252), then stream it continuously.
  bool tesla_3A1_armed = false;
  static constexpr uint16_t TESLA_3A1_ARM_DISCHARGE_RAW = 7692;  // ~100 kW
  //0x504 TWC_status
  bool TESLA_504_INITIAL_SENT = false;
  //0x7FF GTW_carConfig, 5 mux tracker
  uint8_t muxNumber_TESLA_7FF = 0;
  //Max percentage charge tracker
  uint16_t previous_max_percentage = 0;

  //0x082 UI_tripPlanning: "cycle_time" 1000ms
  static constexpr CAN_frame TESLA_082 = {.FD = false,
                                          .ext_ID = false,
                                          .DLC = 8,
                                          .ID = 0x082,
                                          .data = {0x00, 0x00, 0x00, 0x80, 0x00, 0x80, 0x00, 0x80}};

  //0x102 VCLEFT_doorStatus: "cycle_time" 100ms
  static constexpr CAN_frame TESLA_102 = {.FD = false,
                                          .ext_ID = false,
                                          .DLC = 8,
                                          .ID = 0x102,
                                          .data = {0x22, 0x33, 0x00, 0x00, 0xC0, 0x38, 0x21, 0x08}};

  //0x103 VCRIGHT_doorStatus: "cycle_time" 100ms
  static constexpr CAN_frame TESLA_103 = {.FD = false,
                                          .ext_ID = false,
                                          .DLC = 8,
                                          .ID = 0x103,
                                          .data = {0x22, 0x33, 0x00, 0x00, 0x30, 0xF2, 0x20, 0x02}};

  //0x118 DI_systemStatus: "cycle_time" 50ms, DI_systemStatusChecksum/DI_systemStatusCounter generated via generateFrameCounterChecksum
  CAN_frame TESLA_118 = {.FD = false,
                         .ext_ID = false,
                         .DLC = 8,
                         .ID = 0x118,
                         .data = {0xAB, 0x60, 0x2A, 0x00, 0x00, 0x08, 0x00, 0x00}};
  CAN_frame TESLA_118_digital_hvil = {.FD = false,
                                      .ext_ID = false,
                                      .DLC = 8,
                                      .ID = 0x118,
                                      .data = {0x61, 0x80, 0x30, 0x10, 0x00, 0x08, 0x00, 0x80}};
  uint16_t content_118_digital_hvil[16] = {0x6180, 0x6281, 0x6382, 0x6483, 0x6584, 0x6685, 0x6786, 0x6887,
                                           0x6988, 0x6A89, 0x6B8A, 0x6C8B, 0x6D8C, 0x6E8D, 0x6F8E, 0x708F};

  //0x2A8 EPBL_status: "cycle_time" 100ms. Left electronic park brake; shares the counter(52|4)/
  //checksum(56|8) layout with its right-hand pair 0x2E8 EPBR_status via generateFrameCounterChecksum.
  CAN_frame TESLA_2A8 = {.FD = false,
                         .ext_ID = false,
                         .DLC = 8,
                         .ID = 0x2A8,
                         .data = {0x02, 0x00, 0x00, 0x00, 0x00, 0x80, 0x00, 0x2C}};

  //0x213 UI_cruiseControl: "cycle_time" 500ms, UI_speedLimitTick/UI_cruiseControlCounter - different depending on firmware, semi-manual increment for now
  //Ref tesla-m3-pack-findings (fw 2019.20.4.2): 0x213 UI_cruiseControl DLC 3 (this frame uses DLC 2; likely firmware drift)
  CAN_frame TESLA_213 = {.FD = false, .ext_ID = false, .DLC = 2, .ID = 0x213, .data = {0x00, 0x15}};

  //0x221 These frames will/should eventually be migrated to 2 base frames (1 per mux), and then just the relevant bits changed

  //0x221 VCFRONT_LVPowerState "Drive"
  //VCFRONT_vehiclePowerState VEHICLE_POWER_STATE_DRIVE (Mux0, Counter 15): "cycle_time" 50ms each mux/LVPowerStateIndex, VCFRONT_LVPowerStateChecksum/VCFRONT_LVPowerStateCounter generated via generateMuxFrameCounterChecksum
  CAN_frame TESLA_221_DRIVE_Mux0 = {.FD = false,
                                    .ext_ID = false,
                                    .DLC = 8,
                                    .ID = 0x221,
                                    .data = {0x60, 0x55, 0x55, 0x15, 0x54, 0x51, 0xF1, 0xD8}};

  //0x221 VCFRONT_LVPowerState "Drive"
  //VCFRONT_vehiclePowerState VEHICLE_POWER_STATE_DRIVE (Mux1, Counter 0): "cycle_time" 50ms each mux/LVPowerStateIndex, VCFRONT_LVPowerStateChecksum/VCFRONT_LVPowerStateCounter generated via generateMuxFrameCounterChecksum
  CAN_frame TESLA_221_DRIVE_Mux1 = {.FD = false,
                                    .ext_ID = false,
                                    .DLC = 8,
                                    .ID = 0x221,
                                    .data = {0x61, 0x05, 0x55, 0x05, 0x00, 0x00, 0xE0, 0x16}};

  //0x221 VCFRONT_LVPowerState "Accessory"
  //VCFRONT_vehiclePowerState VEHICLE_POWER_STATE_ACCESSORY (Mux0, Counter 15): "cycle_time" 50ms each mux/LVPowerStateIndex, VCFRONT_LVPowerStateChecksum/VCFRONT_LVPowerStateCounter generated via generateMuxFrameCounterChecksum
  CAN_frame TESLA_221_ACCESSORY_Mux0 = {.FD = false,
                                        .ext_ID = false,
                                        .DLC = 8,
                                        .ID = 0x221,
                                        .data = {0x40, 0x55, 0x55, 0x05, 0x54, 0x51, 0xF5, 0xAC}};

  //0x221 VCFRONT_LVPowerState "Accessory"
  //VCFRONT_vehiclePowerState VEHICLE_POWER_STATE_ACCESSORY (Mux1, Counter 0): "cycle_time" 50ms each mux/LVPowerStateIndex, VCFRONT_LVPowerStateChecksum/VCFRONT_LVPowerStateCounter generated via generateMuxFrameCounterChecksum
  CAN_frame TESLA_221_ACCESSORY_Mux1 = {.FD = false,
                                        .ext_ID = false,
                                        .DLC = 8,
                                        .ID = 0x221,
                                        .data = {0x41, 0x05, 0x55, 0x55, 0x01, 0x00, 0x04, 0x18}};

  //0x221 VCFRONT_LVPowerState "Going Down"
  //VCFRONT_vehiclePowerState VEHICLE_POWER_STATE_OFF, key parts GOING_DOWN (Mux0, Counter 15): "cycle_time" 50ms each mux/LVPowerStateIndex, VCFRONT_LVPowerStateChecksum/VCFRONT_LVPowerStateCounter generated via generateMuxFrameCounterChecksum
  CAN_frame TESLA_221_GOING_DOWN_Mux0 = {.FD = false,
                                         .ext_ID = false,
                                         .DLC = 8,
                                         .ID = 0x221,
                                         .data = {0x00, 0x89, 0x55, 0x06, 0xA4, 0x51, 0xF1, 0xED}};

  //0x221 VCFRONT_LVPowerState "Going Down"
  //VCFRONT_vehiclePowerState VEHICLE_POWER_STATE_OFF, key parts GOING_DOWN (Mux1, Counter 0): "cycle_time" 50ms each mux/LVPowerStateIndex, VCFRONT_LVPowerStateChecksum/VCFRONT_LVPowerStateCounter generated via generateMuxFrameCounterChecksum
  CAN_frame TESLA_221_GOING_DOWN_Mux1 = {.FD = false,
                                         .ext_ID = false,
                                         .DLC = 8,
                                         .ID = 0x221,
                                         .data = {0x01, 0x09, 0x55, 0x59, 0x00, 0x00, 0x00, 0xDB}};

  //0x221 VCFRONT_LVPowerState "Off"
  //VCFRONT_vehiclePowerState VEHICLE_POWER_STATE_OFF, key parts OFF (Mux0, Counter 15): "cycle_time" 50ms each mux/LVPowerStateIndex, VCFRONT_LVPowerStateChecksum/VCFRONT_LVPowerStateCounter generated via generateMuxFrameCounterChecksum
  CAN_frame TESLA_221_OFF_Mux0 = {.FD = false,
                                  .ext_ID = false,
                                  .DLC = 8,
                                  .ID = 0x221,
                                  .data = {0x00, 0x01, 0x00, 0x00, 0x00, 0x50, 0xF1, 0x65}};

  //0x221 VCFRONT_LVPowerState "Off"
  //VCFRONT_vehiclePowerState VEHICLE_POWER_STATE_OFF, key parts OFF (Mux1, Counter 0): "cycle_time" 50ms each mux/LVPowerStateIndex, VCFRONT_LVPowerStateChecksum/VCFRONT_LVPowerStateCounter generated via generateMuxFrameCounterChecksum
  CAN_frame TESLA_221_OFF_Mux1 = {.FD = false,
                                  .ext_ID = false,
                                  .DLC = 8,
                                  .ID = 0x221,
                                  .data = {0x01, 0x01, 0x01, 0x50, 0x00, 0x00, 0x00, 0x76}};

  //0x229 SCCM_rightStalk: "cycle_time" 100ms, SCCM_rightStalkChecksum/SCCM_rightStalkCounter generated via dedicated generateTESLA_229 function for now
  //CRC seemingly related to AUTOSAR ID array... "autosarDataIds": [124,182,240,47,105,163,221,28,86,144,202,9,67,125,183,241] found in Model 3 firmware
  CAN_frame TESLA_229 = {.FD = false, .ext_ID = false, .DLC = 3, .ID = 0x229, .data = {0x46, 0x00, 0x00}};

  //0x241 VCFRONT_coolant: "cycle_time" 100ms
  static constexpr CAN_frame TESLA_241 = {.FD = false,
                                          .ext_ID = false,
                                          .DLC = 7,
                                          .ID = 0x241,
                                          .data = {0x35, 0x34, 0x0C, 0x0F, 0x8F, 0x55, 0x00}};

  //VCFRONT/VC messages the PCS monitors for MIA (PCS_a024). Missing any of these raises the alert.
  //0x2D1 VCFRONT_okToUseHighPower: "cycle_time" 100ms
  static constexpr CAN_frame TESLA_2D1 = {.FD = false, .ext_ID = false, .DLC = 2, .ID = 0x2D1, .data = {0xFF, 0x01}};

  //0x2E1, 6 muxes
  //0x2E1 VCFRONT_status: "cycle_time" 10ms each mux/statusIndex
  static constexpr CAN_frame TESLA_2E1_VEHICLE_AND_RAILS = {.FD = false,
                                                            .ext_ID = false,
                                                            .DLC = 8,
                                                            .ID = 0x2E1,
                                                            .data = {0x29, 0x0A, 0x00, 0xFF, 0x0F, 0x00, 0x00, 0x00}};

  //{0x29, 0x0A, 0x00, 0xFF, 0x0F, 0x00, 0x00, 0x00} INIT
  //{0x29, 0x0A, 0x0D, 0xFF, 0x0F, 0x00, 0x00, 0x00} DRIVE
  //{0x29, 0x0A, 0x09, 0xFF, 0x0F, 0x00, 0x00, 0x00} HV_UP_STANDBY
  //{0x29, 0x0A, 0x0A, 0xFF, 0x0F, 0x00, 0x00, 0x00} ACCESSORY
  //{0x29, 0x0A, 0x06, 0xFF, 0x0F, 0x00, 0x00, 0x00} SLEEP_STANDBY

  //0x2E1 VCFRONT_status: "cycle_time" 10ms each mux/statusIndex
  static constexpr CAN_frame TESLA_2E1_HOMELINK = {.FD = false,
                                                   .ext_ID = false,
                                                   .DLC = 8,
                                                   .ID = 0x2E1,
                                                   .data = {0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x00}};

  //0x2E1 VCFRONT_status: "cycle_time" 10ms each mux/statusIndex
  static constexpr CAN_frame TESLA_2E1_REFRIGERANT_SYSTEM = {.FD = false,
                                                             .ext_ID = false,
                                                             .DLC = 8,
                                                             .ID = 0x2E1,
                                                             .data = {0x03, 0x6D, 0x99, 0x02, 0x1B, 0x57, 0x00, 0x00}};

  //0x2E1 VCFRONT_status: "cycle_time" 10ms each mux/statusIndex
  static constexpr CAN_frame TESLA_2E1_LV_BATTERY_DEBUG = {.FD = false,
                                                           .ext_ID = false,
                                                           .DLC = 8,
                                                           .ID = 0x2E1,
                                                           .data = {0xFC, 0x1B, 0xD1, 0x99, 0x9A, 0xD8, 0x09, 0x00}};

  //0x2E1 VCFRONT_status: "cycle_time" 10ms each mux/statusIndex
  //Mux 5 (VCFRONT_statusIndex=5) carries VCFRONT_UP_inDriveState at bit 5, which the BMS keys off
  //(together with 0x221 vehiclePowerState=DRIVE) to raise BMS_powerLimitsState and grant drive power.
  //Set at transmit time from vehicleState, so it is a mutable member rather than static constexpr.
  static constexpr CAN_frame TESLA_2E1_MUX_5 = {.FD = false,
                                                .ext_ID = false,
                                                .DLC = 8,
                                                .ID = 0x2E1,
                                                .data = {0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}};

  //0x2E1 VCFRONT_status: "cycle_time" 10ms each mux/statusIndex
  static constexpr CAN_frame TESLA_2E1_BODY_CONTROLS = {.FD = false,
                                                        .ext_ID = false,
                                                        .DLC = 8,
                                                        .ID = 0x2E1,
                                                        .data = {0x08, 0x21, 0x04, 0x6E, 0xA0, 0x88, 0x06, 0x04}};

  //0x2E8 EPBR_status: "cycle_time" 100ms. Right electronic park brake; shares the counter(52|4)/
  //checksum(56|8) layout with its left-hand pair 0x2A8 EPBL_status via generateFrameCounterChecksum.
  CAN_frame TESLA_2E8 = {.FD = false,
                         .ext_ID = false,
                         .DLC = 8,
                         .ID = 0x2E8,
                         .data = {0x02, 0x00, 0x10, 0x00, 0x00, 0x80, 0x00, 0x6C}};

  //0x284 UI_vehicleModes: "cycle_time" 500ms
  static constexpr CAN_frame TESLA_284 = {.FD = false,
                                          .ext_ID = false,
                                          .DLC = 5,
                                          .ID = 0x284,
                                          .data = {0x10, 0x00, 0x00, 0x00, 0x00}};

  //0x293 UI_chassisControl: "cycle_time" 500ms, UI_chassisControlChecksum/UI_chassisControlCounter generated via generateFrameCounterChecksum
  CAN_frame TESLA_293 = {.FD = false,
                         .ext_ID = false,
                         .DLC = 8,
                         .ID = 0x293,
                         .data = {0x01, 0x0C, 0x55, 0x91, 0x55, 0x15, 0x71, 0x63}};

  //0x3A1 VCFRONT_vehicleStatus: "cycle_time" 50ms, VCFRONT_vehicleStatusChecksum/VCFRONT_vehicleStatusCounter eventually need to be generated via generateMuxFrameCounterChecksum
  //Looks like 2 muxes, counter at bit 52 width 4 and checksum at bit 56 width 8? Need later software Model3_ETH.compact.json signal file or DBC.
  CAN_frame TESLA_3A1 = {.FD = false,
                         .ext_ID = false,
                         .DLC = 8,
                         .ID = 0x3A1,
                         .data = {0xC3, 0xFF, 0xFF, 0xFF, 0x3D, 0x00, 0xD0, 0x01}};
  uint8_t frame6_3A1[16] = {0xD0, 0xE2, 0xF0, 0x02, 0x10, 0x22, 0x30, 0x42,
                            0x50, 0x62, 0x70, 0x82, 0x90, 0xA2, 0xB0, 0xC2};
  uint8_t frame7_3A1[16] = {0x01, 0xCB, 0x21, 0xEB, 0x41, 0x0B, 0x61, 0x2B,
                            0x81, 0x4B, 0xA1, 0x6B, 0xC1, 0x8B, 0xE1, 0xAB};
  //0x313 UI_powertrainControl: "cycle_time" 500ms, UI_powertrainControlChecksum/UI_powertrainControlCounter generated via generateFrameCounterChecksum
  //Ref tesla-m3-pack-findings (fw 2019.20.4.2): CAN 0x313 = UI_trackModeSettings on that firmware; UI_powertrainControl is 0x334 there (name drift)
  //0x313 UI_powertrainControl (initial frame): counter 4, checksum 0x25
  static constexpr CAN_frame TESLA_313_INITIAL = {.FD = false,
                                                  .ext_ID = false,
                                                  .DLC = 8,
                                                  .ID = 0x313,
                                                  .data = {0x02, 0x64, 0x64, 0x05, 0x00, 0x00, 0x40, 0x25}};

  //Seeded to counter 4 so the first regenerate lands on counter 5, continuing the sequence from
  //TESLA_313_INITIAL. This frame itself is never transmitted - the boot frame goes out in its place -
  //so its checksum byte is inert.
  CAN_frame TESLA_313 = {.FD = false,
                         .ext_ID = false,
                         .DLC = 8,
                         .ID = 0x313,
                         .data = {0x00, 0x00, 0x00, 0x05, 0x00, 0x00, 0x40, 0x5B}};

  //0x321 VCFRONT_sensors: "cycle_time" 1000ms
  //Counter seeded to 5 (byte6 hi-nibble)
  CAN_frame TESLA_321 = {
      .FD = false,
      .ext_ID = false,
      .DLC = 8,
      .ID = 0x321,
      .data = {0xEC, 0x71, 0xA7, 0x6E, 0x02, 0x6C, 0x50, 0x54}};  // Last 2 bytes are counter and checksum

  //0x333 UI_chargeRequest: "cycle_time" 500ms, UI_chargeTerminationPct value = 900 [bit 16, width 10, scale 0.1, min 25, max 100]
  //Ref tesla-m3-pack-findings (fw 2019.20.4.2): 0x333 UI_chargeRequest DLC 4 (this frame uses DLC 5; likely firmware drift)
  CAN_frame TESLA_333 = {.FD = false, .ext_ID = false, .DLC = 5, .ID = 0x333, .data = {0x84, 0x30, 0x84, 0x07, 0x02}};

  //0x334 UI request: "cycle_time" 500ms, initial frame car sends
  //Ref tesla-m3-pack-findings (fw 2019.20.4.2): CAN 0x334 = UI_powertrainControl on that firmware
  static constexpr CAN_frame TESLA_334_INITIAL = {.FD = false,
                                                  .ext_ID = false,
                                                  .DLC = 8,
                                                  .ID = 0x334,
                                                  .data = {0x3F, 0x3F, 0xC8, 0x00, 0xE2, 0x3F, 0x80, 0x1E}};

  //0x334 UI request: "cycle_time" 500ms, generated via generateFrameCounterChecksum
  //Seeded to counter 8, not 9
  CAN_frame TESLA_334 = {.FD = false,
                         .ext_ID = false,
                         .DLC = 8,
                         .ID = 0x334,
                         .data = {0x3F, 0x3F, 0x00, 0x0F, 0xE2, 0x3F, 0x80, 0x65}};

  //0x3B3 UI_vehicleControl2: "cycle_time" 500ms
  //Ref tesla-m3-pack-findings (fw 2019.20.4.2): 0x3B3 UI_vehicleControl2 DLC 2 (this frame uses DLC 8; likely firmware drift)
  static constexpr CAN_frame TESLA_3B3 = {.FD = false,
                                          .ext_ID = false,
                                          .DLC = 8,
                                          .ID = 0x3B3,
                                          .data = {0x90, 0x80, 0x05, 0x08, 0x00, 0x00, 0x00, 0x01}};

  //0x39D IBST_status (initial frame): counter 8, checksum 0x00
  static constexpr CAN_frame TESLA_39D_INITIAL = {.FD = false,
                                                  .ext_ID = false,
                                                  .DLC = 5,
                                                  .ID = 0x39D,
                                                  .data = {0x00, 0x58, 0xE1, 0x27, 0x00}};

  //0x39D IBST_status: "cycle_time" 50ms, IBST_statusChecksum/IBST_statusCounter generated via generateFrameCounterChecksum
  //Seeded to counter 8 so the first regenerate lands on counter 9
  CAN_frame TESLA_39D = {.FD = false, .ext_ID = false, .DLC = 5, .ID = 0x39D, .data = {0xE0, 0x58, 0xC1, 0x27, 0x00}};

  //0x3C2 VCLEFT_switchStatus (Mux0, initial frame car sends): "cycle_time" 50ms, sent once.
  static constexpr CAN_frame TESLA_3C2_INITIAL = {.FD = false,
                                                  .ext_ID = false,
                                                  .DLC = 8,
                                                  .ID = 0x3C2,
                                                  .data = {0x00, 0x55, 0x55, 0x55, 0x00, 0x00, 0x5A, 0x05}};

  //0x3C2 VCLEFT_switchStatus (Mux0): "cycle_time" 50ms each mux/SwitchStatusIndex
  static constexpr CAN_frame TESLA_3C2_Mux0 = {.FD = false,
                                               .ext_ID = false,
                                               .DLC = 8,
                                               .ID = 0x3C2,
                                               .data = {0x00, 0x55, 0x55, 0x55, 0x00, 0x00, 0x5A, 0x45}};

  //0x3C2 VCLEFT_switchStatus (Mux1): "cycle_time" 50ms each mux/SwitchStatusIndex
  static constexpr CAN_frame TESLA_3C2_Mux1 = {.FD = false,
                                               .ext_ID = false,
                                               .DLC = 8,
                                               .ID = 0x3C2,
                                               .data = {0x29, 0x55, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}};

  //0x504 Initially sent
  static constexpr CAN_frame TESLA_504_INITIAL = {.FD = false,
                                                  .ext_ID = false,
                                                  .DLC = 8,
                                                  .ID = 0x504,
                                                  .data = {0x00, 0x1B, 0x06, 0x03, 0x00, 0x01, 0x00, 0x01}};

  //0x55A Unknown but always sent: "cycle_time" 500ms
  static constexpr CAN_frame TESLA_55A = {.FD = false,
                                          .ext_ID = false,
                                          .DLC = 8,
                                          .ID = 0x55A,
                                          .data = {0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}};

  //0x7FF GTW_carConfig: "cycle_time" 100ms each mux/carConfigMultiplexer
  //Fixed values now
  static constexpr CAN_frame TESLA_7FF_Mux1 = {.FD = false,
                                               .ext_ID = false,
                                               .DLC = 8,
                                               .ID = 0x7FF,
                                               .data = {0x01, 0x41, 0x41, 0x43, 0x00, 0x03, 0x15, 0x01}};

  //0x7FF GTW_carConfig: "cycle_time" 100ms each mux/carConfigMultiplexer
  //Not constexpr: GTW_numberHVILNodes (bit 62, width 2) is written from selected pack in setup()
  CAN_frame TESLA_7FF_Mux2 = {.FD = false,
                              .ext_ID = false,
                              .DLC = 8,
                              .ID = 0x7FF,
                              .data = {0x02, 0x66, 0x32, 0x24, 0x04, 0x49, 0x95, 0x82}};

  //0x7FF GTW_carConfig: "cycle_time" 100ms each mux/carConfigMultiplexer
  //mapRegion US (bits 8-11) fixed.
  //chassisType (bit 18, width 3) and packEnergy (bit 32, width 5) are written in setup()
  CAN_frame TESLA_7FF_Mux3 = {.FD = false,
                              .ext_ID = false,
                              .DLC = 8,
                              .ID = 0x7FF,
                              .data = {0x03, 0x00, 0x08, 0x48, 0x01, 0x00, 0x00, 0x12}};

  //0x7FF GTW_carConfig: "cycle_time" 100ms each mux/carConfigMultiplexer
  static constexpr CAN_frame TESLA_7FF_Mux4 = {.FD = false,
                                               .ext_ID = false,
                                               .DLC = 8,
                                               .ID = 0x7FF,
                                               .data = {0x04, 0x73, 0x03, 0x67, 0x5C, 0x00, 0x00, 0x00}};

  //0x7FF GTW_carConfig: "cycle_time" 100ms each mux/carConfigMultiplexer
  static constexpr CAN_frame TESLA_7FF_Mux5 = {.FD = false,
                                               .ext_ID = false,
                                               .DLC = 8,
                                               .ID = 0x7FF,
                                               .data = {0x05, 0x00, 0x00, 0x00, 0x20, 0x00, 0x00, 0x00}};

  //0x7FF GTW_carConfig: "cycle_time" 100ms each mux/carConfigMultiplexer - later firmware has muxes 6 & 7, needed?
  static constexpr CAN_frame TESLA_7FF_Mux6 = {.FD = false,
                                               .ext_ID = false,
                                               .DLC = 8,
                                               .ID = 0x7FF,
                                               .data = {0x06, 0x00, 0x30, 0x00, 0x00, 0x00, 0x00, 0xD0}};

  //0x7FF GTW_carConfig: "cycle_time" 100ms each mux/carConfigMultiplexer - later firmware has muxes 6 & 7, needed?
  static constexpr CAN_frame TESLA_7FF_Mux7 = {.FD = false,
                                               .ext_ID = false,
                                               .DLC = 8,
                                               .ID = 0x7FF,
                                               .data = {0x07, 0x00, 0x00, 0x05, 0x00, 0x00, 0x00, 0x00}};

  //0x722 BMS_bmbKeepAlive: "cycle_time" 100ms, should only be sent when testing packs or diagnosing problems
  static constexpr CAN_frame TESLA_722 = {.FD = false,
                                          .ext_ID = false,
                                          .DLC = 8,
                                          .ID = 0x722,
                                          .data = {0x02, 0x02, 0x04, 0x08, 0x10, 0x20, 0x40, 0x80}};

  //0x25D CP_status: "cycle_time" 100ms, stops some cpMia errors, but not necessary for standalone pack operation so not used/necessary. Note CP_type for different regions, the below has "IEC_CCS"
  static constexpr CAN_frame TESLA_25D = {.FD = false,
                                          .ext_ID = false,
                                          .DLC = 8,
                                          .ID = 0x25D,
                                          .data = {0x37, 0x41, 0x01, 0x16, 0x08, 0x00, 0x00, 0x00}};

  //1CF needed for Digital HVIL option
  CAN_frame TESLA_1CF_digital_hvil = {.FD = false,
                                      .ext_ID = false,
                                      .DLC = 8,
                                      .ID = 0x1CF,
                                      .data = {0x01, 0x00, 0x00, 0x1A, 0x1C, 0x02, 0x60, 0x69}};
  uint16_t content_1CF_digital_hvil[8] = {0x6069, 0x8089, 0xA0A9, 0xC0C9, 0xE0E9, 0x0009, 0x2029, 0x4049};
  uint8_t index_1CF = 0;
  uint8_t index_118 = 0;
  uint8_t contactor_counter = 0;
  uint16_t battery_cell_max_v = 3300;
  uint16_t battery_cell_min_v = 3300;
  bool cellvoltagesRead = false;

  //0x372: 882 BMS_log1 mux 3, 0x3B2: 946 BMS_log2 mux 35/39 -- balancing tracking
  uint8_t BMS_balanceState = BALANCE_STATE_IDLE;
  bool BMS_vshTestNeeded = false;
  bool BMS_balanceStateUnknown = false;
  uint16_t BMS_balanceTimeThisCycle_s = 0;
  uint32_t BMS_cumulativeBleedTime_s = 0;
  uint8_t BMS_cacEstimationSource = 0;
  //0x3d2: 978 BMS_kwhCounter
  uint32_t battery_total_discharge = 0;
  uint32_t battery_total_charge = 0;
  //0x352: 850 BMS_energyStatus
  bool BMS352_mux = false;                            // variable to store when 0x352 mux is present
  uint16_t battery_energy_buffer = 0;                 // kWh
  uint16_t battery_energy_buffer_m1 = 0;              // kWh
  uint16_t battery_energy_to_charge_complete = 0;     // kWh
  uint16_t battery_energy_to_charge_complete_m1 = 0;  // kWh
  uint16_t battery_expected_energy_remaining = 0;     // kWh
  uint16_t battery_expected_energy_remaining_m1 = 0;  // kWh
  bool battery_full_charge_complete = false;          // Changed to bool
  bool battery_fully_charged = false;                 // Changed to bool
  uint16_t battery_ideal_energy_remaining = 0;        // kWh
  uint16_t battery_ideal_energy_remaining_m0 = 0;     // kWh
  uint16_t battery_nominal_energy_remaining = 0;      // kWh
  uint16_t battery_nominal_energy_remaining_m0 = 0;   // kWh
  uint16_t battery_nominal_full_pack_energy = 0;      // Kwh
  uint16_t battery_nominal_full_pack_energy_m0 = 0;   // Kwh
  //0x132 306 HVBattAmpVolt
  uint16_t battery_volts = 0;                  // V
  int16_t battery_amps = 0;                    // A
  uint16_t battery_charge_time_remaining = 0;  // Minutes
  //0x252 594 BMS_powerAvailable
  uint16_t BMS_maxRegenPower = 0;           //rename from battery_regenerative_limit
  uint16_t BMS_maxDischargePower = 0;       // rename from battery_discharge_limit
  uint16_t BMS_maxStationaryHeatPower = 0;  //rename from battery_max_heat_park
  uint16_t BMS_hvacPowerBudget = 0;         //rename from battery_hvac_max_power
  uint8_t BMS_notEnoughPowerForHeatPump = 0;
  uint8_t BMS_powerLimitState = 0;
  uint8_t BMS_inverterTQF = 0;
  //0x2d2: 722 BMSVAlimits
  uint16_t battery_max_discharge_current = 0;
  uint16_t battery_max_charge_current = 0;
  uint16_t BMS_max_voltage = 0;
  uint16_t BMS_min_voltage = 0;
  //0x2b4: 692 PCS_dcdcRailStatus
  uint16_t battery_dcdcHvBusVolt = 0;        // Change name from battery_high_voltage to battery_dcdcHvBusVolt
  uint16_t battery_dcdcLvBusVolt = 0;        // Change name from battery_low_voltage to battery_dcdcLvBusVolt
  uint16_t battery_dcdcLvOutputCurrent = 0;  // Change name from battery_output_current to battery_dcdcLvOutputCurrent
  //0x292: 658 BMS_socStatus
  uint16_t battery_beginning_of_life = 0;  // kWh
  uint16_t battery_soc_min = 0;
  uint16_t battery_soc_max = 0;
  uint16_t battery_soc_ui = 0;  //Change name from battery_soc_vi to reflect DBC battery_soc_ui
  uint16_t battery_soc_ave = 0;
  uint8_t battery_battTempPct = 0;
  //0x352 mux2: BMS_userFacingSoe 52|11@1+ (0.05,0) "%" - only transmitted by 2024.20+ pack firmware, 2047 = SNA
  uint16_t battery_userFacingSoe = 0;
  //0x392: BMS_packConfig
  uint32_t battery_packMass = 0;
  uint32_t battery_platformMaxBusVoltage = 0;
  uint32_t battery_packConfigMultiplexer = 0;
  uint32_t battery_moduleType = 0;
  uint32_t battery_reservedConfig = 0;
  //0x332: 818 BattBrickMinMax:BMS_bmbMinMax
  int16_t battery_max_temp = 0;  // C*
  int16_t battery_min_temp = 0;  // C*
  uint16_t battery_BrickVoltageMax = 0;
  uint16_t battery_BrickVoltageMin = 0;
  uint8_t battery_BrickTempMaxNum = 0;
  uint8_t battery_BrickTempMinNum = 0;
  uint8_t battery_BrickModelTMax = 0;
  uint8_t battery_BrickModelTMin = 0;
  uint8_t battery_BrickVoltageMaxNum = 0;  //rename from battery_max_vno
  uint8_t battery_BrickVoltageMinNum = 0;  //rename from battery_min_vno
  //0x20A: 522 HVP_contactorState
  uint8_t battery_contactor = 0;  //State of contactor
  uint8_t battery_hvil_status = 0;
  uint8_t battery_packContNegativeState = 0;
  uint8_t battery_packContPositiveState = 0;
  uint8_t battery_packContactorSetState = 0;
  bool battery_packCtrsClosingBlocked = false;    // Change to bool
  bool battery_pyroTestInProgress = false;        // Change to bool
  bool battery_packCtrsOpenNowRequested = false;  // Change to bool
  bool battery_packCtrsOpenRequested = false;     // Change to bool
  uint8_t battery_packCtrsRequestStatus = 0;
  bool battery_packCtrsResetRequestRequired = false;  // Change to bool
  bool battery_dcLinkAllowedToEnergize = false;       // Change to bool
  bool battery_fcContNegativeAuxOpen = false;         // Change to bool
  uint8_t battery_fcContNegativeState = 0;
  bool battery_fcContPositiveAuxOpen = false;  // Change to bool
  uint8_t battery_fcContPositiveState = 0;
  uint8_t battery_fcContactorSetState = 0;
  bool battery_fcCtrsClosingAllowed = false;    // Change to bool
  bool battery_fcCtrsOpenNowRequested = false;  // Change to bool
  bool battery_fcCtrsOpenRequested = false;     // Change to bool
  uint8_t battery_fcCtrsRequestStatus = 0;
  bool battery_fcCtrsResetRequestRequired = false;  // Change to bool
  bool battery_fcLinkAllowedToEnergize = false;     // Change to bool
                                                    //0x72A: BMS_serialNumber, or UDS 0xF015
  uint8_t battery_serialNumber[14] = {0};           // Stores raw HEX values for ASCII chars
  bool parsed_battery_serialNumber = false;
  //Derived from the serial number by derive_manufacture_date_from_serial(), which points this at a
  //static buffer. Must stay initialised: update_values() copies it into the datalayer for the web UI,
  //which would otherwise strlen() an indeterminate pointer and panic the board.
  char* battery_manufactureDate = nullptr;  // YYYY-MM-DD\0
                                            //Via UDS 0xF014
  uint8_t battery_partNumber[12] = {0};     //stores raw HEX values for ASCII chars
  bool parsed_battery_partNumber = false;
  //Via UDS 0xF012 on the HVBMS - the BMS board itself, not the pack package part number
  uint8_t BMS_partNumber[12] = {0};  //stores raw HEX values for ASCII chars
  bool parsed_BMS_partNumber = false;
  // Startup identity pass: one shot per boot, bounded retry while the BMS is not yet answering.
  bool ident_requested = false;
  uint8_t ident_attempts = 0;
  bool identity_from_uds = false;  // serial came from 0xF015 rather than the 0x72A broadcast
  static constexpr uint8_t TESLA_IDENT_MAX_ATTEMPTS = 3;
  //0x300: BMS_info
  uint16_t BMS_info_buildConfigId = 0;
  uint16_t BMS_info_hardwareId = 0;
  uint16_t BMS_info_componentId = 0;
  uint8_t BMS_info_pcbaId = 0;
  uint8_t BMS_info_assemblyId = 0;
  uint16_t BMS_info_usageId = 0;
  uint16_t BMS_info_subUsageId = 0;
  uint8_t BMS_info_platformType = 0;
  uint32_t BMS_info_appCrc = 0;
  uint64_t BMS_info_bootGitHash = 0;
  uint8_t BMS_info_bootUdsProtoVersion = 0;
  uint32_t BMS_info_bootCrc = 0;
  //0x212: 530 BMS_status
  bool BMS_hvacPowerRequest = false;          //Change to bool
  bool BMS_notEnoughPowerForDrive = false;    //Change to bool
  bool BMS_notEnoughPowerForSupport = false;  //Change to bool
  bool BMS_preconditionAllowed = false;       //Change to bool
  bool BMS_updateAllowed = false;             //Change to bool
  bool BMS_activeHeatingWorthwhile = false;   //Change to bool
  bool BMS_cpMiaOnHvs = false;                //Change to bool
  uint8_t BMS_contactorState = 0;
  uint8_t BMS_state = 0;
  uint8_t BMS_hvState = 0;
  uint16_t BMS_isolationResistance = 0;
  bool BMS_chargeRequest = false;    //Change to bool
  bool BMS_keepWarmRequest = false;  //Change to bool
  uint8_t BMS_uiChargeStatus = 0;
  bool BMS_diLimpRequest = false;   //Change to bool
  bool BMS_okToShipByAir = false;   //Change to bool
  bool BMS_okToShipByLand = false;  //Change to bool
  uint32_t BMS_chgPowerAvailable = 0;
  uint8_t BMS_chargeRetryCount = 0;
  bool BMS_pcsPwmEnabled = false;        //Change to bool
  bool BMS_ecuLogUploadRequest = false;  //Change to bool
  uint8_t BMS_minPackTemperature = 0;
  // 0x224:548 PCS_dcdcStatus
  uint8_t PCS_dcdcPrechargeStatus = 0;
  uint8_t PCS_dcdc12VSupportStatus = 0;
  uint8_t PCS_dcdcHvBusDischargeStatus = 0;
  uint16_t PCS_dcdcMainState = 0;
  uint8_t PCS_dcdcSubState = 0;
  bool PCS_dcdcFaulted = false;          //Change to bool
  bool PCS_dcdcOutputIsLimited = false;  //Change to bool
  uint32_t PCS_dcdcMaxOutputCurrentAllowed = 0;
  uint8_t PCS_dcdcPrechargeRtyCnt = 0;
  uint8_t PCS_dcdc12VSupportRtyCnt = 0;
  uint8_t PCS_dcdcDischargeRtyCnt = 0;
  uint8_t PCS_dcdcPwmEnableLine = 0;
  uint8_t PCS_dcdcSupportingFixedLvTarget = 0;
  uint8_t PCS_ecuLogUploadRequest = 0;
  uint8_t PCS_dcdcPrechargeRestartCnt = 0;
  uint8_t PCS_dcdcInitialPrechargeSubState = 0;
  //0x312: 786 BMS_thermalStatus
  uint16_t BMS_powerDissipation = 0;
  uint16_t BMS_flowRequest = 0;
  uint16_t BMS_inletActiveCoolTargetT = 0;
  uint16_t BMS_inletPassiveTargetT = 0;
  uint16_t BMS_inletActiveHeatTargetT = 0;
  uint16_t BMS_packTMin = 0;
  uint16_t BMS_packTMax = 0;
  bool BMS_pcsNoFlowRequest = false;
  bool BMS_noFlowRequest = false;
  //0x3C4: PCS_info, or UDS 0xF012 on the PCS (0x628/0x629) - whichever answers first
  uint8_t PCS_partNumber[13] = {0};  //stores raw HEX values for ASCII chars
  bool parsed_PCS_partNumber = false;
  uint16_t PCS_info_buildConfigId = 0;
  uint16_t PCS_info_hardwareId = 0;
  uint16_t PCS_info_componentId = 0;
  uint8_t PCS_info_pcbaId = 0;
  uint8_t PCS_info_assemblyId = 0;
  uint16_t PCS_info_usageId = 0;
  uint16_t PCS_info_subUsageId = 0;
  uint8_t PCS_info_platformType = 0;
  uint32_t PCS_info_appCrc = 0;
  uint32_t PCS_info_cpu2AppCrc = 0;
  uint64_t PCS_info_bootGitHash = 0;
  uint8_t PCS_info_bootUdsProtoVersion = 0;
  uint32_t PCS_info_bootCrc = 0;
  //0x2A4; 676 PCS_thermalStatus
  int16_t PCS_chgPhATemp = 0;
  int16_t PCS_chgPhBTemp = 0;
  int16_t PCS_chgPhCTemp = 0;
  int16_t PCS_dcdcTemp = 0;
  int16_t PCS_ambientTemp = 0;
  //0x2C4; 708 PCS_logging
  uint16_t PCS_logMessageSelect = 0;
  uint16_t PCS_dcdcMaxLvOutputCurrent = 0;
  uint16_t PCS_dcdcCurrentLimit = 0;
  uint16_t PCS_dcdcLvOutputCurrentTempLimit = 0;
  uint16_t PCS_dcdcUnifiedCommand = 0;
  uint16_t PCS_dcdcCLAControllerOutput = 0;
  int16_t PCS_dcdcTankVoltage = 0;
  uint16_t PCS_dcdcTankVoltageTarget = 0;
  uint16_t PCS_dcdcClaCurrentFreq = 0;
  int16_t PCS_dcdcTCommMeasured = 0;
  uint16_t PCS_dcdcShortTimeUs = 0;
  uint16_t PCS_dcdcHalfPeriodUs = 0;
  uint16_t PCS_dcdcIntervalMaxFrequency = 0;
  uint16_t PCS_dcdcIntervalMaxHvBusVolt = 0;
  uint16_t PCS_dcdcIntervalMaxLvBusVolt = 0;
  uint16_t PCS_dcdcIntervalMaxLvOutputCurr = 0;
  uint16_t PCS_dcdcIntervalMinFrequency = 0;
  uint16_t PCS_dcdcIntervalMinHvBusVolt = 0;
  uint16_t PCS_dcdcIntervalMinLvBusVolt = 0;
  uint16_t PCS_dcdcIntervalMinLvOutputCurr = 0;
  uint32_t PCS_dcdc12vSupportLifetimekWh = 0;
  //0x7AA: //1962 HVP_debugMessage:
  uint8_t HVP_debugMessageMultiplexer = 0;
  bool HVP_gpioPassivePyroDepl = false;       //Change to bool
  bool HVP_gpioPyroIsoEn = false;             //Change to bool
  bool HVP_gpioCpFaultIn = false;             //Change to bool
  bool HVP_gpioPackContPowerEn = false;       //Change to bool
  bool HVP_gpioHvCablesOk = false;            //Change to bool
  bool HVP_gpioHvpSelfEnable = false;         //Change to bool
  bool HVP_gpioLed = false;                   //Change to bool
  bool HVP_gpioCrashSignal = false;           //Change to bool
  bool HVP_gpioShuntDataReady = false;        //Change to bool
  bool HVP_gpioFcContPosAux = false;          //Change to bool
  bool HVP_gpioFcContNegAux = false;          //Change to bool
  bool HVP_gpioBmsEout = false;               //Change to bool
  bool HVP_gpioCpFaultOut = false;            //Change to bool
  bool HVP_gpioPyroPor = false;               //Change to bool
  bool HVP_gpioShuntEn = false;               //Change to bool
  bool HVP_gpioHvpVerEn = false;              //Change to bool
  bool HVP_gpioPackCoontPosFlywheel = false;  //Change to bool
  bool HVP_gpioCpLatchEnable = false;         //Change to bool
  bool HVP_gpioPcsEnable = false;             //Change to bool
  bool HVP_gpioPcsDcdcPwmEnable = false;      //Change to bool
  bool HVP_gpioPcsChargePwmEnable = false;    //Change to bool
  bool HVP_gpioFcContPowerEnable = false;     //Change to bool
  bool HVP_gpioHvilEnable = false;            //Change to bool
  bool HVP_gpioSecDrdy = false;               //Change to bool
  uint16_t HVP_hvp1v5Ref = 0;
  int16_t HVP_shuntCurrentDebug = 0;
  bool HVP_packCurrentMia = false;           //Change to bool
  bool HVP_auxCurrentMia = false;            //Change to bool
  bool HVP_currentSenseMia = false;          //Change to bool
  bool HVP_shuntRefVoltageMismatch = false;  //Change to bool
  bool HVP_shuntThermistorMia = false;       //Change to bool
  bool HVP_shuntHwMia = false;               //Change to bool
  uint16_t HVP_info_buildConfigId = 0;
  uint16_t HVP_info_hardwareId = 0;
  uint16_t HVP_info_componentId = 0;
  uint8_t HVP_info_pcbaId = 0;
  uint8_t HVP_info_assemblyId = 0;
  uint16_t HVP_info_usageId = 0;
  uint16_t HVP_info_subUsageId = 0;
  uint8_t HVP_info_platformType = 0;
  uint32_t HVP_info_appCrc = 0;
  uint64_t HVP_info_bootGitHash = 0;
  uint8_t HVP_info_bootUdsProtoVersion = 0;
  uint32_t HVP_info_bootCrc = 0;
  int16_t HVP_dcLinkVoltage = 0;
  int16_t HVP_packVoltage = 0;
  int16_t HVP_fcLinkVoltage = 0;
  uint16_t HVP_packContVoltage = 0;
  int16_t HVP_packNegativeV = 0;
  int16_t HVP_packPositiveV = 0;
  uint16_t HVP_pyroAnalog = 0;
  int16_t HVP_dcLinkNegativeV = 0;
  int16_t HVP_dcLinkPositiveV = 0;
  int16_t HVP_fcLinkNegativeV = 0;
  uint16_t HVP_fcContCoilCurrent = 0;
  uint16_t HVP_fcContVoltage = 0;
  uint16_t HVP_hvilInVoltage = 0;
  uint16_t HVP_hvilOutVoltage = 0;
  int16_t HVP_fcLinkPositiveV = 0;
  uint16_t HVP_packContCoilCurrent = 0;
  uint16_t HVP_battery12V = 0;
  int16_t HVP_shuntRefVoltageDbg = 0;
  int16_t HVP_shuntAuxCurrentDbg = 0;
  int16_t HVP_shuntBarTempDbg = 0;
  int16_t HVP_shuntAsicTempDbg = 0;
  uint8_t HVP_shuntAuxCurrentStatus = 0;
  uint8_t HVP_shuntBarTempStatus = 0;
  uint8_t HVP_shuntAsicTempStatus = 0;
  //0x3aa: HVP_alertMatrix1 Fault codes   // Change to bool
  //We ignore 0x3AA for now, as on later software/firmware this is a muxed frame so values aren't correct.
  /*
  bool battery_WatchdogReset = false;           //Warns if the processor has experienced a reset due to watchdog reset.
  bool battery_PowerLossReset = false;          //Warns if the processor has experienced a reset due to power loss.
  bool battery_SwAssertion = false;             //An internal software assertion has failed.
  bool battery_CrashEvent = false;              //Warns if the crash signal is detected by HVP
  bool battery_OverDchgCurrentFault = false;    //Warns if the pack discharge is above max discharge current limit
  bool battery_OverChargeCurrentFault = false;  //Warns if the pack discharge current is above max charge current limit
  bool battery_OverCurrentFault = false;  //Warns if the pack current (discharge or charge) is above max current limit.
  bool battery_OverTemperatureFault = false;  //A pack module temperature is above maximum temperature limit
  bool battery_OverVoltageFault = false;      //A brick voltage is above maximum voltage limit
  bool battery_UnderVoltageFault = false;     //A brick voltage is below minimum voltage limit
  bool battery_PrimaryBmbMiaFault =
      false;  //Warns if the voltage and temperature readings from primary BMB chain are mia
  bool battery_SecondaryBmbMiaFault =
      false;  //Warns if the voltage and temperature readings from secondary BMB chain are mia
  bool battery_BmbMismatchFault =
      false;  //Warns if the primary and secondary BMB chain readings don't match with each other
  bool battery_BmsHviMiaFault = false;   //Warns if the BMS node is mia on HVS or HVI CAN
  bool battery_CpMiaFault = false;       //Warns if the CP node is mia on HVS CAN
  bool battery_PcsMiaFault = false;      //The PCS node is mia on HVS CAN
  bool battery_BmsFault = false;         //Warns if the BMS ECU has faulted
  bool battery_PcsFault = false;         //Warns if the PCS ECU has faulted
  bool battery_CpFault = false;          //Warns if the CP ECU has faulted
  bool battery_ShuntHwMiaFault = false;  //Warns if the shunt current reading is not available
  bool battery_PyroMiaFault = false;     //Warns if the pyro squib is not connected
  bool battery_hvsMiaFault = false;      //Warns if the pack contactor hw fault
  bool battery_hviMiaFault = false;      //Warns if the FC contactor hw fault
  bool battery_Supply12vFault = false;   //Warns if the low voltage (12V) battery is below minimum voltage threshold
  bool battery_VerSupplyFault = false;   //Warns if the Energy reserve voltage supply is below minimum voltage threshold
  bool battery_HvilFault = false;        //Warn if a High Voltage Inter Lock fault is detected
  bool battery_BmsHvsMiaFault = false;   //Warns if the BMS node is mia on HVS or HVI CAN
  bool battery_PackVoltMismatchFault =
      false;                         //Warns if the pack voltage doesn't match approximately with sum of brick voltages
  bool battery_EnsMiaFault = false;  //Warns if the ENS line is not connected to HVC
  bool battery_PackPosCtrArcFault = false;  //Warns if the HVP detectes series arc at pack contactor
  bool battery_packNegCtrArcFault = false;  //Warns if the HVP detectes series arc at FC contactor
  bool battery_ShuntHwAndBmsMiaFault = false;
  bool battery_fcContHwFault = false;
  bool battery_robinOverVoltageFault = false;
  bool battery_packContHwFault = false;
  bool battery_pyroFuseBlown = false;
  bool battery_pyroFuseFailedToBlow = false;
  bool battery_CpilFault = false;
  bool battery_PackContactorFellOpen = false;
  bool battery_FcContactorFellOpen = false;
  bool battery_packCtrCloseBlocked = false;
  bool battery_fcCtrCloseBlocked = false;
  bool battery_packContactorForceOpen = false;
  bool battery_fcContactorForceOpen = false;
  bool battery_dcLinkOverVoltage = false;
  bool battery_shuntOverTemperature = false;
  bool battery_passivePyroDeploy = false;
  bool battery_logUploadRequest = false;
  bool battery_packCtrCloseFailed = false;
  bool battery_fcCtrCloseFailed = false;
  bool battery_shuntThermistorMia = false;
  */
  //0x320: 800 BMS_alertMatrix
  uint8_t BMS_matrixIndex = 0;  // Changed to bool
  // Alerts below added from tesla-m3-pack-findings (firmware 2019.20.4.2)
  bool BMS_a001_Pack_Config_Mismatch = false;
  bool BMS_a055_SW_HvChain_Model_Fault = false;
  bool BMS_a126_SW_Thermistor_Failure = false;
  bool BMS_a135_HW_BMB_Diagnostics_Failure = false;
  bool BMS_a143_SW_CAC_Change = false;
  bool BMS_a155_SW_Weak_short_impedence = false;
  bool BMS_a173_SW_Charge_Component_Fault = false;
  bool BMS_a178_SW_Uncontrolled_Regen_PwrB = false;
  bool BMS_a061_robinBrickOverVoltage = false;
  bool BMS_a062_SW_BrickV_Imbalance = false;
  bool BMS_a063_SW_ChargePort_Fault = false;
  bool BMS_a064_SW_SOC_Imbalance = false;
  bool BMS_a127_SW_shunt_SNA = false;
  bool BMS_a128_SW_shunt_MIA = false;
  bool BMS_a069_SW_Low_Power = false;
  bool BMS_a130_IO_CAN_Error = false;
  bool BMS_a071_SW_SM_TransCon_Not_Met = false;
  bool BMS_a132_HW_BMB_OTP_Uncorrctbl = false;
  bool BMS_a134_SW_Delayed_Ctr_Off = false;
  bool BMS_a075_SW_Chg_Disable_Failure = false;
  bool BMS_a076_SW_Dch_While_Charging = false;
  bool BMS_a017_SW_Brick_OV = false;
  bool BMS_a018_SW_Brick_UV = false;
  bool BMS_a019_SW_Module_OT = false;
  bool BMS_a021_SW_Dr_Limits_Regulation = false;
  bool BMS_a022_SW_Over_Current = false;
  bool BMS_a023_SW_Stack_OV = false;
  bool BMS_a024_SW_Islanded_Brick = false;
  bool BMS_a025_SW_PwrBalance_Anomaly = false;
  bool BMS_a026_SW_HFCurrent_Anomaly = false;
  bool BMS_a087_SW_Feim_Test_Blocked = false;
  bool BMS_a088_SW_VcFront_MIA_InDrive = false;
  bool BMS_a089_SW_VcFront_MIA = false;
  bool BMS_a090_SW_Gateway_MIA = false;
  bool BMS_a091_SW_ChargePort_MIA = false;
  bool BMS_a092_SW_ChargePort_Mia_On_Hv = false;
  bool BMS_a034_SW_Passive_Isolation = false;
  bool BMS_a035_SW_Isolation = false;
  bool BMS_a036_SW_HvpHvilFault = false;
  bool BMS_a037_SW_Flood_Port_Open = false;
  bool BMS_a158_SW_HVP_HVI_Comms = false;
  bool BMS_a039_SW_DC_Link_Over_Voltage = false;
  bool BMS_a041_SW_Power_On_Reset = false;
  bool BMS_a042_SW_MPU_Error = false;
  bool BMS_a043_SW_Watch_Dog_Reset = false;
  bool BMS_a044_SW_Assertion = false;
  bool BMS_a045_SW_Exception = false;
  bool BMS_a046_SW_Task_Stack_Usage = false;
  bool BMS_a047_SW_Task_Stack_Overflow = false;
  bool BMS_a048_SW_Log_Upload_Request = false;
  bool BMS_a169_SW_FC_Pack_Weld = false;
  bool BMS_a050_SW_Brick_Voltage_MIA = false;
  bool BMS_a051_SW_HVC_Vref_Bad = false;
  bool BMS_a052_SW_PCS_MIA = false;
  bool BMS_a053_SW_ThermalModel_Sanity = false;
  bool BMS_a054_SW_Ver_Supply_Fault = false;
  bool BMS_a176_SW_GracefulPowerOff = false;
  bool BMS_a059_SW_Pack_Voltage_Sensing = false;
  bool BMS_a060_SW_Leakage_Test_Failure = false;
  bool BMS_a077_SW_Charger_Regulation = false;
  bool BMS_a081_SW_Ctr_Close_Blocked = false;
  bool BMS_a082_SW_Ctr_Force_Open = false;
  bool BMS_a083_SW_Ctr_Close_Failure = false;
  bool BMS_a084_SW_Sleep_Wake_Aborted = false;
  bool BMS_a094_SW_Drive_Inverter_MIA = false;
  bool BMS_a099_SW_BMB_Communication = false;
  bool BMS_a105_SW_One_Module_Tsense = false;
  bool BMS_a106_SW_All_Module_Tsense = false;
  bool BMS_a107_SW_Stack_Voltage_MIA = false;
  bool BMS_a121_SW_NVRAM_Config_Error = false;
  bool BMS_a122_SW_BMS_Therm_Irrational = false;
  bool BMS_a123_SW_Internal_Isolation = false;
  bool BMS_a129_SW_VSH_Failure = false;
  bool BMS_a131_Bleed_FET_Failure = false;
  bool BMS_a136_SW_Module_OT_Warning = false;
  bool BMS_a137_SW_Brick_UV_Warning = false;
  bool BMS_a138_SW_Brick_OV_Warning = false;
  bool BMS_a139_SW_DC_Link_V_Irrational = false;
  bool BMS_a141_SW_BMB_Status_Warning = false;
  bool BMS_a144_Hvp_Config_Mismatch = false;
  bool BMS_a145_SW_SOC_Change = false;
  bool BMS_a146_SW_Brick_Overdischarged = false;
  bool BMS_a149_SW_Missing_Config_Block = false;
  bool BMS_a151_SW_external_isolation = false;
  bool BMS_a156_SW_BMB_Vref_bad = false;
  bool BMS_a157_SW_HVP_HVS_Comms = false;
  bool BMS_a159_SW_HVP_ECU_Error = false;
  bool BMS_a161_SW_DI_Open_Request = false;
  bool BMS_a162_SW_No_Power_For_Support = false;
  bool BMS_a163_SW_Contactor_Mismatch = false;
  bool BMS_a164_SW_Uncontrolled_Regen = false;
  bool BMS_a165_SW_Pack_Partial_Weld = false;
  bool BMS_a166_SW_Pack_Full_Weld = false;
  bool BMS_a167_SW_FC_Partial_Weld = false;
  bool BMS_a168_SW_FC_Full_Weld = false;
  bool BMS_a170_SW_Limp_Mode = false;
  bool BMS_a171_SW_Stack_Voltage_Sense = false;
  bool BMS_a174_SW_Charge_Failure = false;
  bool BMS_a179_SW_Hvp_12V_Fault = false;
  bool BMS_a180_SW_ECU_reset_blocked = false;
  //0x3A4: PCS_alertMatrix — Tesla Model 3/Y, mapped from tesla-m3-pack-findings (fw 2019.20.4.2)
  // NOTE: findings dictionary (libQtCarCANData.so) lists alt names for a032=excessiveGridTransientsDetected,
  //       a047=bootloaderCrcMismatch, a048=softwareAssertion, a084=vDropFastInParasiticDiodeRegion.
  //       Names below follow the extracted bit-map table (dtc_matrices.json).
  bool PCS_a001_chgHwInputOc = false;
  bool PCS_a002_chgHwOutputOc = false;
  bool PCS_a003_chgHwInputOv = false;
  bool PCS_a004_chgHwIntBusOv = false;
  bool PCS_a005_chgOutputOv = false;
  bool PCS_a006_chgPrechargeFailedScr = false;
  bool PCS_a007_chgPhaseTempHot = false;
  bool PCS_a008_chgPhaseOverTemp = false;
  bool PCS_a009_chgPfcCurrentRegulation = false;
  bool PCS_a010_chgIntBusVRegulation = false;
  bool PCS_a011_chgLlcCurrentRegulation = false;
  bool PCS_a012_chgPfcIBandTracerFault = false;
  bool PCS_a013_chgPrechargeFailedBoost = false;
  bool PCS_a014_chgTempRationality = false;
  bool PCS_a015_chg12vUv = false;
  bool PCS_a016_chgAllPhasesFaulted = false;
  bool PCS_a017_chgWallPowerRemoval = false;
  bool PCS_a018_chgUnknownGridConfig = false;
  bool PCS_a019_acChargePowerLimited = false;
  bool PCS_a020_chgEnableLineMismatch = false;
  bool PCS_a021_hvpMia = false;
  bool PCS_a022_bmsMia = false;
  bool PCS_a023_cpMia = false;
  bool PCS_a024_vcfrontMia = false;
  bool PCS_a025_cpu2Malfunction = false;
  bool PCS_a026_watchdogAlarmed = false;
  bool PCS_a027_chgInsufficientCooling = false;
  bool PCS_a028_chgOutputUv = false;
  bool PCS_a029_chgPowerRationality = false;
  bool PCS_a030_canRationality = false;
  bool PCS_a031_uiMia = false;
  bool PCS_a032_gtwMia = false;
  bool PCS_a033_hvBusUv = false;
  bool PCS_a034_hvBusOv = false;
  bool PCS_a035_lvBusUv = false;
  bool PCS_a036_lvBusOv = false;
  bool PCS_a037_resonantTankOc = false;
  bool PCS_a038_claFaulted = false;
  bool PCS_a039_sdModuleClkFault = false;
  bool PCS_a040_dcdcMaxPowerReached = false;
  bool PCS_a041_dcdcOverTemp = false;
  bool PCS_a042_dcdcEnableLineMismatch = false;
  bool PCS_a043_hvBusPrechargeFailure = false;
  bool PCS_a044_12vSupportRegulation = false;
  bool PCS_a045_hvBusLowImpedance = false;
  bool PCS_a046_hvBusHighImpedence = false;
  bool PCS_a047_lvBusLowImpedance = false;
  bool PCS_a048_lvBusHighImpedance = false;
  bool PCS_a049_dcdcTempRationality = false;
  bool PCS_a050_dcdc12VsupportFaulted = false;
  bool PCS_a051_chgIntBusUv = false;
  bool PCS_a052_acVoltageNotPresent = false;
  bool PCS_a053_chgInputVDropHigh = false;
  bool PCS_a054_chgInputVDropTooHigh = false;
  bool PCS_a055_chgLineImedanceHigh = false;
  bool PCS_a056_chgLineImedanceTooHigh = false;
  bool PCS_a057_chgInputOverFreq = false;
  bool PCS_a058_chgInputUnderFreq = false;
  bool PCS_a059_chgInputOvRms = false;
  bool PCS_a060_chgInputOvPeak = false;
  bool PCS_a061_chgVLineRationality = false;
  bool PCS_a062_chgILineRationality = false;
  bool PCS_a063_chgVOutRationality = false;
  bool PCS_a064_chgIOutRationality = false;
  bool PCS_a065_chgPllNotLocked = false;
  bool PCS_a066_dcdcHvRationality = false;
  bool PCS_a067_dcdcLvRationality = false;
  bool PCS_a068_dcdcTankvRationality = false;
  bool PCS_a069_chgPfcLineDidt = false;
  bool PCS_a070_chgPfcLineDvdt = false;
  bool PCS_a071_chgPfcILoopRationality = false;
  bool PCS_a072_cpu2ClaStopped = false;
  bool PCS_a073_unexpectedAcInputVoltage = false;
  bool PCS_a074_hvBusDischargeFailure = false;
  bool PCS_a075_hvBusDischargeTimeout = false;
  bool PCS_a076_dcdcEnDeassertedErr = false;
  bool PCS_a077_microGridEnergyLow = false;
  bool PCS_a078_chgStopDcdcTooHot = false;
  bool PCS_a079_eepromOperationError = false;
  bool PCS_a080_damagedPhaseDetected = false;
  bool PCS_a081_dcdcPchgTimeout = false;
  bool PCS_a082_dcdcPchgUnsafeDiVoltage = false;
  bool PCS_a083_triggerOdin = false;
  bool PCS_a084_dcdcPchgStartVoltages = false;
  bool PCS_a085_dcdcFetsNotSwitching = false;
  bool PCS_a086_dcdcInsufficientCooling = false;
  bool PCS_a087_nvramRecordStatusError = false;
  bool PCS_a088_pchgParameters = false;
  bool PCS_a089_hvBusDischargeIrrational = false;
  bool PCS_a090_expectedAcVoltageSourceMissing = false;
  bool PCS_a091_chgIntBusRationality = false;
  bool PCS_a092_chgPowerLimitedByBusRipple = false;
  bool PCS_a093_powerRailRationality = false;
  bool PCS_a094_pcsDcdcNeedService = false;

  //0x31E: CP_alertMatrix — Tesla Model 3/Y, mapped from tesla-m3-pack-findings (fw 2019.20.4.2)
  bool CP_a001_canRx = false;
  bool CP_a002_canTx = false;
  bool CP_a003_canError = false;
  bool CP_a004_proximityRationality = false;
  bool CP_a005_gbdcLiveDisconnect = false;
  bool CP_a006_lostCommsBMS = false;
  bool CP_a007_watchdog = false;
  bool CP_a008_memoryError = false;
  bool CP_a009_coverOpen = false;
  bool CP_a010_pilotRationality = false;
  bool CP_a011_eeprom = false;
  bool CP_a012_ledDriver = false;
  bool CP_a013_lostCommsGTW = false;
  bool CP_a014_lostCommsCHG = false;
  bool CP_a015_apsVov = false;
  bool CP_a016_apsVuv = false;
  bool CP_a017_fiveVov = false;
  bool CP_a018_fiveVuv = false;
  bool CP_a019_threeVov = false;
  bool CP_a020_threeVuv = false;
  bool CP_a021_zeroVov = false;
  bool CP_a022_zeroVuv = false;
  bool CP_a023_gbdcSessionFailed = false;
  bool CP_a024_ledsUC = false;
  bool CP_a025_ledsOC = false;
  bool CP_a026_networkManagement = false;
  bool CP_a027_doorSensorOutOfSpec = false;
  bool CP_a028_insertEnableMismatch = false;
  bool CP_a029_doorClosedProxPilot = false;
  bool CP_a030_busOff = false;
  bool CP_a031_doorClosedCommandedOpen = false;
  bool CP_a032_doorOpenExpectedClosed = false;
  bool CP_a033_spiOpen = false;
  bool CP_a034_calibrationIncomplete = false;
  bool CP_a035_latchMovement_1 = false;
  bool CP_a036_latchNotDisengaged = false;
  bool CP_a037_latchNotEngaged = false;
  bool CP_a038_latchNotBlocking = false;
  bool CP_a039_latchMovement_2 = false;
  bool CP_a040_doNotUse = false;
  bool CP_a041_doorSensorUnplugged = false;
  bool CP_a042_doorAssemblyBroken = false;
  bool CP_a043_doorPotIrrational = false;
  bool CP_a044_lostCommsHVP = false;
  bool CP_a045_lostCommsVCSEC = false;
  bool CP_a046_lostCommsEVSE = false;
  bool CP_a047_lostCommsVCFRONT = false;
  bool CP_a048_lostCommsUI = false;
  bool CP_a049_multipleCablesDetected = false;
  bool CP_a050_latchNotConnected = false;
  bool CP_a051_doorInductiveSensorMIA = false;
  bool CP_a052_evseNotSupported = false;
  bool CP_a053_proxLatchedNoPilot = false;
  bool CP_a054_cableNotSecured = false;
  bool CP_a055_chargeStoppedNoPilot = false;
  bool CP_a056_proxDisconnected = false;
  bool CP_a057_evseFaulted = false;
  bool CP_a058_acChargingBlocked = false;
  bool CP_a059_swcanError = false;
  bool CP_a060_lostCommsPCS = false;
  bool CP_a061_uhfReceiverMIA = false;
  bool CP_a062_scOutOfService = false;
  bool CP_a063_scUpdateInProgress = false;
  bool CP_a064_superchargingBlocked = false;
  bool CP_a065_selfTestFailed = false;
  bool CP_a066_proxLatchedIdlePilot = false;
  bool CP_a067_gbdcConnFault = false;
  bool CP_a068_doorSensorMismatch = false;
  bool CP_a069_doorInductiveSensorError = false;
  bool CP_a070_doorInductiveSensorReset = false;
  bool CP_a071_exiDecodeFailure = false;
  bool CP_a072_v2gEvccTimeout = false;
  bool CP_a073_iecComboShutdown = false;
  bool CP_a074_failedToEstablishV2gComm = false;
  bool CP_a075_v2gCommsFailure = false;
  bool CP_a076_LDC1612errorWatchdog = false;
  bool CP_a077_invalidMacAddress = false;
  bool CP_a078_latchNotDisengagedCold = false;
  bool CP_a079_cableNotSecuredCold = false;
  bool CP_a080_taskStackOverflow = false;
  bool CP_a081_swException = false;
  bool CP_a082_powerOnReset = false;
  bool CP_a083_watchdogTraceData = false;
  bool CP_a084_proximityPeDisconnected = false;
  bool CP_a085_dcPinTempFaulted = false;
  bool CP_a086_dcPinTempIrrational = false;
  bool CP_a087_dcTempModelFault = false;
  bool CP_a088_dcTempModelDeviation = false;
  bool CP_a089_plcConfigMismatch = false;
  bool CP_a090_ccsEvseLowIso = false;
  bool CP_a091_wrongSuperchargerHandle = false;
  bool CP_a092_modemAppLoadFailed = false;
  bool CP_a093_modemLoadedWithReset = false;
  bool CP_a094_inductiveResetSuccessful = false;
  bool CP_a095_thermalDcLimitActive = false;
  bool CP_a096_pilotWake = false;
};

#endif
