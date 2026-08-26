#include "TESLA-BATTERY.h"
#include <cstdio>   //snprintf for the HVC backup JSON builder
#include <cstring>  //For unit test
#include "../battery/BATTERIES.h"
#include "../communication/can/comm_can.h"
#include "../communication/contactorcontrol/comm_contactorcontrol.h"
#include "../datalayer/datalayer.h"
#include "../datalayer/datalayer_extended.h"  //For Advanced Battery Insights webpage
#include "../devboard/utils/events.h"
#include "../devboard/utils/logging.h"

/* Credits: */
/* Some of the original CAN frame parsing code below comes from Per Carlen's bms_comms_tesla_model3.py (https://gitlab.com/pelle8/batt2gen24/) */
/* Most of the additional CAN frame parsing/information/display comes from Josiah Higgs (https://github.com/josiahhiggs/) */

inline const char* getContactorText(int index) {
  switch (index) {
    case 0:
      return "UNKNOWN(0)";
    case 1:
      return "OPEN";
    case 2:
      return "CLOSING";
    case 3:
      return "BLOCKED";
    case 4:
      return "OPENING";
    case 5:
      return "CLOSED";
    case 6:
      return "UNKNOWN(6)";
    case 7:
      return "WELDED";
    case 8:
      return "POS_CL";
    case 9:
      return "NEG_CL";
    case 10:
      return "UNKNOWN(10)";
    case 11:
      return "UNKNOWN(11)";
    case 12:
      return "UNKNOWN(12)";
    default:
      return "UNKNOWN";
  }
}

inline const char* getContactorState(int index) {
  switch (index) {
    case 0:
      return "SNA";
    case 1:
      return "OPEN";
    case 2:
      return "PRECHARGE";
    case 3:
      return "BLOCKED";
    case 4:
      return "PULLED_IN";
    case 5:
      return "OPENING";
    case 6:
      return "ECONOMIZED";
    case 7:
      return "WELDED";
    case 8:
      return "UNKNOWN(8)";
    case 9:
      return "UNKNOWN(9)";
    case 10:
      return "UNKNOWN(10)";
    case 11:
      return "UNKNOWN(11)";
    default:
      return "UNKNOWN";
  }
}

inline const char* getHvilStatusState(int index) {
  switch (index) {
    case 0:
      return "NOT OK";
    case 1:
      return "STATUS_OK";
    case 2:
      return "CURRENT_SOURCE_FAULT";
    case 3:
      return "INTERNAL_OPEN_FAULT";
    case 4:
      return "VEHICLE_OPEN_FAULT";
    case 5:
      return "PENTHOUSE_LID_OPEN_FAULT";
    case 6:
      return "UNKNOWN_LOCATION_OPEN_FAULT";
    case 7:
      return "VEHICLE_NODE_FAULT";
    case 8:
      return "NO_12V_SUPPLY";
    case 9:
      return "VEHICLE_OR_PENTHOUSE_LID_OPENFAULT";
    case 10:
      return "UNKNOWN(10)";
    case 11:
      return "UNKNOWN(11)";
    case 12:
      return "UNKNOWN(12)";
    case 13:
      return "UNKNOWN(13)";
    case 14:
      return "UNKNOWN(14)";
    case 15:
      return "UNKNOWN(15)";
    default:
      return "UNKNOWN";
  }
}

inline const char* getBMSContactorState(int index) {
  switch (index) {
    case 0:
      return "SNA";
    case 1:
      return "OPEN";
    case 2:
      return "OPENING";
    case 3:
      return "CLOSING";
    case 4:
      return "CLOSED";
    case 5:
      return "WELDED";
    case 6:
      return "BLOCKED";
    default:
      return "UNKNOWN";
  }
}

inline const char* getNoYes(bool value) {
  return value ? "Yes" : "No";
}

// Clamp DLC to 0-8 bytes for classic CAN
inline int getDataLen(uint8_t dlc) {
  return std::min<int>(dlc, 8);
}

// Fast bit-field writer: writes 'bitLen' bits of 'value' starting at 'startBit'
inline void setBitField(uint8_t* data, int bytes, int startBit, int bitLen, uint64_t value) {
  int bit = startBit;
  for (int i = 0; i < bitLen && bit < bytes * 8; ++i, ++bit) {
    uint8_t* p = data + (bit >> 3);
    uint8_t m = uint8_t(1u << (bit & 7));
    *p = (*p & ~m) | (uint8_t((value >> i) & 1) << (bit & 7));
  }
}

// Shared core for the Tesla counter+checksum frames: write `counter` into the counter field, then
// recompute the additive 8-bit checksum (low byte of ID + payload with the checksum bits zeroed +
// high nibble of the ID's top byte). The two public wrappers differ only in how `counter` is chosen.
static void applyCounterChecksum(CAN_frame& f, uint64_t counter, int ctrStartBit, int ctrBitLength,
                                 int csumStartBit, int csumBitLength) {
  int bytes = getDataLen(f.DLC);
  auto data = f.data.u8;

  // Pack payload into a 64-bit word and write the counter field
  uint64_t w = 0;
  for (int i = 0; i < bytes; ++i) {
    w |= uint64_t(data[i]) << (8 * i);
  }
  uint64_t mask = (uint64_t(1) << ctrBitLength) - 1;
  w = (w & ~(mask << ctrStartBit)) | ((counter & mask) << ctrStartBit);
  for (int i = 0; i < bytes; ++i) {
    data[i] = uint8_t((w >> (8 * i)) & 0xFF);
  }

  // Copy the payload with the checksum bits zeroed, then sum
  uint8_t buf[8];
  for (int i = 0; i < bytes; ++i) {
    buf[i] = data[i];
  }
  for (int bit = csumStartBit; bit < csumStartBit + csumBitLength; ++bit) {
    int b = bit >> 3;
    if (b >= bytes)
      break;
    buf[b] &= uint8_t(~(1u << (bit & 7)));
  }
  uint8_t sum = uint8_t(f.ID & 0xFF);
  for (int i = 0; i < bytes; ++i) {
    sum = uint8_t(sum + buf[i]);
  }
  uint8_t checksum = uint8_t(sum + uint8_t((f.ID >> 8) & 0xF));  // + high nibble of ID top byte
  setBitField(data, bytes, csumStartBit, csumBitLength, checksum);
}

// Set an externally-supplied counter value (e.g. a shared mux counter), then recompute the checksum.
void generateMuxFrameCounterChecksum(CAN_frame& f, uint8_t frameCounter, int ctrStartBit, int ctrBitLength,
                                     int csumStartBit, int csumBitLength) {
  applyCounterChecksum(f, frameCounter, ctrStartBit, ctrBitLength, csumStartBit, csumBitLength);
}

// Increment the frame's own counter field by +1 (modulo its width), then recompute the checksum.
void generateFrameCounterChecksum(CAN_frame& f, int ctrStartBit, int ctrBitLength, int csumStartBit,
                                  int csumBitLength) {
  int bytes = getDataLen(f.DLC);
  uint64_t w = 0;
  for (int i = 0; i < bytes; ++i) {
    w |= uint64_t(f.data.u8[i]) << (8 * i);
  }
  uint64_t mask = (uint64_t(1) << ctrBitLength) - 1;
  uint64_t ctr = ((w >> ctrStartBit) & mask) + 1;
  applyCounterChecksum(f, ctr, ctrStartBit, ctrBitLength, csumStartBit, csumBitLength);
}

// Function to extract raw bits/values from a given CAN frame signal
inline uint64_t extract_signal_value(const uint8_t* data, uint32_t start_bit, uint32_t bit_length) {
  //
  // Usage: uint8_t bms_state = static_cast<uint8_t>(extract_signal_value(rx_frame.data.u8, 32, 4));
  //
  // Calculate the starting byte and bit offset
  uint32_t byte_index = start_bit / 8;
  uint32_t bit_offset = start_bit % 8;

  // Read up to 8 bytes starting from byte_index (need enough to cover bit_length + bit_offset)
  uint64_t raw = 0;
  for (int i = 0; i < 8 && (byte_index + i) < 64; ++i) {
    raw |= (uint64_t)data[byte_index + i] << (8 * i);
  }

  // Shift and mask
  raw >>= bit_offset;
  if (bit_length == 64)
    return raw;
  return raw & ((1ULL << bit_length) - 1);
}

// Function to write a value to a given CAN frame signal
void write_signal_value(CAN_frame* frame, uint16_t start_bit, uint8_t bit_length, int64_t value, bool is_signed) {
  if (bit_length == 0 || bit_length > 64 || frame == nullptr)
    return;

  uint64_t uvalue;

  if (is_signed) {
    int64_t min_val = -(1LL << (bit_length - 1));
    int64_t max_val = (1LL << (bit_length - 1)) - 1;

    // Clamp to valid range
    if (value < min_val)
      value = min_val;
    if (value > max_val)
      value = max_val;

    // Two's complement encoding
    uvalue = static_cast<uint64_t>(value) & ((1ULL << bit_length) - 1);
  } else {
    uvalue = static_cast<uint64_t>(value) & ((1ULL << bit_length) - 1);
  }

  // Write value into frame->data.u8 using little-endian bit layout
  for (uint8_t i = 0; i < bit_length; ++i) {
    uint8_t bit_val = (uvalue >> i) & 1;
    uint16_t bit_pos = start_bit + i;

    uint8_t byte_index = bit_pos / 8;
    uint8_t bit_index = bit_pos % 8;

    if (byte_index >= frame->DLC)
      continue;  // Prevent overrun

    if (bit_val)
      frame->data.u8[byte_index] |= (1 << bit_index);
    else
      frame->data.u8[byte_index] &= ~(1 << bit_index);
  }
}

void generateTESLA_229(CAN_frame& f) {
  static const uint8_t checksumLookup[16] = {0x46, 0x44, 0x52, 0x6D, 0x43, 0x41, 0xDD, 0xF9,
                                             0x4C, 0xA5, 0xF6, 0x8C, 0x49, 0x2F, 0x31, 0x3B};

  // Safety, only run if this is the right ID
  if (f.ID != 0x229)
    return;

  const int ctrStartBit = 8;
  const int ctrBitLength = 4;
  const int csumStartBit = 0;
  const int csumBitLength = 8;

  int bytes = getDataLen(f.DLC);
  auto data = f.data.u8;

  // Pack the first few bytes into a word
  uint64_t w = 0;
  for (int i = 0; i < bytes; ++i) {
    w |= uint64_t(data[i]) << (8 * i);
  }

  // Extract current counter
  uint64_t mask = (uint64_t(1) << ctrBitLength) - 1;
  uint8_t ctr = (w >> ctrStartBit) & mask;

  // Increment counter mod 16
  ctr = (ctr + 1) & 0xF;

  // Write updated counter back
  w = (w & ~(mask << ctrStartBit)) | (uint64_t(ctr) << ctrStartBit);
  for (int i = 0; i < bytes; ++i) {
    data[i] = uint8_t((w >> (8 * i)) & 0xFF);
  }

  // Look up and insert checksum
  uint8_t checksum = checksumLookup[ctr];
  setBitField(data, bytes, csumStartBit, csumBitLength, checksum);
}

void generateTESLA_213(CAN_frame& f) {
  static uint8_t counter = 0;

  // Increment counter (wrap at 16)
  counter = (counter + 1) & 0xF;

  // Safety, only modify if ID is 0x213 and DLC is at least 2
  if (f.ID != 0x213 || f.DLC < 2)
    return;

  // Byte 0: counter in high nibble
  uint8_t value = counter << 4;

  // Byte 1: checksum = value + 0x15
  uint8_t checksum = (value + 0x15) & 0xFF;

  f.data.u8[0] = value;
  f.data.u8[1] = checksum;
}

// Function to check if a year is a leap year
bool isLeapYear(int year) {
  if ((year % 4 == 0 && year % 100 != 0) || (year % 400 == 0)) {
    return true;
  }
  return false;
}

// Function to convert year and day of year (i.e. Julian date) into human readable date
char* dayOfYearToDate(int year, int dayOfYear) {

  // Arrays to hold the number of days in each month for standard/leap years
  int daysInMonthStandard[] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
  int daysInMonthLeap[] = {31, 29, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};

  // Select the appropriate array for the given year
  int* daysInMonth = isLeapYear(year) ? daysInMonthLeap : daysInMonthStandard;
  int month = 0;

  // Find the month and the day within the month
  while (month < 12 && dayOfYear > daysInMonth[month]) {
    dayOfYear -= daysInMonth[month];
    month++;
  }

  // Ensure month is in valid range [0, 11]
  if (month >= 12) {
    month = 11;
    dayOfYear = daysInMonth[11];  // Set to last day of December
  }

  // Ensure day is in valid range [1, daysInMonth[month]]
  if (dayOfYear < 1) {
    dayOfYear = 1;
  } else if (dayOfYear > daysInMonth[month]) {
    dayOfYear = daysInMonth[month];
  }

  static char dateString[11];  // For "YYYY-MM-DD\0"

  // Clamp values to ensure they fit in the expected number of digits
  int safeYear = year % 10000;
  if (safeYear < 0)
    safeYear = 0;
  if (safeYear > 9999)
    safeYear = 9999;

  int safeMonth = month + 1;
  if (safeMonth < 1)
    safeMonth = 1;
  if (safeMonth > 12)
    safeMonth = 12;

  int safeDay = dayOfYear;
  if (safeDay < 1)
    safeDay = 1;
  if (safeDay > 31)
    safeDay = 31;

  // Format the date string in "YYYY-MM-DD" format
  snprintf(dateString, sizeof(dateString), "%04d-%02d-%02d", safeYear, safeMonth, safeDay);
  return dateString;
}

void TeslaBattery::
    update_values() {  //This function maps all the values fetched via CAN to the correct parameters used for modbus
  //After values are mapped, we perform some safety checks, and do some serial printouts

  if (datalayer_battery->settings.user_requests_tesla_drive_mode) {
    datalayer_battery->settings.user_requests_tesla_drive_mode = false;
    {
      // An explicit choice outranks the backup's ACCESSORY hold; forget the hold rather than undoing
      // the user's mode when the read finishes.
      hvc_bk_accessory_held = false;
      balance_at_rest_enabled = false;  //a manual choice takes over from the experimental auto mode
      accessory_mode_forced = false;
      power_zeroed = false;  //DRIVE removes the power limit
    }
  }
  if (datalayer_battery->settings.user_requests_tesla_accessory_mode) {
    datalayer_battery->settings.user_requests_tesla_accessory_mode = false;
    {
      hvc_bk_accessory_held = false;
      balance_at_rest_enabled = false;
      accessory_mode_forced = true;
      power_zeroed = true;  //ACCESSORY button parks the pack: no charge or discharge
    }
  }
  if (datalayer_battery->settings.user_requests_tesla_toggle_balance_at_rest) {
    datalayer_battery->settings.user_requests_tesla_toggle_balance_at_rest = false;
    {
      balance_at_rest_enabled = !balance_at_rest_enabled;
      power_zeroed = false;  //auto mode imposes no power limits
      rest_below_since_ms = 0;
    }
  }

  //Experimental: switch vehicleState by load only (no power limits). ANY charge current (positive)
  //forces DRIVE instantly so a full pack cannot keep charging in ACCESSORY; discharge forces DRIVE
  //once it exceeds ~1 A. Only truly at rest for the dwell does it drop to ACCESSORY so the pack can
  //balance. battery_amps > 0 is charge, < 0 is discharge (deci-amps).
  if (balance_at_rest_enabled) {
    int32_t discharge_dA = (battery_amps < 0) ? -battery_amps : 0;
    if (battery_amps > 0 || discharge_dA > TESLA_REST_THRESHOLD_dA) {
      accessory_mode_forced = false;
      rest_below_since_ms = 0;
    } else {
      if (rest_below_since_ms == 0) {
        rest_below_since_ms = millis();
      }
      if ((millis() - rest_below_since_ms) >= TESLA_REST_DWELL_MS) {
        accessory_mode_forced = true;
      }
    }
  }

  datalayer_battery->status.soh_pptt = 9900;  //Tesla batteries do not send a SOH% value on bus. Hardcode to 99%

  //User-selectable SOC source. The 0x292 signals are 0.1%/bit (x10 -> 0-10000 pptt); BMS_userFacingSoe
  //(0x352 mux2, 2024.20+ pack firmware) is 0.05%/bit (x5). If userFacingSoe is selected but the pack
  //never sends it (older firmware: stays 0) or reports SNA (2047), fall back to BMS_socUI so real_soc
  //cannot stick at 0 on a misconfigured source.
  switch (user_selected_tesla_soc_source) {
    case 1:  //battery_soc_min (0x292 BMS_socMin)
      datalayer_battery->status.real_soc = (battery_soc_min * 10);
      break;
    case 2:  //battery_soc_avg (0x292 BMS_socAvg)
      datalayer_battery->status.real_soc = (battery_soc_ave * 10);
      break;
    case 3:  //battery_soc_max (0x292 BMS_socMax)
      datalayer_battery->status.real_soc = (battery_soc_max * 10);
      break;
    case 4:  //battery_userFacingSoe (0x352 BMS_userFacingSoe)
      if (battery_userFacingSoe > 0 && battery_userFacingSoe != 2047) {
        datalayer_battery->status.real_soc = (battery_userFacingSoe * 5);
      } else {
        datalayer_battery->status.real_soc = (battery_soc_ui * 10);
      }
      break;
    default:  //battery_soc_ui (0x292 BMS_socUI)
      datalayer_battery->status.real_soc = (battery_soc_ui * 10);  //increase SOC range from 0-100.0 -> 100.00
      break;
  }

  datalayer_battery->status.voltage_dV = battery_volts;

  datalayer_battery->status.current_dA = battery_amps;  //13.0A

  //Calculate the remaining Wh amount from SOC% and max Wh value.
  datalayer_battery->status.remaining_capacity_Wh = static_cast<uint32_t>(
      (static_cast<double>(datalayer_battery->status.real_soc) / 10000) * datalayer_battery->info.total_capacity_Wh);

  // Define the allowed discharge power
  datalayer_battery->status.max_discharge_power_W = (battery_max_discharge_current * (battery_volts / 10));
  // Cap the allowed discharge power if higher than the maximum discharge power allowed
  if (datalayer_battery->status.max_discharge_power_W > datalayer_battery->status.override_discharge_power_W) {
    datalayer_battery->status.max_discharge_power_W = datalayer_battery->status.override_discharge_power_W;
  }

  //The allowed charge power behaves strangely. We instead estimate this value
  //The inverter setting will ramp down this value based on SOC%
  datalayer_battery->status.max_charge_power_W = datalayer_battery->status.override_charge_power_W;

  //The ACCESSORY button parks the pack with no power allowed until DRIVE is pressed.
  if (power_zeroed) {
    datalayer_battery->status.max_discharge_power_W = 0;
    datalayer_battery->status.max_charge_power_W = 0;
  }

  datalayer_battery->status.temperature_min_dC = battery_min_temp;

  datalayer_battery->status.temperature_max_dC = battery_max_temp;

  datalayer_battery->status.cell_max_voltage_mV = battery_cell_max_v;

  datalayer_battery->status.cell_min_voltage_mV = battery_cell_min_v;

  /* Value mapping is completed. Start to check all safeties */

  //12V battery too low for contactor operation. Inform user via Event
  // Only monitor 12V health while contactors are OPEN to avoid false triggers during energization
  if (HVP_packNegativeV == 0 && HVP_packPositiveV == 0) {

    if (HVP_battery12V > 0) {
      // Scale is 0.1, so HVP_battery12V * 0.1 is the physical voltage
      if ((HVP_battery12V * 0.1) < 11.7) {
        set_event(EVENT_12V_LOW, 0);
      } else {
        clear_event(EVENT_12V_LOW);
      }
    }
  } else {
    // Optional: If contactors are closed, we might want to clear the event
    // or simply stop updating it to prevent a latching error.
    clear_event(EVENT_12V_LOW);
  }
  //INTERNAL_OPEN_FAULT - Someone disconnected a high voltage cable while battery was in use
  if (battery_hvil_status == 3) {
    set_event(EVENT_INTERNAL_OPEN_FAULT, 0);
  } else {
    clear_event(EVENT_INTERNAL_OPEN_FAULT);
  }
  //Voltage between 0.5-5.0V, pyrofuse most likely blown
  if (datalayer_battery->status.voltage_dV >= 5 && datalayer_battery->status.voltage_dV <= 50) {
    set_event(EVENT_BATTERY_FUSE, 0, battery_index);
  } else {
    clear_event(EVENT_BATTERY_FUSE, battery_index);
  }
  // Raise any Tesla BMS events in BE
  // Events: Informational
  if (BMS_a145_SW_SOC_Change) {  // BMS has newly recalibrated pack SOC
    set_event_latched(EVENT_BATTERY_SOC_RECALIBRATION, 0,
                      battery_index);  // Latcched as BMS_a145 can be active for a while
  } else if (!BMS_a145_SW_SOC_Change) {
    clear_event(EVENT_BATTERY_SOC_RECALIBRATION, battery_index);
  }
  // Events: Warning
  if (BMS_contactorState == 5) {  // BMS has detected welded contactor(s)
    set_event_latched(EVENT_CONTACTOR_WELDED, 0);
  } else if (BMS_contactorState != 5) {
    clear_event(EVENT_CONTACTOR_WELDED);
  }

  // Pack-internal contactors: DC bus is live only when the BMS confirms CLOSED (4).
  // Guarded so the GPIO contactor state machine stays authoritative when enabled.
  if (!contactor_control_enabled) {
    datalayer.system.status.dc_bus_live = (battery_contactor == 4);
  }

  if (user_selected_battery_type == BatteryType::TeslaModel3Y) {
    // Autodetect algorithm for chemistry on 3/Y packs.
    // NCM/A batteries have 96s, LFP has 102-108s
    // Drawback with this check is that it takes 3-5 minutes before all cells have been counted!
    if (datalayer_battery->info.number_of_cells > 101) {
      datalayer_battery->info.chemistry = battery_chemistry_enum::LFP;
    }

    //Once cell chemistry is determined, set maximum and minimum total pack voltage safety limits
    if (datalayer_battery->info.chemistry == battery_chemistry_enum::LFP) {
      datalayer_battery->info.max_design_voltage_dV = MAX_PACK_VOLTAGE_3Y_LFP;
      datalayer_battery->info.min_design_voltage_dV = MIN_PACK_VOLTAGE_3Y_LFP;
      datalayer_battery->info.max_cell_voltage_mV = MAX_CELL_VOLTAGE_LFP;
      datalayer_battery->info.min_cell_voltage_mV = MIN_CELL_VOLTAGE_LFP;
      datalayer_battery->info.max_cell_voltage_deviation_mV = MAX_CELL_DEVIATION_LFP;
    } else {  // NCM/A chemistry
      datalayer_battery->info.max_design_voltage_dV = MAX_PACK_VOLTAGE_3Y_NCMA;
      datalayer_battery->info.min_design_voltage_dV = MIN_PACK_VOLTAGE_3Y_NCMA;
      datalayer_battery->info.max_cell_voltage_mV = MAX_CELL_VOLTAGE_NCA_NCM;
      datalayer_battery->info.min_cell_voltage_mV = MIN_CELL_VOLTAGE_NCA_NCM;
      datalayer_battery->info.max_cell_voltage_deviation_mV = MAX_CELL_DEVIATION_NCA_NCM;
    }

    // During forced balancing request via webserver, we allow the battery to exceed normal safety parameters
    if (datalayer_battery->settings.user_requests_balancing) {
      datalayer_battery->status.real_soc = 9900;  //Force battery to show up as 99% when balancing
      datalayer_battery->info.max_design_voltage_dV = datalayer_battery->settings.balancing_max_pack_voltage_dV;
      datalayer_battery->info.max_cell_voltage_mV = datalayer_battery->settings.balancing_max_cell_voltage_mV;
      datalayer_battery->info.max_cell_voltage_deviation_mV =
          datalayer_battery->settings.balancing_max_deviation_cell_voltage_mV;
      datalayer_battery->status.max_charge_power_W = datalayer_battery->settings.balancing_float_power_W;
    }
  }

  // Check if user requests some action
  if (datalayer_battery->settings.user_requests_tesla_isolation_clear) {
    datalayer_battery->settings.user_requests_tesla_isolation_clear = false;
    start_sequence(TESLA_UDS_ISO_CLEAR);
  }
  // The reset commands below need the pack contactors OPEN, which the user does manually (put the car
  // in a mode that drops them) before pressing the button. A command asked for with the contactors
  // still closed is refused with a *_FAIL event rather than moving any HV. battery_contactor == 1 is
  // the BMS reporting its contactors OPEN.
  const bool contactors_open = (battery_contactor == 1);
  if (datalayer_battery->settings.user_requests_tesla_bms_reset) {
    datalayer_battery->settings.user_requests_tesla_bms_reset = false;
    if (contactors_open && BMS_a180_SW_ECU_reset_blocked == false) {
      start_sequence(TESLA_UDS_BMS_RESET);
      logging.println("INFO: BMS reset requested");
    } else {
      set_event(EVENT_BMS_RESET_REQ_FAIL, 0);  // also printing a log entry
      clear_event(EVENT_BMS_RESET_REQ_FAIL);
    }
  }
  if (datalayer_battery->settings.user_requests_tesla_hvp_reset) {
    datalayer_battery->settings.user_requests_tesla_hvp_reset = false;
    if (contactors_open) {
      pending_ecu_reset_req_id = HVP_UDS_REQUEST;
      set_event(EVENT_HVP_RESET_REQ_SUCCESS, 0);
      clear_event(EVENT_HVP_RESET_REQ_SUCCESS);
    } else {
      set_event(EVENT_HVP_RESET_REQ_FAIL, 0);
      clear_event(EVENT_HVP_RESET_REQ_FAIL);
    }
  }
  if (datalayer_battery->settings.user_requests_tesla_pcs_reset) {
    datalayer_battery->settings.user_requests_tesla_pcs_reset = false;
    if (contactors_open) {
      pending_ecu_reset_req_id = PCS_UDS_REQUEST;
      set_event(EVENT_PCS_RESET_REQ_SUCCESS, 0);
      clear_event(EVENT_PCS_RESET_REQ_SUCCESS);
    } else {
      set_event(EVENT_PCS_RESET_REQ_FAIL, 0);
      clear_event(EVENT_PCS_RESET_REQ_FAIL);
    }
  }
  if (datalayer_battery->settings.user_requests_tesla_hvc_backup_partial) {
    datalayer_battery->settings.user_requests_tesla_hvc_backup_partial = false;
    // ODIN parity: the backup just wants the pack awake. HV up is fine and the contactors are left
    // exactly where they are.
    if (start_hvc_backup()) {
      logging.println("INFO: HVC backup started");
    } else {
      refuse_hvc_backup();  // surfaced by hvc_backup_status_json so the UI can say why
    }
  }
  if (datalayer_battery->settings.user_requests_tesla_soc_reset) {
    datalayer_battery->settings.user_requests_tesla_soc_reset = false;
    if (contactors_open &&
        (datalayer_battery->status.real_soc < 1500 || datalayer_battery->status.real_soc > 9000)) {
      start_sequence(TESLA_UDS_SOC_RESET);
      logging.println("INFO: SOC reset requested");
    } else {
      set_event(EVENT_BATTERY_SOC_RESET_FAIL, 0, battery_index);  // also printing a log entry
      clear_event(EVENT_BATTERY_SOC_RESET_FAIL, battery_index);
    }
  }
  if (datalayer_battery->settings.user_requests_tesla_cac_reset) {
    datalayer_battery->settings.user_requests_tesla_cac_reset = false;
    if (contactors_open) {
      start_sequence(TESLA_UDS_CAC_RESET);
    } else {
      set_event(EVENT_CAC_RESET_FAIL, 0);
      clear_event(EVENT_CAC_RESET_FAIL);
    }
  }
  if (datalayer_battery->settings.user_requests_tesla_clear_blocked_contactors) {
    datalayer_battery->settings.user_requests_tesla_clear_blocked_contactors = false;
    // Ends by rebooting the HVBMS and HVP, so it is only run with the contactors already open.
    if (contactors_open) {
      start_sequence(TESLA_UDS_CBC);
      set_event(EVENT_CLEAR_BLOCKED_CONTACTORS_REQ, 0);
      clear_event(EVENT_CLEAR_BLOCKED_CONTACTORS_REQ);
    } else {
      set_event(EVENT_CLEAR_BLOCKED_CONTACTORS_FAIL, 0);
      clear_event(EVENT_CLEAR_BLOCKED_CONTACTORS_FAIL);
    }
  }
  if (datalayer_battery->settings.user_requests_tesla_disable_isolation) {
    datalayer_battery->settings.user_requests_tesla_disable_isolation = false;
    isolation_monitor_disable_latched = true;
    iso_mon_needs_assert = true;  //asserted once in SUPPORT below, re-armed only on a BMS restart
    set_event(EVENT_ISOLATION_MONITOR_DISABLED, 0);
    clear_event(EVENT_ISOLATION_MONITOR_DISABLED);
  }
  if (datalayer_battery->settings.user_requests_tesla_enable_isolation) {
    datalayer_battery->settings.user_requests_tesla_enable_isolation = false;
    isolation_monitor_disable_latched = false;
    isolation_monitor_disabling = false;
    set_event(EVENT_ISOLATION_MONITOR_ENABLED, 0);
    clear_event(EVENT_ISOLATION_MONITOR_ENABLED);
    start_sequence(TESLA_UDS_ISO_MON);
  }

  //Update 0x333 UI_chargeTerminationPct (bit 16, width 10) value to SOC max value - expose via UI?
  //One firmware version this was seen at bit 17 width 11
  write_signal_value(&TESLA_333, 16, 10, static_cast<int64_t>(datalayer_battery->settings.max_percentage / 10), false);

  // Update webserver datalayer
  //datalayer_extended.tesla.BMS_hvilFault = BMS_a036_SW_HvpHvilFault;
  datalayer_extended.tesla.hvil_status = battery_hvil_status;
  //0x20A
  datalayer_extended.tesla.packContNegativeState = battery_packContNegativeState;
  datalayer_extended.tesla.packContPositiveState = battery_packContPositiveState;
  datalayer_extended.tesla.packContactorSetState = battery_packContactorSetState;
  datalayer_extended.tesla.packCtrsClosingBlocked = battery_packCtrsClosingBlocked;
  datalayer_extended.tesla.pyroTestInProgress = battery_pyroTestInProgress;
  datalayer_extended.tesla.battery_packCtrsOpenNowRequested = battery_packCtrsOpenNowRequested;
  datalayer_extended.tesla.battery_packCtrsOpenRequested = battery_packCtrsOpenRequested;
  datalayer_extended.tesla.battery_packCtrsRequestStatus = battery_packCtrsRequestStatus;
  datalayer_extended.tesla.battery_packCtrsResetRequestRequired = battery_packCtrsResetRequestRequired;
  datalayer_extended.tesla.battery_dcLinkAllowedToEnergize = battery_dcLinkAllowedToEnergize;
  //0x72A
  if (parsed_battery_serialNumber && battery_serialNumber[13] != 0) {
    memcpy(datalayer_extended.tesla.battery_serialNumber, battery_serialNumber, sizeof(battery_serialNumber));
    datalayer_extended.tesla.battery_manufactureDate = battery_manufactureDate;
    //Fallback only. The startup identity pass reads 0xF014 as part of its chain, so this older
    //serial-triggered query would otherwise race it for the UDS channel and read the same DID twice.
    //It stays as a safety net for a pack whose identity pass never completed.
    if (!parsed_battery_partNumber && ident_attempts >= TESLA_IDENT_MAX_ATTEMPTS && !uds_is_busy()) {
      start_sequence(TESLA_UDS_QUERY);
    }
  }
  //Via UDS 0xF014 PACKAGE_PART_NUMBER
  if (parsed_battery_partNumber && battery_partNumber[11] != 0) {
    memcpy(datalayer_extended.tesla.battery_partNumber, battery_partNumber, sizeof(battery_partNumber));
  }
  //Via UDS 0xF012 BOARD_PART_NUMBER on the HVBMS
  if (parsed_BMS_partNumber && BMS_partNumber[11] != 0) {
    memcpy(datalayer_extended.tesla.BMS_partNumber, BMS_partNumber, sizeof(BMS_partNumber));
  }
  //0x3C4 broadcast, or UDS 0xF012 on the PCS
  if (parsed_PCS_partNumber && PCS_partNumber[11] != 0) {
    memcpy(datalayer_extended.tesla.PCS_partNumber, PCS_partNumber, sizeof(PCS_partNumber));
  }
  //0x2B4
  datalayer_extended.tesla.battery_dcdcLvBusVolt = battery_dcdcLvBusVolt;
  datalayer_extended.tesla.battery_dcdcHvBusVolt = battery_dcdcHvBusVolt;
  datalayer_extended.tesla.battery_dcdcLvOutputCurrent = battery_dcdcLvOutputCurrent;
  //0x352
  datalayer_extended.tesla.BMS352_mux = BMS352_mux;
  datalayer_extended.tesla.battery_nominal_full_pack_energy = battery_nominal_full_pack_energy;
  datalayer_extended.tesla.battery_nominal_full_pack_energy_m0 = battery_nominal_full_pack_energy_m0;
  datalayer_extended.tesla.battery_nominal_energy_remaining = battery_nominal_energy_remaining;
  datalayer_extended.tesla.battery_nominal_energy_remaining_m0 = battery_nominal_energy_remaining_m0;
  datalayer_extended.tesla.battery_ideal_energy_remaining = battery_ideal_energy_remaining;
  datalayer_extended.tesla.battery_ideal_energy_remaining_m0 = battery_ideal_energy_remaining_m0;
  datalayer_extended.tesla.battery_energy_to_charge_complete = battery_energy_to_charge_complete;
  datalayer_extended.tesla.battery_energy_to_charge_complete_m1 = battery_energy_to_charge_complete_m1;
  datalayer_extended.tesla.battery_energy_buffer = battery_energy_buffer;
  datalayer_extended.tesla.battery_energy_buffer_m1 = battery_energy_buffer_m1;
  datalayer_extended.tesla.battery_expected_energy_remaining_m1 = battery_expected_energy_remaining_m1;
  datalayer_extended.tesla.battery_full_charge_complete = battery_full_charge_complete;
  datalayer_extended.tesla.battery_fully_charged = battery_fully_charged;
  //0x3D2
  datalayer_battery->status.total_discharged_battery_Wh = battery_total_discharge;
  datalayer_battery->status.total_charged_battery_Wh = battery_total_charge;
  //0x392
  datalayer_extended.tesla.battery_moduleType = battery_moduleType;
  datalayer_extended.tesla.battery_packMass = battery_packMass;
  datalayer_extended.tesla.battery_platformMaxBusVoltage = battery_platformMaxBusVoltage;
  //0x2D2
  datalayer_extended.tesla.BMS_min_voltage = BMS_min_voltage;
  datalayer_extended.tesla.BMS_max_voltage = BMS_max_voltage;
  datalayer_extended.tesla.battery_max_charge_current = battery_max_charge_current;
  datalayer_extended.tesla.battery_max_discharge_current = battery_max_discharge_current;
  //0x292
  datalayer_extended.tesla.battery_beginning_of_life = battery_beginning_of_life;
  datalayer_extended.tesla.battery_battTempPct = battery_battTempPct;
  datalayer_extended.tesla.battery_soc_ave = battery_soc_ave;
  datalayer_extended.tesla.battery_soc_max = battery_soc_max;
  datalayer_extended.tesla.battery_soc_min = battery_soc_min;
  datalayer_extended.tesla.battery_soc_ui = battery_soc_ui;
  //0x332
  datalayer_extended.tesla.battery_BrickVoltageMax = battery_BrickVoltageMax;
  datalayer_extended.tesla.battery_BrickVoltageMin = battery_BrickVoltageMin;
  datalayer_extended.tesla.battery_BrickTempMaxNum = battery_BrickTempMaxNum;
  datalayer_extended.tesla.battery_BrickTempMinNum = battery_BrickTempMinNum;
  datalayer_extended.tesla.battery_BrickModelTMax = battery_BrickModelTMax;
  datalayer_extended.tesla.battery_BrickModelTMin = battery_BrickModelTMin;
  //0x212
  datalayer_extended.tesla.BMS_isolationResistance = BMS_isolationResistance;
  datalayer_extended.tesla.BMS_contactorState = BMS_contactorState;
  datalayer_extended.tesla.BMS_state = BMS_state;
  datalayer_extended.tesla.BMS_hvState = BMS_hvState;

  //Experimental Balancing Monitoring
  //Tesla does not broadcast per-cell bleed status on the vehicle bus
  //However, we should be able to query per-brick bleed time via UDS in a future PR
  //Balancing *does* happen with contactors closed, not observed in DRIVE, just SUPPORT (ACCESSORY)
  //This is just a first experimental step to monitor behaviour/stats
  datalayer_extended.tesla.BMS_balanceState = BMS_balanceState;
  datalayer_extended.tesla.BMS_vshTestActive = (BMS_balanceState == BALANCE_STATE_VSH_TEST);
  datalayer_extended.tesla.BMS_balanceStateUnknown = BMS_balanceStateUnknown;
  datalayer_extended.tesla.BMS_balanceTimeThisCycle_s = BMS_balanceTimeThisCycle_s;
  datalayer_extended.tesla.BMS_cumulativeBleedTime_s = BMS_cumulativeBleedTime_s;
  datalayer_extended.tesla.BMS_cacEstimationSource = BMS_cacEstimationSource;

  if (BMS_balanceStateUnknown) {
    set_balancing_status(BALANCING_STATUS_UNKNOWN);
  } else if (BMS_balanceState == BALANCE_STATE_BALANCING) {
    set_balancing_status(BALANCING_STATUS_ACTIVE);
  } else {
    //The VSH test switches on the bleed FETs - but is a diagnostic, not balancing
    //Therefore it is reported as READY with the separate vshTestActive flag raised.
    set_balancing_status(BALANCING_STATUS_READY);
  }

  if (datalayer_battery->status.balancing_status == BALANCING_STATUS_ACTIVE) {
    if (!uds_is_busy() &&
        (brick_bleed_last_poll == 0 || (millis() - brick_bleed_last_poll) >= TESLA_BLEED_POLL_INTERVAL_MS)) {
      brick_bleed_last_poll = millis();
      start_sequence(TESLA_UDS_BLEED);
    }
  } else if (datalayer_extended.tesla.BMS_brickBleedValid) {
    //Balancing has stopped: clear the per-brick cyan and the bleeding summary so nothing is left
    //latched on, and re-baseline so the next episode is measured from a fresh reading.
    for (uint16_t i = 0; i < MAX_AMOUNT_CELLS; i++) {
      datalayer_battery->status.cell_balancing_status[i] = false;
    }
    datalayer_extended.tesla.BMS_brickBleedActiveCount = 0;
    datalayer_extended.tesla.BMS_brickBleedValid = false;
    brick_bleed_has_prev = false;
  }

  //The disable persists in the BMS until it resets, so it only needs re-asserting
  //when the BMS restarts. Detect that as the battery going stale and coming back.
  bool battery_alive = datalayer_battery->status.CAN_battery_still_alive > 0;
  if (battery_alive && !battery_was_alive && isolation_monitor_disable_latched) {
    iso_mon_needs_assert = true;
  }
  battery_was_alive = battery_alive;

  if (isolation_monitor_disable_latched && iso_mon_needs_assert && BMS_state == BMS_STATE_SUPPORT &&
      !uds_is_busy()) {
    iso_mon_needs_assert = false;
    isolation_monitor_disabling = true;
    start_sequence(TESLA_UDS_ISO_MON);
  }

  //Populate the cell monitor immediately over UDS (DID 0x410) rather than waiting minutes for all
  //0x401 brick-voltage muxes to broadcast. One-shot once the BMS is reachable; falls back to 0x401.
  if (!brickv_requested && brickv_attempts < TESLA_BRICKV_MAX_ATTEMPTS && battery_alive && !uds_is_busy() &&
      datalayer_battery->info.number_of_cells == 0) {
    brickv_requested = true;
    brickv_attempts++;
    start_sequence(TESLA_UDS_BRICKV);
  }

  //Read the pack identity straight off the ECUs once the BMS is reachable, rather than waiting for
  //the 0x72A / 0x3C4 broadcasts and deriving the birthday from the serial. One-shot per boot.
  if (!ident_requested && ident_attempts < TESLA_IDENT_MAX_ATTEMPTS && battery_alive && !uds_is_busy()) {
    ident_requested = true;
    ident_attempts++;
    start_identity_query();
  }
  datalayer_extended.tesla.BMS_uiChargeStatus = BMS_uiChargeStatus;
  datalayer_extended.tesla.BMS_diLimpRequest = BMS_diLimpRequest;
  datalayer_extended.tesla.BMS_chgPowerAvailable = BMS_chgPowerAvailable;
  datalayer_extended.tesla.BMS_pcsPwmEnabled = BMS_pcsPwmEnabled;
  //0x224
  datalayer_extended.tesla.PCS_dcdcPrechargeStatus = PCS_dcdcPrechargeStatus;
  datalayer_extended.tesla.PCS_dcdc12VSupportStatus = PCS_dcdc12VSupportStatus;
  datalayer_extended.tesla.PCS_dcdcHvBusDischargeStatus = PCS_dcdcHvBusDischargeStatus;
  datalayer_extended.tesla.PCS_dcdcMainState = PCS_dcdcMainState;
  datalayer_extended.tesla.PCS_dcdcSubState = PCS_dcdcSubState;
  datalayer_extended.tesla.PCS_dcdcFaulted = PCS_dcdcFaulted;
  datalayer_extended.tesla.PCS_dcdcOutputIsLimited = PCS_dcdcOutputIsLimited;
  datalayer_extended.tesla.PCS_dcdcMaxOutputCurrentAllowed = PCS_dcdcMaxOutputCurrentAllowed;
  datalayer_extended.tesla.PCS_dcdcPrechargeRtyCnt = PCS_dcdcPrechargeRtyCnt;
  datalayer_extended.tesla.PCS_dcdc12VSupportRtyCnt = PCS_dcdc12VSupportRtyCnt;
  datalayer_extended.tesla.PCS_dcdcDischargeRtyCnt = PCS_dcdcDischargeRtyCnt;
  datalayer_extended.tesla.PCS_dcdcPwmEnableLine = PCS_dcdcPwmEnableLine;
  datalayer_extended.tesla.PCS_dcdcSupportingFixedLvTarget = PCS_dcdcSupportingFixedLvTarget;
  datalayer_extended.tesla.PCS_dcdcPrechargeRestartCnt = PCS_dcdcPrechargeRestartCnt;
  datalayer_extended.tesla.PCS_dcdcInitialPrechargeSubState = PCS_dcdcInitialPrechargeSubState;
  //0x252
  datalayer_extended.tesla.BMS_maxRegenPower = BMS_maxRegenPower;
  datalayer_extended.tesla.BMS_maxDischargePower = BMS_maxDischargePower;
  datalayer_extended.tesla.BMS_maxStationaryHeatPower = BMS_maxStationaryHeatPower;
  datalayer_extended.tesla.BMS_hvacPowerBudget = BMS_hvacPowerBudget;
  datalayer_extended.tesla.BMS_notEnoughPowerForHeatPump = BMS_notEnoughPowerForHeatPump;
  datalayer_extended.tesla.BMS_powerLimitState = BMS_powerLimitState;
  datalayer_extended.tesla.BMS_inverterTQF = BMS_inverterTQF;
  //Via UDS
  //datalayer_extended.tesla.BMS_info_buildConfigId = BMS_info_buildConfigId;
  //0x300
  datalayer_extended.tesla.BMS_info_buildConfigId = BMS_info_buildConfigId;
  datalayer_extended.tesla.BMS_info_hardwareId = BMS_info_hardwareId;
  datalayer_extended.tesla.BMS_info_componentId = BMS_info_componentId;
  datalayer_extended.tesla.BMS_info_pcbaId = BMS_info_pcbaId;
  datalayer_extended.tesla.BMS_info_assemblyId = BMS_info_assemblyId;
  datalayer_extended.tesla.BMS_info_usageId = BMS_info_usageId;
  datalayer_extended.tesla.BMS_info_subUsageId = BMS_info_subUsageId;
  datalayer_extended.tesla.BMS_info_platformType = BMS_info_platformType;
  datalayer_extended.tesla.BMS_info_appCrc = BMS_info_appCrc;
  datalayer_extended.tesla.BMS_info_bootGitHash = BMS_info_bootGitHash;
  datalayer_extended.tesla.BMS_info_bootUdsProtoVersion = BMS_info_bootUdsProtoVersion;
  datalayer_extended.tesla.BMS_info_bootCrc = BMS_info_bootCrc;
  //0x312
  datalayer_extended.tesla.BMS_powerDissipation = BMS_powerDissipation;
  datalayer_extended.tesla.BMS_flowRequest = BMS_flowRequest;
  datalayer_extended.tesla.BMS_inletActiveCoolTargetT = BMS_inletActiveCoolTargetT;
  datalayer_extended.tesla.BMS_inletPassiveTargetT = BMS_inletPassiveTargetT;
  datalayer_extended.tesla.BMS_inletActiveHeatTargetT = BMS_inletActiveHeatTargetT;
  datalayer_extended.tesla.BMS_packTMin = BMS_packTMin;
  datalayer_extended.tesla.BMS_packTMax = BMS_packTMax;
  datalayer_extended.tesla.BMS_pcsNoFlowRequest = BMS_pcsNoFlowRequest;
  datalayer_extended.tesla.BMS_noFlowRequest = BMS_noFlowRequest;
  //0x3C4
  datalayer_extended.tesla.PCS_info_buildConfigId = PCS_info_buildConfigId;
  datalayer_extended.tesla.PCS_info_hardwareId = PCS_info_hardwareId;
  datalayer_extended.tesla.PCS_info_componentId = PCS_info_componentId;
  //0x2A4
  datalayer_extended.tesla.PCS_dcdcTemp = PCS_dcdcTemp;
  datalayer_extended.tesla.PCS_ambientTemp = PCS_ambientTemp;
  datalayer_extended.tesla.PCS_chgPhATemp = PCS_chgPhATemp;
  datalayer_extended.tesla.PCS_chgPhBTemp = PCS_chgPhBTemp;
  datalayer_extended.tesla.PCS_chgPhCTemp = PCS_chgPhCTemp;
  //0x2C4
  datalayer_extended.tesla.PCS_dcdcMaxLvOutputCurrent = PCS_dcdcMaxLvOutputCurrent;
  datalayer_extended.tesla.PCS_dcdcCurrentLimit = PCS_dcdcCurrentLimit;
  datalayer_extended.tesla.PCS_dcdcLvOutputCurrentTempLimit = PCS_dcdcLvOutputCurrentTempLimit;
  datalayer_extended.tesla.PCS_dcdcUnifiedCommand = PCS_dcdcUnifiedCommand;
  datalayer_extended.tesla.PCS_dcdcCLAControllerOutput = PCS_dcdcCLAControllerOutput;
  datalayer_extended.tesla.PCS_dcdcTankVoltage = PCS_dcdcTankVoltage;
  datalayer_extended.tesla.PCS_dcdcTankVoltageTarget = PCS_dcdcTankVoltageTarget;
  datalayer_extended.tesla.PCS_dcdcClaCurrentFreq = PCS_dcdcClaCurrentFreq;
  datalayer_extended.tesla.PCS_dcdcTCommMeasured = PCS_dcdcTCommMeasured;
  datalayer_extended.tesla.PCS_dcdcShortTimeUs = PCS_dcdcShortTimeUs;
  datalayer_extended.tesla.PCS_dcdcHalfPeriodUs = PCS_dcdcHalfPeriodUs;
  datalayer_extended.tesla.PCS_dcdcIntervalMaxFrequency = PCS_dcdcIntervalMaxFrequency;
  datalayer_extended.tesla.PCS_dcdcIntervalMaxHvBusVolt = PCS_dcdcIntervalMaxHvBusVolt;
  datalayer_extended.tesla.PCS_dcdcIntervalMaxLvBusVolt = PCS_dcdcIntervalMaxLvBusVolt;
  datalayer_extended.tesla.PCS_dcdcIntervalMaxLvOutputCurr = PCS_dcdcIntervalMaxLvOutputCurr;
  datalayer_extended.tesla.PCS_dcdcIntervalMinFrequency = PCS_dcdcIntervalMinFrequency;
  datalayer_extended.tesla.PCS_dcdcIntervalMinHvBusVolt = PCS_dcdcIntervalMinHvBusVolt;
  datalayer_extended.tesla.PCS_dcdcIntervalMinLvBusVolt = PCS_dcdcIntervalMinLvBusVolt;
  datalayer_extended.tesla.PCS_dcdcIntervalMinLvOutputCurr = PCS_dcdcIntervalMinLvOutputCurr;
  datalayer_extended.tesla.PCS_dcdc12vSupportLifetimekWh = PCS_dcdc12vSupportLifetimekWh;
  //0x310
  datalayer_extended.tesla.HVP_info_buildConfigId = HVP_info_buildConfigId;
  datalayer_extended.tesla.HVP_info_hardwareId = HVP_info_hardwareId;
  datalayer_extended.tesla.HVP_info_componentId = HVP_info_componentId;
  //0x7AA
  datalayer_extended.tesla.HVP_gpioPassivePyroDepl = HVP_gpioPassivePyroDepl;
  datalayer_extended.tesla.HVP_gpioPyroIsoEn = HVP_gpioPyroIsoEn;
  datalayer_extended.tesla.HVP_gpioCpFaultIn = HVP_gpioCpFaultIn;
  datalayer_extended.tesla.HVP_gpioPackContPowerEn = HVP_gpioPackContPowerEn;
  datalayer_extended.tesla.HVP_gpioHvCablesOk = HVP_gpioHvCablesOk;
  datalayer_extended.tesla.HVP_gpioHvpSelfEnable = HVP_gpioHvpSelfEnable;
  datalayer_extended.tesla.HVP_gpioLed = HVP_gpioLed;
  datalayer_extended.tesla.HVP_gpioCrashSignal = HVP_gpioCrashSignal;
  datalayer_extended.tesla.HVP_gpioShuntDataReady = HVP_gpioShuntDataReady;
  datalayer_extended.tesla.HVP_gpioFcContPosAux = HVP_gpioFcContPosAux;
  datalayer_extended.tesla.HVP_gpioFcContNegAux = HVP_gpioFcContNegAux;
  datalayer_extended.tesla.HVP_gpioBmsEout = HVP_gpioBmsEout;
  datalayer_extended.tesla.HVP_gpioCpFaultOut = HVP_gpioCpFaultOut;
  datalayer_extended.tesla.HVP_gpioPyroPor = HVP_gpioPyroPor;
  datalayer_extended.tesla.HVP_gpioShuntEn = HVP_gpioShuntEn;
  datalayer_extended.tesla.HVP_gpioHvpVerEn = HVP_gpioHvpVerEn;
  datalayer_extended.tesla.HVP_gpioPackCoontPosFlywheel = HVP_gpioPackCoontPosFlywheel;
  datalayer_extended.tesla.HVP_gpioCpLatchEnable = HVP_gpioCpLatchEnable;
  datalayer_extended.tesla.HVP_gpioPcsEnable = HVP_gpioPcsEnable;
  datalayer_extended.tesla.HVP_gpioPcsDcdcPwmEnable = HVP_gpioPcsDcdcPwmEnable;
  datalayer_extended.tesla.HVP_gpioPcsChargePwmEnable = HVP_gpioPcsChargePwmEnable;
  datalayer_extended.tesla.HVP_gpioFcContPowerEnable = HVP_gpioFcContPowerEnable;
  datalayer_extended.tesla.HVP_gpioHvilEnable = HVP_gpioHvilEnable;
  datalayer_extended.tesla.HVP_gpioSecDrdy = HVP_gpioSecDrdy;
  datalayer_extended.tesla.HVP_hvp1v5Ref = HVP_hvp1v5Ref;
  datalayer_extended.tesla.HVP_shuntCurrentDebug = HVP_shuntCurrentDebug;
  datalayer_extended.tesla.HVP_packCurrentMia = HVP_packCurrentMia;
  datalayer_extended.tesla.HVP_auxCurrentMia = HVP_auxCurrentMia;
  datalayer_extended.tesla.HVP_currentSenseMia = HVP_currentSenseMia;
  datalayer_extended.tesla.HVP_shuntRefVoltageMismatch = HVP_shuntRefVoltageMismatch;
  datalayer_extended.tesla.HVP_shuntThermistorMia = HVP_shuntThermistorMia;
  datalayer_extended.tesla.HVP_shuntHwMia = HVP_shuntHwMia;
  datalayer_extended.tesla.HVP_dcLinkVoltage = HVP_dcLinkVoltage;
  datalayer_extended.tesla.HVP_packVoltage = HVP_packVoltage;
  datalayer_extended.tesla.HVP_fcLinkVoltage = HVP_fcLinkVoltage;
  datalayer_extended.tesla.HVP_packContVoltage = HVP_packContVoltage;
  datalayer_extended.tesla.HVP_packNegativeV = HVP_packNegativeV;
  datalayer_extended.tesla.HVP_packPositiveV = HVP_packPositiveV;
  datalayer_extended.tesla.HVP_pyroAnalog = HVP_pyroAnalog;
  datalayer_extended.tesla.HVP_dcLinkNegativeV = HVP_dcLinkNegativeV;
  datalayer_extended.tesla.HVP_dcLinkPositiveV = HVP_dcLinkPositiveV;
  datalayer_extended.tesla.HVP_fcLinkNegativeV = HVP_fcLinkNegativeV;
  datalayer_extended.tesla.HVP_fcContCoilCurrent = HVP_fcContCoilCurrent;
  datalayer_extended.tesla.HVP_fcContVoltage = HVP_fcContVoltage;
  datalayer_extended.tesla.HVP_hvilInVoltage = HVP_hvilInVoltage;
  datalayer_extended.tesla.HVP_hvilOutVoltage = HVP_hvilOutVoltage;
  datalayer_extended.tesla.HVP_fcLinkPositiveV = HVP_fcLinkPositiveV;
  datalayer_extended.tesla.HVP_packContCoilCurrent = HVP_packContCoilCurrent;
  datalayer_extended.tesla.HVP_battery12V = HVP_battery12V;
  datalayer_extended.tesla.HVP_shuntRefVoltageDbg = HVP_shuntRefVoltageDbg;
  datalayer_extended.tesla.HVP_shuntAuxCurrentDbg = HVP_shuntAuxCurrentDbg;
  datalayer_extended.tesla.HVP_shuntBarTempDbg = HVP_shuntBarTempDbg;
  datalayer_extended.tesla.HVP_shuntAsicTempDbg = HVP_shuntAsicTempDbg;
  datalayer_extended.tesla.HVP_shuntAuxCurrentStatus = HVP_shuntAuxCurrentStatus;
  datalayer_extended.tesla.HVP_shuntBarTempStatus = HVP_shuntBarTempStatus;
  datalayer_extended.tesla.HVP_shuntAsicTempStatus = HVP_shuntAsicTempStatus;

  // Alert-matrix (DTC) fault state -> datalayer for the Faults web page (fw 2019.20.4.2)
  datalayer_extended.tesla.BMS_alertMatrixActive[0] = BMS_a001_Pack_Config_Mismatch;
  datalayer_extended.tesla.BMS_alertMatrixActive[1] = BMS_a017_SW_Brick_OV;
  datalayer_extended.tesla.BMS_alertMatrixActive[2] = BMS_a018_SW_Brick_UV;
  datalayer_extended.tesla.BMS_alertMatrixActive[3] = BMS_a019_SW_Module_OT;
  datalayer_extended.tesla.BMS_alertMatrixActive[4] = BMS_a021_SW_Dr_Limits_Regulation;
  datalayer_extended.tesla.BMS_alertMatrixActive[5] = BMS_a022_SW_Over_Current;
  datalayer_extended.tesla.BMS_alertMatrixActive[6] = BMS_a023_SW_Stack_OV;
  datalayer_extended.tesla.BMS_alertMatrixActive[7] = BMS_a024_SW_Islanded_Brick;
  datalayer_extended.tesla.BMS_alertMatrixActive[8] = BMS_a025_SW_PwrBalance_Anomaly;
  datalayer_extended.tesla.BMS_alertMatrixActive[9] = BMS_a026_SW_HFCurrent_Anomaly;
  datalayer_extended.tesla.BMS_alertMatrixActive[10] = BMS_a034_SW_Passive_Isolation;
  datalayer_extended.tesla.BMS_alertMatrixActive[11] = BMS_a035_SW_Isolation;
  datalayer_extended.tesla.BMS_alertMatrixActive[12] = BMS_a036_SW_HvpHvilFault;
  datalayer_extended.tesla.BMS_alertMatrixActive[13] = BMS_a037_SW_Flood_Port_Open;
  datalayer_extended.tesla.BMS_alertMatrixActive[14] = BMS_a039_SW_DC_Link_Over_Voltage;
  datalayer_extended.tesla.BMS_alertMatrixActive[15] = BMS_a041_SW_Power_On_Reset;
  datalayer_extended.tesla.BMS_alertMatrixActive[16] = BMS_a042_SW_MPU_Error;
  datalayer_extended.tesla.BMS_alertMatrixActive[17] = BMS_a043_SW_Watch_Dog_Reset;
  datalayer_extended.tesla.BMS_alertMatrixActive[18] = BMS_a044_SW_Assertion;
  datalayer_extended.tesla.BMS_alertMatrixActive[19] = BMS_a045_SW_Exception;
  datalayer_extended.tesla.BMS_alertMatrixActive[20] = BMS_a046_SW_Task_Stack_Usage;
  datalayer_extended.tesla.BMS_alertMatrixActive[21] = BMS_a047_SW_Task_Stack_Overflow;
  datalayer_extended.tesla.BMS_alertMatrixActive[22] = BMS_a048_SW_Log_Upload_Request;
  datalayer_extended.tesla.BMS_alertMatrixActive[23] = BMS_a050_SW_Brick_Voltage_MIA;
  datalayer_extended.tesla.BMS_alertMatrixActive[24] = BMS_a051_SW_HVC_Vref_Bad;
  datalayer_extended.tesla.BMS_alertMatrixActive[25] = BMS_a052_SW_PCS_MIA;
  datalayer_extended.tesla.BMS_alertMatrixActive[26] = BMS_a053_SW_ThermalModel_Sanity;
  datalayer_extended.tesla.BMS_alertMatrixActive[27] = BMS_a054_SW_Ver_Supply_Fault;
  datalayer_extended.tesla.BMS_alertMatrixActive[28] = BMS_a055_SW_HvChain_Model_Fault;
  datalayer_extended.tesla.BMS_alertMatrixActive[29] = BMS_a059_SW_Pack_Voltage_Sensing;
  datalayer_extended.tesla.BMS_alertMatrixActive[30] = BMS_a060_SW_Leakage_Test_Failure;
  datalayer_extended.tesla.BMS_alertMatrixActive[31] = BMS_a061_robinBrickOverVoltage;
  datalayer_extended.tesla.BMS_alertMatrixActive[32] = BMS_a062_SW_BrickV_Imbalance;
  datalayer_extended.tesla.BMS_alertMatrixActive[33] = BMS_a063_SW_ChargePort_Fault;
  datalayer_extended.tesla.BMS_alertMatrixActive[34] = BMS_a064_SW_SOC_Imbalance;
  datalayer_extended.tesla.BMS_alertMatrixActive[35] = BMS_a069_SW_Low_Power;
  datalayer_extended.tesla.BMS_alertMatrixActive[36] = BMS_a071_SW_SM_TransCon_Not_Met;
  datalayer_extended.tesla.BMS_alertMatrixActive[37] = BMS_a075_SW_Chg_Disable_Failure;
  datalayer_extended.tesla.BMS_alertMatrixActive[38] = BMS_a076_SW_Dch_While_Charging;
  datalayer_extended.tesla.BMS_alertMatrixActive[39] = BMS_a077_SW_Charger_Regulation;
  datalayer_extended.tesla.BMS_alertMatrixActive[40] = BMS_a081_SW_Ctr_Close_Blocked;
  datalayer_extended.tesla.BMS_alertMatrixActive[41] = BMS_a082_SW_Ctr_Force_Open;
  datalayer_extended.tesla.BMS_alertMatrixActive[42] = BMS_a083_SW_Ctr_Close_Failure;
  datalayer_extended.tesla.BMS_alertMatrixActive[43] = BMS_a084_SW_Sleep_Wake_Aborted;
  datalayer_extended.tesla.BMS_alertMatrixActive[44] = BMS_a087_SW_Feim_Test_Blocked;
  datalayer_extended.tesla.BMS_alertMatrixActive[45] = BMS_a088_SW_VcFront_MIA_InDrive;
  datalayer_extended.tesla.BMS_alertMatrixActive[46] = BMS_a089_SW_VcFront_MIA;
  datalayer_extended.tesla.BMS_alertMatrixActive[47] = BMS_a090_SW_Gateway_MIA;
  datalayer_extended.tesla.BMS_alertMatrixActive[48] = BMS_a091_SW_ChargePort_MIA;
  datalayer_extended.tesla.BMS_alertMatrixActive[49] = BMS_a092_SW_ChargePort_Mia_On_Hv;
  datalayer_extended.tesla.BMS_alertMatrixActive[50] = BMS_a094_SW_Drive_Inverter_MIA;
  datalayer_extended.tesla.BMS_alertMatrixActive[51] = BMS_a099_SW_BMB_Communication;
  datalayer_extended.tesla.BMS_alertMatrixActive[52] = BMS_a105_SW_One_Module_Tsense;
  datalayer_extended.tesla.BMS_alertMatrixActive[53] = BMS_a106_SW_All_Module_Tsense;
  datalayer_extended.tesla.BMS_alertMatrixActive[54] = BMS_a107_SW_Stack_Voltage_MIA;
  datalayer_extended.tesla.BMS_alertMatrixActive[55] = BMS_a121_SW_NVRAM_Config_Error;
  datalayer_extended.tesla.BMS_alertMatrixActive[56] = BMS_a122_SW_BMS_Therm_Irrational;
  datalayer_extended.tesla.BMS_alertMatrixActive[57] = BMS_a123_SW_Internal_Isolation;
  datalayer_extended.tesla.BMS_alertMatrixActive[58] = BMS_a126_SW_Thermistor_Failure;
  datalayer_extended.tesla.BMS_alertMatrixActive[59] = BMS_a127_SW_shunt_SNA;
  datalayer_extended.tesla.BMS_alertMatrixActive[60] = BMS_a128_SW_shunt_MIA;
  datalayer_extended.tesla.BMS_alertMatrixActive[61] = BMS_a129_SW_VSH_Failure;
  datalayer_extended.tesla.BMS_alertMatrixActive[62] = BMS_a130_IO_CAN_Error;
  datalayer_extended.tesla.BMS_alertMatrixActive[63] = BMS_a131_Bleed_FET_Failure;
  datalayer_extended.tesla.BMS_alertMatrixActive[64] = BMS_a132_HW_BMB_OTP_Uncorrctbl;
  datalayer_extended.tesla.BMS_alertMatrixActive[65] = BMS_a134_SW_Delayed_Ctr_Off;
  datalayer_extended.tesla.BMS_alertMatrixActive[66] = BMS_a135_HW_BMB_Diagnostics_Failure;
  datalayer_extended.tesla.BMS_alertMatrixActive[67] = BMS_a136_SW_Module_OT_Warning;
  datalayer_extended.tesla.BMS_alertMatrixActive[68] = BMS_a137_SW_Brick_UV_Warning;
  datalayer_extended.tesla.BMS_alertMatrixActive[69] = BMS_a138_SW_Brick_OV_Warning;
  datalayer_extended.tesla.BMS_alertMatrixActive[70] = BMS_a139_SW_DC_Link_V_Irrational;
  datalayer_extended.tesla.BMS_alertMatrixActive[71] = BMS_a141_SW_BMB_Status_Warning;
  datalayer_extended.tesla.BMS_alertMatrixActive[72] = BMS_a143_SW_CAC_Change;
  datalayer_extended.tesla.BMS_alertMatrixActive[73] = BMS_a144_Hvp_Config_Mismatch;
  datalayer_extended.tesla.BMS_alertMatrixActive[74] = BMS_a145_SW_SOC_Change;
  datalayer_extended.tesla.BMS_alertMatrixActive[75] = BMS_a146_SW_Brick_Overdischarged;
  datalayer_extended.tesla.BMS_alertMatrixActive[76] = BMS_a149_SW_Missing_Config_Block;
  datalayer_extended.tesla.BMS_alertMatrixActive[77] = BMS_a151_SW_external_isolation;
  datalayer_extended.tesla.BMS_alertMatrixActive[78] = BMS_a155_SW_Weak_short_impedence;
  datalayer_extended.tesla.BMS_alertMatrixActive[79] = BMS_a156_SW_BMB_Vref_bad;
  datalayer_extended.tesla.BMS_alertMatrixActive[80] = BMS_a157_SW_HVP_HVS_Comms;
  datalayer_extended.tesla.BMS_alertMatrixActive[81] = BMS_a158_SW_HVP_HVI_Comms;
  datalayer_extended.tesla.BMS_alertMatrixActive[82] = BMS_a159_SW_HVP_ECU_Error;
  datalayer_extended.tesla.BMS_alertMatrixActive[83] = BMS_a161_SW_DI_Open_Request;
  datalayer_extended.tesla.BMS_alertMatrixActive[84] = BMS_a162_SW_No_Power_For_Support;
  datalayer_extended.tesla.BMS_alertMatrixActive[85] = BMS_a163_SW_Contactor_Mismatch;
  datalayer_extended.tesla.BMS_alertMatrixActive[86] = BMS_a164_SW_Uncontrolled_Regen;
  datalayer_extended.tesla.BMS_alertMatrixActive[87] = BMS_a165_SW_Pack_Partial_Weld;
  datalayer_extended.tesla.BMS_alertMatrixActive[88] = BMS_a166_SW_Pack_Full_Weld;
  datalayer_extended.tesla.BMS_alertMatrixActive[89] = BMS_a167_SW_FC_Partial_Weld;
  datalayer_extended.tesla.BMS_alertMatrixActive[90] = BMS_a168_SW_FC_Full_Weld;
  datalayer_extended.tesla.BMS_alertMatrixActive[91] = BMS_a169_SW_FC_Pack_Weld;
  datalayer_extended.tesla.BMS_alertMatrixActive[92] = BMS_a170_SW_Limp_Mode;
  datalayer_extended.tesla.BMS_alertMatrixActive[93] = BMS_a171_SW_Stack_Voltage_Sense;
  datalayer_extended.tesla.BMS_alertMatrixActive[94] = BMS_a173_SW_Charge_Component_Fault;
  datalayer_extended.tesla.BMS_alertMatrixActive[95] = BMS_a174_SW_Charge_Failure;
  datalayer_extended.tesla.BMS_alertMatrixActive[96] = BMS_a176_SW_GracefulPowerOff;
  datalayer_extended.tesla.BMS_alertMatrixActive[97] = BMS_a178_SW_Uncontrolled_Regen_PwrB;
  datalayer_extended.tesla.BMS_alertMatrixActive[98] = BMS_a179_SW_Hvp_12V_Fault;
  datalayer_extended.tesla.BMS_alertMatrixActive[99] = BMS_a180_SW_ECU_reset_blocked;
  datalayer_extended.tesla.PCS_alertMatrixActive[0] = PCS_a001_chgHwInputOc;
  datalayer_extended.tesla.PCS_alertMatrixActive[1] = PCS_a002_chgHwOutputOc;
  datalayer_extended.tesla.PCS_alertMatrixActive[2] = PCS_a003_chgHwInputOv;
  datalayer_extended.tesla.PCS_alertMatrixActive[3] = PCS_a004_chgHwIntBusOv;
  datalayer_extended.tesla.PCS_alertMatrixActive[4] = PCS_a005_chgOutputOv;
  datalayer_extended.tesla.PCS_alertMatrixActive[5] = PCS_a006_chgPrechargeFailedScr;
  datalayer_extended.tesla.PCS_alertMatrixActive[6] = PCS_a007_chgPhaseTempHot;
  datalayer_extended.tesla.PCS_alertMatrixActive[7] = PCS_a008_chgPhaseOverTemp;
  datalayer_extended.tesla.PCS_alertMatrixActive[8] = PCS_a009_chgPfcCurrentRegulation;
  datalayer_extended.tesla.PCS_alertMatrixActive[9] = PCS_a010_chgIntBusVRegulation;
  datalayer_extended.tesla.PCS_alertMatrixActive[10] = PCS_a011_chgLlcCurrentRegulation;
  datalayer_extended.tesla.PCS_alertMatrixActive[11] = PCS_a012_chgPfcIBandTracerFault;
  datalayer_extended.tesla.PCS_alertMatrixActive[12] = PCS_a013_chgPrechargeFailedBoost;
  datalayer_extended.tesla.PCS_alertMatrixActive[13] = PCS_a014_chgTempRationality;
  datalayer_extended.tesla.PCS_alertMatrixActive[14] = PCS_a015_chg12vUv;
  datalayer_extended.tesla.PCS_alertMatrixActive[15] = PCS_a016_chgAllPhasesFaulted;
  datalayer_extended.tesla.PCS_alertMatrixActive[16] = PCS_a017_chgWallPowerRemoval;
  datalayer_extended.tesla.PCS_alertMatrixActive[17] = PCS_a018_chgUnknownGridConfig;
  datalayer_extended.tesla.PCS_alertMatrixActive[18] = PCS_a019_acChargePowerLimited;
  datalayer_extended.tesla.PCS_alertMatrixActive[19] = PCS_a020_chgEnableLineMismatch;
  datalayer_extended.tesla.PCS_alertMatrixActive[20] = PCS_a021_hvpMia;
  datalayer_extended.tesla.PCS_alertMatrixActive[21] = PCS_a022_bmsMia;
  datalayer_extended.tesla.PCS_alertMatrixActive[22] = PCS_a023_cpMia;
  datalayer_extended.tesla.PCS_alertMatrixActive[23] = PCS_a024_vcfrontMia;
  datalayer_extended.tesla.PCS_alertMatrixActive[24] = PCS_a025_cpu2Malfunction;
  datalayer_extended.tesla.PCS_alertMatrixActive[25] = PCS_a026_watchdogAlarmed;
  datalayer_extended.tesla.PCS_alertMatrixActive[26] = PCS_a027_chgInsufficientCooling;
  datalayer_extended.tesla.PCS_alertMatrixActive[27] = PCS_a028_chgOutputUv;
  datalayer_extended.tesla.PCS_alertMatrixActive[28] = PCS_a029_chgPowerRationality;
  datalayer_extended.tesla.PCS_alertMatrixActive[29] = PCS_a030_canRationality;
  datalayer_extended.tesla.PCS_alertMatrixActive[30] = PCS_a031_uiMia;
  datalayer_extended.tesla.PCS_alertMatrixActive[31] = PCS_a032_gtwMia;
  datalayer_extended.tesla.PCS_alertMatrixActive[32] = PCS_a033_hvBusUv;
  datalayer_extended.tesla.PCS_alertMatrixActive[33] = PCS_a034_hvBusOv;
  datalayer_extended.tesla.PCS_alertMatrixActive[34] = PCS_a035_lvBusUv;
  datalayer_extended.tesla.PCS_alertMatrixActive[35] = PCS_a036_lvBusOv;
  datalayer_extended.tesla.PCS_alertMatrixActive[36] = PCS_a037_resonantTankOc;
  datalayer_extended.tesla.PCS_alertMatrixActive[37] = PCS_a038_claFaulted;
  datalayer_extended.tesla.PCS_alertMatrixActive[38] = PCS_a039_sdModuleClkFault;
  datalayer_extended.tesla.PCS_alertMatrixActive[39] = PCS_a040_dcdcMaxPowerReached;
  datalayer_extended.tesla.PCS_alertMatrixActive[40] = PCS_a041_dcdcOverTemp;
  datalayer_extended.tesla.PCS_alertMatrixActive[41] = PCS_a042_dcdcEnableLineMismatch;
  datalayer_extended.tesla.PCS_alertMatrixActive[42] = PCS_a043_hvBusPrechargeFailure;
  datalayer_extended.tesla.PCS_alertMatrixActive[43] = PCS_a044_12vSupportRegulation;
  datalayer_extended.tesla.PCS_alertMatrixActive[44] = PCS_a045_hvBusLowImpedance;
  datalayer_extended.tesla.PCS_alertMatrixActive[45] = PCS_a046_hvBusHighImpedence;
  datalayer_extended.tesla.PCS_alertMatrixActive[46] = PCS_a047_lvBusLowImpedance;
  datalayer_extended.tesla.PCS_alertMatrixActive[47] = PCS_a048_lvBusHighImpedance;
  datalayer_extended.tesla.PCS_alertMatrixActive[48] = PCS_a049_dcdcTempRationality;
  datalayer_extended.tesla.PCS_alertMatrixActive[49] = PCS_a050_dcdc12VsupportFaulted;
  datalayer_extended.tesla.PCS_alertMatrixActive[50] = PCS_a051_chgIntBusUv;
  datalayer_extended.tesla.PCS_alertMatrixActive[51] = PCS_a052_acVoltageNotPresent;
  datalayer_extended.tesla.PCS_alertMatrixActive[52] = PCS_a053_chgInputVDropHigh;
  datalayer_extended.tesla.PCS_alertMatrixActive[53] = PCS_a054_chgInputVDropTooHigh;
  datalayer_extended.tesla.PCS_alertMatrixActive[54] = PCS_a055_chgLineImedanceHigh;
  datalayer_extended.tesla.PCS_alertMatrixActive[55] = PCS_a056_chgLineImedanceTooHigh;
  datalayer_extended.tesla.PCS_alertMatrixActive[56] = PCS_a057_chgInputOverFreq;
  datalayer_extended.tesla.PCS_alertMatrixActive[57] = PCS_a058_chgInputUnderFreq;
  datalayer_extended.tesla.PCS_alertMatrixActive[58] = PCS_a059_chgInputOvRms;
  datalayer_extended.tesla.PCS_alertMatrixActive[59] = PCS_a060_chgInputOvPeak;
  datalayer_extended.tesla.PCS_alertMatrixActive[60] = PCS_a061_chgVLineRationality;
  datalayer_extended.tesla.PCS_alertMatrixActive[61] = PCS_a062_chgILineRationality;
  datalayer_extended.tesla.PCS_alertMatrixActive[62] = PCS_a063_chgVOutRationality;
  datalayer_extended.tesla.PCS_alertMatrixActive[63] = PCS_a064_chgIOutRationality;
  datalayer_extended.tesla.PCS_alertMatrixActive[64] = PCS_a065_chgPllNotLocked;
  datalayer_extended.tesla.PCS_alertMatrixActive[65] = PCS_a066_dcdcHvRationality;
  datalayer_extended.tesla.PCS_alertMatrixActive[66] = PCS_a067_dcdcLvRationality;
  datalayer_extended.tesla.PCS_alertMatrixActive[67] = PCS_a068_dcdcTankvRationality;
  datalayer_extended.tesla.PCS_alertMatrixActive[68] = PCS_a069_chgPfcLineDidt;
  datalayer_extended.tesla.PCS_alertMatrixActive[69] = PCS_a070_chgPfcLineDvdt;
  datalayer_extended.tesla.PCS_alertMatrixActive[70] = PCS_a071_chgPfcILoopRationality;
  datalayer_extended.tesla.PCS_alertMatrixActive[71] = PCS_a072_cpu2ClaStopped;
  datalayer_extended.tesla.PCS_alertMatrixActive[72] = PCS_a073_unexpectedAcInputVoltage;
  datalayer_extended.tesla.PCS_alertMatrixActive[73] = PCS_a074_hvBusDischargeFailure;
  datalayer_extended.tesla.PCS_alertMatrixActive[74] = PCS_a075_hvBusDischargeTimeout;
  datalayer_extended.tesla.PCS_alertMatrixActive[75] = PCS_a076_dcdcEnDeassertedErr;
  datalayer_extended.tesla.PCS_alertMatrixActive[76] = PCS_a077_microGridEnergyLow;
  datalayer_extended.tesla.PCS_alertMatrixActive[77] = PCS_a078_chgStopDcdcTooHot;
  datalayer_extended.tesla.PCS_alertMatrixActive[78] = PCS_a079_eepromOperationError;
  datalayer_extended.tesla.PCS_alertMatrixActive[79] = PCS_a080_damagedPhaseDetected;
  datalayer_extended.tesla.PCS_alertMatrixActive[80] = PCS_a081_dcdcPchgTimeout;
  datalayer_extended.tesla.PCS_alertMatrixActive[81] = PCS_a082_dcdcPchgUnsafeDiVoltage;
  datalayer_extended.tesla.PCS_alertMatrixActive[82] = PCS_a083_triggerOdin;
  datalayer_extended.tesla.PCS_alertMatrixActive[83] = PCS_a084_dcdcPchgStartVoltages;
  datalayer_extended.tesla.PCS_alertMatrixActive[84] = PCS_a085_dcdcFetsNotSwitching;
  datalayer_extended.tesla.PCS_alertMatrixActive[85] = PCS_a086_dcdcInsufficientCooling;
  datalayer_extended.tesla.PCS_alertMatrixActive[86] = PCS_a087_nvramRecordStatusError;
  datalayer_extended.tesla.PCS_alertMatrixActive[87] = PCS_a088_pchgParameters;
  datalayer_extended.tesla.PCS_alertMatrixActive[88] = PCS_a089_hvBusDischargeIrrational;
  datalayer_extended.tesla.PCS_alertMatrixActive[89] = PCS_a090_expectedAcVoltageSourceMissing;
  datalayer_extended.tesla.PCS_alertMatrixActive[90] = PCS_a091_chgIntBusRationality;
  datalayer_extended.tesla.PCS_alertMatrixActive[91] = PCS_a092_chgPowerLimitedByBusRipple;
  datalayer_extended.tesla.PCS_alertMatrixActive[92] = PCS_a093_powerRailRationality;
  datalayer_extended.tesla.PCS_alertMatrixActive[93] = PCS_a094_pcsDcdcNeedService;
  datalayer_extended.tesla.CP_alertMatrixActive[0] = CP_a001_canRx;
  datalayer_extended.tesla.CP_alertMatrixActive[1] = CP_a002_canTx;
  datalayer_extended.tesla.CP_alertMatrixActive[2] = CP_a003_canError;
  datalayer_extended.tesla.CP_alertMatrixActive[3] = CP_a004_proximityRationality;
  datalayer_extended.tesla.CP_alertMatrixActive[4] = CP_a005_gbdcLiveDisconnect;
  datalayer_extended.tesla.CP_alertMatrixActive[5] = CP_a006_lostCommsBMS;
  datalayer_extended.tesla.CP_alertMatrixActive[6] = CP_a007_watchdog;
  datalayer_extended.tesla.CP_alertMatrixActive[7] = CP_a008_memoryError;
  datalayer_extended.tesla.CP_alertMatrixActive[8] = CP_a009_coverOpen;
  datalayer_extended.tesla.CP_alertMatrixActive[9] = CP_a010_pilotRationality;
  datalayer_extended.tesla.CP_alertMatrixActive[10] = CP_a011_eeprom;
  datalayer_extended.tesla.CP_alertMatrixActive[11] = CP_a012_ledDriver;
  datalayer_extended.tesla.CP_alertMatrixActive[12] = CP_a013_lostCommsGTW;
  datalayer_extended.tesla.CP_alertMatrixActive[13] = CP_a014_lostCommsCHG;
  datalayer_extended.tesla.CP_alertMatrixActive[14] = CP_a015_apsVov;
  datalayer_extended.tesla.CP_alertMatrixActive[15] = CP_a016_apsVuv;
  datalayer_extended.tesla.CP_alertMatrixActive[16] = CP_a017_fiveVov;
  datalayer_extended.tesla.CP_alertMatrixActive[17] = CP_a018_fiveVuv;
  datalayer_extended.tesla.CP_alertMatrixActive[18] = CP_a019_threeVov;
  datalayer_extended.tesla.CP_alertMatrixActive[19] = CP_a020_threeVuv;
  datalayer_extended.tesla.CP_alertMatrixActive[20] = CP_a021_zeroVov;
  datalayer_extended.tesla.CP_alertMatrixActive[21] = CP_a022_zeroVuv;
  datalayer_extended.tesla.CP_alertMatrixActive[22] = CP_a023_gbdcSessionFailed;
  datalayer_extended.tesla.CP_alertMatrixActive[23] = CP_a024_ledsUC;
  datalayer_extended.tesla.CP_alertMatrixActive[24] = CP_a025_ledsOC;
  datalayer_extended.tesla.CP_alertMatrixActive[25] = CP_a026_networkManagement;
  datalayer_extended.tesla.CP_alertMatrixActive[26] = CP_a027_doorSensorOutOfSpec;
  datalayer_extended.tesla.CP_alertMatrixActive[27] = CP_a028_insertEnableMismatch;
  datalayer_extended.tesla.CP_alertMatrixActive[28] = CP_a029_doorClosedProxPilot;
  datalayer_extended.tesla.CP_alertMatrixActive[29] = CP_a030_busOff;
  datalayer_extended.tesla.CP_alertMatrixActive[30] = CP_a031_doorClosedCommandedOpen;
  datalayer_extended.tesla.CP_alertMatrixActive[31] = CP_a032_doorOpenExpectedClosed;
  datalayer_extended.tesla.CP_alertMatrixActive[32] = CP_a033_spiOpen;
  datalayer_extended.tesla.CP_alertMatrixActive[33] = CP_a034_calibrationIncomplete;
  datalayer_extended.tesla.CP_alertMatrixActive[34] = CP_a035_latchMovement_1;
  datalayer_extended.tesla.CP_alertMatrixActive[35] = CP_a036_latchNotDisengaged;
  datalayer_extended.tesla.CP_alertMatrixActive[36] = CP_a037_latchNotEngaged;
  datalayer_extended.tesla.CP_alertMatrixActive[37] = CP_a038_latchNotBlocking;
  datalayer_extended.tesla.CP_alertMatrixActive[38] = CP_a039_latchMovement_2;
  datalayer_extended.tesla.CP_alertMatrixActive[39] = CP_a040_doNotUse;
  datalayer_extended.tesla.CP_alertMatrixActive[40] = CP_a041_doorSensorUnplugged;
  datalayer_extended.tesla.CP_alertMatrixActive[41] = CP_a042_doorAssemblyBroken;
  datalayer_extended.tesla.CP_alertMatrixActive[42] = CP_a043_doorPotIrrational;
  datalayer_extended.tesla.CP_alertMatrixActive[43] = CP_a044_lostCommsHVP;
  datalayer_extended.tesla.CP_alertMatrixActive[44] = CP_a045_lostCommsVCSEC;
  datalayer_extended.tesla.CP_alertMatrixActive[45] = CP_a046_lostCommsEVSE;
  datalayer_extended.tesla.CP_alertMatrixActive[46] = CP_a047_lostCommsVCFRONT;
  datalayer_extended.tesla.CP_alertMatrixActive[47] = CP_a048_lostCommsUI;
  datalayer_extended.tesla.CP_alertMatrixActive[48] = CP_a049_multipleCablesDetected;
  datalayer_extended.tesla.CP_alertMatrixActive[49] = CP_a050_latchNotConnected;
  datalayer_extended.tesla.CP_alertMatrixActive[50] = CP_a051_doorInductiveSensorMIA;
  datalayer_extended.tesla.CP_alertMatrixActive[51] = CP_a052_evseNotSupported;
  datalayer_extended.tesla.CP_alertMatrixActive[52] = CP_a053_proxLatchedNoPilot;
  datalayer_extended.tesla.CP_alertMatrixActive[53] = CP_a054_cableNotSecured;
  datalayer_extended.tesla.CP_alertMatrixActive[54] = CP_a055_chargeStoppedNoPilot;
  datalayer_extended.tesla.CP_alertMatrixActive[55] = CP_a056_proxDisconnected;
  datalayer_extended.tesla.CP_alertMatrixActive[56] = CP_a057_evseFaulted;
  datalayer_extended.tesla.CP_alertMatrixActive[57] = CP_a058_acChargingBlocked;
  datalayer_extended.tesla.CP_alertMatrixActive[58] = CP_a059_swcanError;
  datalayer_extended.tesla.CP_alertMatrixActive[59] = CP_a060_lostCommsPCS;
  datalayer_extended.tesla.CP_alertMatrixActive[60] = CP_a061_uhfReceiverMIA;
  datalayer_extended.tesla.CP_alertMatrixActive[61] = CP_a062_scOutOfService;
  datalayer_extended.tesla.CP_alertMatrixActive[62] = CP_a063_scUpdateInProgress;
  datalayer_extended.tesla.CP_alertMatrixActive[63] = CP_a064_superchargingBlocked;
  datalayer_extended.tesla.CP_alertMatrixActive[64] = CP_a065_selfTestFailed;
  datalayer_extended.tesla.CP_alertMatrixActive[65] = CP_a066_proxLatchedIdlePilot;
  datalayer_extended.tesla.CP_alertMatrixActive[66] = CP_a067_gbdcConnFault;
  datalayer_extended.tesla.CP_alertMatrixActive[67] = CP_a068_doorSensorMismatch;
  datalayer_extended.tesla.CP_alertMatrixActive[68] = CP_a069_doorInductiveSensorError;
  datalayer_extended.tesla.CP_alertMatrixActive[69] = CP_a070_doorInductiveSensorReset;
  datalayer_extended.tesla.CP_alertMatrixActive[70] = CP_a071_exiDecodeFailure;
  datalayer_extended.tesla.CP_alertMatrixActive[71] = CP_a072_v2gEvccTimeout;
  datalayer_extended.tesla.CP_alertMatrixActive[72] = CP_a073_iecComboShutdown;
  datalayer_extended.tesla.CP_alertMatrixActive[73] = CP_a074_failedToEstablishV2gComm;
  datalayer_extended.tesla.CP_alertMatrixActive[74] = CP_a075_v2gCommsFailure;
  datalayer_extended.tesla.CP_alertMatrixActive[75] = CP_a076_LDC1612errorWatchdog;
  datalayer_extended.tesla.CP_alertMatrixActive[76] = CP_a077_invalidMacAddress;
  datalayer_extended.tesla.CP_alertMatrixActive[77] = CP_a078_latchNotDisengagedCold;
  datalayer_extended.tesla.CP_alertMatrixActive[78] = CP_a079_cableNotSecuredCold;
  datalayer_extended.tesla.CP_alertMatrixActive[79] = CP_a080_taskStackOverflow;
  datalayer_extended.tesla.CP_alertMatrixActive[80] = CP_a081_swException;
  datalayer_extended.tesla.CP_alertMatrixActive[81] = CP_a082_powerOnReset;
  datalayer_extended.tesla.CP_alertMatrixActive[82] = CP_a083_watchdogTraceData;
  datalayer_extended.tesla.CP_alertMatrixActive[83] = CP_a084_proximityPeDisconnected;
  datalayer_extended.tesla.CP_alertMatrixActive[84] = CP_a085_dcPinTempFaulted;
  datalayer_extended.tesla.CP_alertMatrixActive[85] = CP_a086_dcPinTempIrrational;
  datalayer_extended.tesla.CP_alertMatrixActive[86] = CP_a087_dcTempModelFault;
  datalayer_extended.tesla.CP_alertMatrixActive[87] = CP_a088_dcTempModelDeviation;
  datalayer_extended.tesla.CP_alertMatrixActive[88] = CP_a089_plcConfigMismatch;
  datalayer_extended.tesla.CP_alertMatrixActive[89] = CP_a090_ccsEvseLowIso;
  datalayer_extended.tesla.CP_alertMatrixActive[90] = CP_a091_wrongSuperchargerHandle;
  datalayer_extended.tesla.CP_alertMatrixActive[91] = CP_a092_modemAppLoadFailed;
  datalayer_extended.tesla.CP_alertMatrixActive[92] = CP_a093_modemLoadedWithReset;
  datalayer_extended.tesla.CP_alertMatrixActive[93] = CP_a094_inductiveResetSuccessful;
  datalayer_extended.tesla.CP_alertMatrixActive[94] = CP_a095_thermalDcLimitActive;
  datalayer_extended.tesla.CP_alertMatrixActive[95] = CP_a096_pilotWake;

  //Safety checks for CAN message sending
  //A diagnostic ACCESSORY hold (HVC backup) keeps the pack awake while the inverter is out of the
  //picture. A fault or an equipment stop still overrides it and walks 0x221 down as usual - neither
  //is a moment to be telling the pack the car is on.
  const bool diagnostic_hold_blocked =
      (datalayer.system.status.system_status == FAULT) || datalayer.system.info.equipment_stop_active;
  if (diagnostic_hold_blocked) {
    release_hvc_backup_hold();
  }
  const bool diagnostic_accessory_hold = hvc_bk_accessory_held && !diagnostic_hold_blocked;
  if ((datalayer.system.status.inverter_allows_contactor_closing == true) && !diagnostic_hold_blocked) {
    // Carry on: 0x221 DRIVE (or user-forced ACCESSORY) state & reset power down timer
    vehicleState = accessory_mode_forced ? ACCESSORY : CAR_DRIVE;
    powerDownSeconds = 9;
  } else if (diagnostic_accessory_hold) {
    // Paused, but a diagnostic needs the pack awake. ODIN's own procedures run the car in ACCESSORY
    // (BMS SUPPORT) and command the contactors over UDS instead of letting the car switch off.
    vehicleState = ACCESSORY;
    powerDownSeconds = 9;
  } else {
    // Faulted state, or inverter blocks contactor closing
    // Shut down: 0x221 ACCESSORY state for 3 seconds, followed by GOING_DOWN, then OFF
    if (powerDownSeconds <= 9 && powerDownSeconds > 6) {
      vehicleState = ACCESSORY;
      powerDownSeconds--;
    }
    if (powerDownSeconds <= 6 && powerDownSeconds > 3) {
      vehicleState = GOING_DOWN;
      powerDownSeconds--;
    }
    if (powerDownSeconds <= 3 && powerDownSeconds > 0) {
      vehicleState = CAR_OFF;
      powerDownSeconds--;
    }
  }
  datalayer_extended.tesla.vehicleState = vehicleState;
  datalayer_extended.tesla.balance_at_rest_enabled = balance_at_rest_enabled;

  printFaultCodesIfActive();
  logging.printf("Contactor State: ");
  logging.printf(getBMSContactorState(battery_contactor));  // Display what state the BMS thinks the contactors are in
  logging.printf(" HVIL: ");
  logging.printf(getHvilStatusState(battery_hvil_status));
  logging.printf(" NegState: ");
  logging.printf(getContactorState(battery_packContNegativeState));
  logging.printf(" PosState: ");
  logging.println(getContactorState(battery_packContPositiveState));
  logging.printf("Cont. setState: ");
  logging.printf(
      getContactorText(battery_packContactorSetState));  // Display what state the HVP has set the contactors to be in
  logging.printf(" Closing blocked: ");
  logging.printf(getNoYes(battery_packCtrsClosingBlocked));
  if (battery_packContactorSetState == 5) {
    logging.printf(" (already CLOSED)");
  }
  logging.printf(", Pyrotest: ");
  logging.println(getNoYes(battery_pyroTestInProgress));

  logging.printf("HV: %.2f V, 12V: %.2f V, 12V current: %.2f A.\n", (battery_dcdcHvBusVolt * 0.146484),
                 (battery_dcdcLvBusVolt * 0.0390625), (battery_dcdcLvOutputCurrent * 0.1));
}

/* Mirrors the Nissan Leaf handler: fire the events only on a real edge, and never
 * announce an end for the initial UNKNOWN -> READY settle at boot. */
void TeslaBattery::set_balancing_status(balancing_status_enum new_status) {
  if (new_status == datalayer_battery->status.balancing_status) {
    return;
  }
  if (new_status == BALANCING_STATUS_ACTIVE) {
    set_event_latched(EVENT_BALANCING_START, 0);
  } else if (datalayer_battery->status.balancing_status == BALANCING_STATUS_ACTIVE) {
    set_event(EVENT_BALANCING_END, 0);
  }
  datalayer_battery->status.balancing_status = new_status;
}

void TeslaBattery::handle_incoming_can_frame(CAN_frame rx_frame) {
  // mux, temp, mux0_read, mux1_read are instance member variables (TESLA-BATTERY.h)

  if (handle_incoming_uds_can_frame(rx_frame)) {
    return;
  }

  switch (rx_frame.ID) {
    case 0x352:  // 850 BMS_energyStatus newer BMS
      datalayer_battery->status.CAN_battery_still_alive = CAN_STILL_ALIVE;
      mux = ((rx_frame.data.u8[0]) & 0x03);  //BMS_energyStatusIndex M : 0|2@1+ (1,0) [0|0] ""  X
      if (mux == 0) {
        battery_nominal_full_pack_energy_m0 =
            (((rx_frame.data.u8[3]) << 8) |
             rx_frame.data.u8[2]);  //16|16@1+ (0.02,0) [0|0] "kWh"//to datalayer_extended
        battery_nominal_energy_remaining_m0 =
            (((rx_frame.data.u8[5]) << 8) |
             rx_frame.data.u8[4]);  //32|16@1+ (0.02,0) [0|0] "kWh"//to datalayer_extended
        battery_ideal_energy_remaining_m0 =
            (((rx_frame.data.u8[7]) << 8) |
             rx_frame.data.u8[6]);  //48|16@1+ (0.02,0) [0|0] "kWh"//to datalayer_extended
        mux0_read = true;           //Set flag to true
      }
      if (mux == 1) {
        battery_fully_charged = (rx_frame.data.u8[1] & 0x01);  //BMS_fullChargeComplete : 15|1@1+ (1,0) [0|1]//noYes
        battery_energy_buffer_m1 =
            ((rx_frame.data.u8[3] << 8) | rx_frame.data.u8[2]);  //16|16@1+ (0.01,0) [0|0] "kWh"//to datalayer_extended
        battery_expected_energy_remaining_m1 =
            ((rx_frame.data.u8[5] << 8) | rx_frame.data.u8[4]);  //32|16@1+ (0.02,0) [0|0] "kWh"//to datalayer_extended
        battery_energy_to_charge_complete_m1 =
            ((rx_frame.data.u8[7] << 8) | rx_frame.data.u8[6]);  //48|16@1+ (0.02,0) [0|0] "kWh"//to datalayer_extended
        mux1_read = true;                                        //Set flag to true
      }
      if (mux == 2) {
        //BMS_userFacingSoe 52|11@1+ (0.05,0) "%" - added in 2024.20 firmware (bytes 6-7 are 0 on older
        //packs, e.g. 02 26 02 20 02 80 00 00), 2047 = SNA. Consumed when user_selected_tesla_soc_source = 4.
        battery_userFacingSoe = (((rx_frame.data.u8[7] << 8) | rx_frame.data.u8[6]) >> 4) & 0x7FF;
      }
      if (mux0_read && mux1_read) {
        mux0_read = false;
        mux1_read = false;
        BMS352_mux = true;  //Set flag to true
        break;
      }
      if (mux0_read || mux1_read || BMS352_mux) {
        //Skip older BMS frame parsing
        break;
      }
      // Older BMS <2021 without mux
      battery_nominal_full_pack_energy =  //BMS_nominalFullPackEnergy : 0|11@1+ (0.1,0) [0|204.6] "kWh" //((_d[1] & (0x07U)) << 8) | (_d[0] & (0xFFU));
          (((rx_frame.data.u8[1] & 0x07) << 8) | (rx_frame.data.u8[0]));  //Example 752 (75.2kWh)
      battery_nominal_energy_remaining =  //BMS_nominalEnergyRemaining : 11|11@1+ (0.1,0) [0|204.6] "kWh" //((_d[2] & (0x3FU)) << 5) | ((_d[1] >> 3) & (0x1FU));
          (((rx_frame.data.u8[2] & 0x3F) << 5) | ((rx_frame.data.u8[1] & 0x1F) >> 3));  //Example 1247 * 0.1 = 124.7kWh
      battery_expected_energy_remaining =  //BMS_expectedEnergyRemaining : 22|11@1+ (0.1,0) [0|204.6] "kWh"// ((_d[4] & (0x01U)) << 10) | ((_d[3] & (0xFFU)) << 2) | ((_d[2] >> 6) & (0x03U));
          (((rx_frame.data.u8[4] & 0x01) << 10) | (rx_frame.data.u8[3] << 2) |
           ((rx_frame.data.u8[2] & 0x03) >> 6));  //Example 622 (62.2kWh)
      battery_ideal_energy_remaining =  //BMS_idealEnergyRemaining : 33|11@1+ (0.1,0) [0|204.6] "kWh" //((_d[5] & (0x0FU)) << 7) | ((_d[4] >> 1) & (0x7FU));
          (((rx_frame.data.u8[5] & 0x0F) << 7) | ((rx_frame.data.u8[4] & 0x7F) >> 1));  //Example 311 * 0.1 = 31.1kWh
      battery_energy_to_charge_complete =  // BMS_energyToChargeComplete : 44|11@1+ (0.1,0) [0|204.6] "kWh"// ((_d[6] & (0x7FU)) << 4) | ((_d[5] >> 4) & (0x0FU));
          (((rx_frame.data.u8[6] & 0x7F) << 4) | ((rx_frame.data.u8[5] & 0x0F) << 4));  //Example 147 * 0.1 = 14.7kWh
      battery_energy_buffer =  //BMS_energyBuffer : 55|8@1+ (0.1,0) [0|25.4] "kWh"// ((_d[7] & (0x7FU)) << 1) | ((_d[6] >> 7) & (0x01U));
          (((rx_frame.data.u8[7] & 0xFE) >> 1) | ((rx_frame.data.u8[6] & 0x80) >> 7));  //Example 1 * 0.1 = 0
      battery_fully_charged =  //BMS_fullChargeComplete : 63|1@1+ (1,0) [0|1] ""//((_d[7] >> 7) & (0x01U));
          ((rx_frame.data.u8[7] >> 7) & 0x01);  //noYes
      break;
    case 0x20A:  //522 HVP_contactorState:
      datalayer_battery->status.CAN_battery_still_alive = CAN_STILL_ALIVE;
      battery_packContNegativeState =
          (rx_frame.data.u8[0] & 0x07);  //(_d[0] & (0x07U)); 0|3@1+ (1,0) [0|7] //contactorState
      battery_packContPositiveState =
          (rx_frame.data.u8[0] & 0x38) >> 3;  //((_d[0] >> 3) & (0x07U)); 3|3@1+ (1,0) [0|7] //contactorState
      //battery_contactor = (rx_frame.data.u8[1] & 0x0F);  // 8|4@1+ (1,0) [0|9] //contactorText
      battery_packContactorSetState =
          (rx_frame.data.u8[1] & 0x0F);  //(_d[1] & (0x0FU)); 8|4@1+ (1,0) [0|9] //contactorState
      battery_packCtrsClosingBlocked =
          (rx_frame.data.u8[4] & 0x08) >> 3;  //((_d[4] >> 3) & (0x01U)); 35|1@1+ (1,0) [0|1] //noYes
      battery_pyroTestInProgress =
          (rx_frame.data.u8[4] & 0x20) >> 5;  //((_d[4] >> 5) & (0x01U));//37|1@1+ (1,0) [0|1] //noYes
      battery_hvil_status =
          (rx_frame.data.u8[5] & 0x0F);  //(_d[5] & (0x0FU));   //40|4@1+ (1,0) [0|9] //hvilStatusState
      battery_packCtrsOpenNowRequested = ((rx_frame.data.u8[4] >> 1) & (0x01U));  //33|1@1+ (1,0) [0|1] //noYes
      battery_packCtrsOpenRequested = ((rx_frame.data.u8[4] >> 2) & (0x01U));     //34|1@1+ (1,0) [0|1] //noYes
      battery_packCtrsRequestStatus = ((rx_frame.data.u8[3] >> 6) & (0x03U));     //30|2@1+ (1,0) [0|2] //HVP_contactor
      battery_packCtrsResetRequestRequired = (rx_frame.data.u8[4] & (0x01U));     //32|1@1+ (1,0) [0|1] //noYes
      battery_dcLinkAllowedToEnergize = ((rx_frame.data.u8[4] >> 4) & (0x01U));   //36|1@1+ (1,0) [0|1] //noYes
      battery_fcContNegativeAuxOpen = ((rx_frame.data.u8[0] >> 7) & (0x01U));     //7|1@1+ (1,0) [0|1] ""  Receiver
      battery_fcContNegativeState = ((rx_frame.data.u8[1] >> 4) & (0x07U));       //12|3@1+ (1,0) [0|7] ""  Receiver
      battery_fcContPositiveAuxOpen = ((rx_frame.data.u8[0] >> 6) & (0x01U));     //6|1@1+ (1,0) [0|1] ""  Receiver
      battery_fcContPositiveState = (rx_frame.data.u8[2] & (0x07U));              //16|3@1+ (1,0) [0|7] ""  Receiver
      battery_fcContactorSetState = ((rx_frame.data.u8[2] >> 3) & (0x0FU));       //19|4@1+ (1,0) [0|9] ""  Receiver
      battery_fcCtrsClosingAllowed = ((rx_frame.data.u8[3] >> 5) & (0x01U));      //29|1@1+ (1,0) [0|1] ""  Receiver
      battery_fcCtrsOpenNowRequested = ((rx_frame.data.u8[3] >> 3) & (0x01U));    //27|1@1+ (1,0) [0|1] ""  Receiver
      battery_fcCtrsOpenRequested = ((rx_frame.data.u8[3] >> 4) & (0x01U));       //28|1@1+ (1,0) [0|1] ""  Receiver
      battery_fcCtrsRequestStatus = (rx_frame.data.u8[3] & (0x03U));              //24|2@1+ (1,0) [0|2] ""  Receiver
      battery_fcCtrsResetRequestRequired = ((rx_frame.data.u8[3] >> 2) & (0x01U));  //26|1@1+ (1,0) [0|1] ""  Receiver
      battery_fcLinkAllowedToEnergize = ((rx_frame.data.u8[5] >> 4) & (0x03U));     //44|2@1+ (1,0) [0|2] ""  Receiver
      break;
    case 0x212:  //530 BMS_status: 8
      datalayer_battery->status.CAN_battery_still_alive = CAN_STILL_ALIVE;
      BMS_hvacPowerRequest = (rx_frame.data.u8[0] & (0x01U));
      BMS_notEnoughPowerForDrive = ((rx_frame.data.u8[0] >> 1) & (0x01U));
      BMS_notEnoughPowerForSupport = ((rx_frame.data.u8[0] >> 2) & (0x01U));
      BMS_preconditionAllowed = ((rx_frame.data.u8[0] >> 3) & (0x01U));
      BMS_updateAllowed = ((rx_frame.data.u8[0] >> 4) & (0x01U));
      BMS_activeHeatingWorthwhile = ((rx_frame.data.u8[0] >> 5) & (0x01U));
      BMS_cpMiaOnHvs = ((rx_frame.data.u8[0] >> 6) & (0x01U));
      BMS_contactorState = (rx_frame.data.u8[1] &
                            (0x07U));  //0 "SNA" 1 "OPEN" 2 "OPENING" 3 "CLOSING" 4 "CLOSED" 5 "WELDED" 6 "BLOCKED" ;
      battery_contactor = BMS_contactorState;
      //BMS_state = // Original code from older DBCs
      //((rx_frame.data.u8[1] >> 3) &
      //(0x0FU));  //0 "STANDBY" 1 "DRIVE" 2 "SUPPORT" 3 "CHARGE" 4 "FEIM" 5 "CLEAR_FAULT" 6 "FAULT" 7 "WELD" 8 "TEST" 9 "SNA" ;
      BMS_state = static_cast<uint8_t>(extract_signal_value(rx_frame.data.u8, 32, 4));  //32|4@1+
      //0 "STANDBY" 1 "DRIVE" 2 "SUPPORT" 3 "CHARGE" 4 "FEIM" 5 "CLEAR_FAULT" 6 "FAULT" 7 "WELD" 8 "TEST" 9 "SNA" 10 "BMS_DIAG";
      BMS_hvState = (rx_frame.data.u8[2] & (0x07U));
      //0 "DOWN" 1 "COMING_UP" 2 "GOING_DOWN" 3 "UP_FOR_DRIVE" 4 "UP_FOR_CHARGE" 5 "UP_FOR_DC_CHARGE" 6 "UP" ;
      BMS_isolationResistance =
          ((rx_frame.data.u8[3] & (0x1FU)) << 5) |
          ((rx_frame.data.u8[2] >> 3) & (0x1FU));  //19|10@1+ (10,0) [0|0] "kOhm"/to datalayer_extended
      datalayer_battery->status.insulation_resistance_kOhm = BMS_isolationResistance * 10;
      datalayer_battery->status.insulation_resistance_available = true;
      //BMS_chargeRequest = ((rx_frame.data.u8[3] >> 5) & (0x01U));
      BMS_chargeRequest = static_cast<bool>(extract_signal_value(rx_frame.data.u8, 29, 1));
      BMS_keepWarmRequest = ((rx_frame.data.u8[3] >> 6) & (0x01U));
      BMS_uiChargeStatus = static_cast<uint8_t>(extract_signal_value(rx_frame.data.u8, 11, 3));
      //BMS_uiChargeStatus =
      //(rx_frame.data.u8[4] &
      //(0x07U));
      // 0 "DISCONNECTED" 1 "NO_POWER" 2 "ABOUT_TO_CHARGE" 3 "CHARGING" 4 "CHARGE_COMPLETE" 5 "CHARGE_STOPPED" ;
      BMS_diLimpRequest = ((rx_frame.data.u8[4] >> 3) & (0x01U));
      BMS_okToShipByAir = ((rx_frame.data.u8[4] >> 4) & (0x01U));
      BMS_okToShipByLand = ((rx_frame.data.u8[4] >> 5) & (0x01U));
      BMS_chgPowerAvailable = ((rx_frame.data.u8[6] & (0x01U)) << 10) | ((rx_frame.data.u8[5] & (0xFFU)) << 2) |
                              ((rx_frame.data.u8[4] >> 6) & (0x03U));  //38|11@1+ (0.125,0) [0|0] "kW"
      BMS_chargeRetryCount = ((rx_frame.data.u8[6] >> 1) & (0x0FU));
      BMS_pcsPwmEnabled = ((rx_frame.data.u8[6] >> 5) & (0x01U));
      BMS_ecuLogUploadRequest = ((rx_frame.data.u8[6] >> 6) & (0x03U));
      BMS_minPackTemperature = (rx_frame.data.u8[7] & (0xFFU));  //56|8@1+ (0.5,-40) [0|0] "DegC
      break;
    case 0x224:  //548 PCS_dcdcStatus:
      datalayer_battery->status.CAN_battery_still_alive = CAN_STILL_ALIVE;
      PCS_dcdcPrechargeStatus = (rx_frame.data.u8[0] & (0x03U));              //0 "IDLE" 1 "ACTIVE" 2 "FAULTED" ;
      PCS_dcdc12VSupportStatus = ((rx_frame.data.u8[0] >> 2) & (0x03U));      //0 "IDLE" 1 "ACTIVE" 2 "FAULTED"
      PCS_dcdcHvBusDischargeStatus = ((rx_frame.data.u8[0] >> 4) & (0x03U));  //0 "IDLE" 1 "ACTIVE" 2 "FAULTED"
      PCS_dcdcMainState =
          ((rx_frame.data.u8[1] & (0x03U)) << 2) |
          ((rx_frame.data.u8[0] >> 6) &
           (0x03U));  //0 "STANDBY" 1 "12V_SUPPORT_ACTIVE" 2 "PRECHARGE_STARTUP" 3 "PRECHARGE_ACTIVE" 4 "DIS_HVBUS_ACTIVE" 5 "SHUTDOWN" 6 "FAULTED" ;
      PCS_dcdcSubState =
          ((rx_frame.data.u8[1] >> 2) &
           (0x1FU));  //0 "PWR_UP_INIT" 1 "STANDBY" 2 "12V_SUPPORT_ACTIVE" 3 "DIS_HVBUS" 4 "PCHG_FAST_DIS_HVBUS" 5 "PCHG_SLOW_DIS_HVBUS" 6 "PCHG_DWELL_CHARGE" 7 "PCHG_DWELL_WAIT" 8 "PCHG_DI_RECOVERY_WAIT" 9 "PCHG_ACTIVE" 10 "PCHG_FLT_FAST_DIS_HVBUS" 11 "SHUTDOWN" 12 "12V_SUPPORT_FAULTED" 13 "DIS_HVBUS_FAULTED" 14 "PCHG_FAULTED" 15 "CLEAR_FAULTS" 16 "FAULTED" 17 "NUM" ;
      PCS_dcdcFaulted = ((rx_frame.data.u8[1] >> 7) & (0x01U));
      PCS_dcdcOutputIsLimited = ((rx_frame.data.u8[3] >> 4) & (0x01U));
      PCS_dcdcMaxOutputCurrentAllowed = ((rx_frame.data.u8[5] & (0x01U)) << 11) |
                                        ((rx_frame.data.u8[4] & (0xFFU)) << 3) |
                                        ((rx_frame.data.u8[3] >> 5) & (0x07U));  //29|12@1+ (0.1,0) [0|0] "A"
      PCS_dcdcPrechargeRtyCnt = ((rx_frame.data.u8[5] >> 1) & (0x07U));          //Retry Count
      PCS_dcdc12VSupportRtyCnt = ((rx_frame.data.u8[5] >> 4) & (0x0FU));         //Retry Count
      PCS_dcdcDischargeRtyCnt = (rx_frame.data.u8[6] & (0x0FU));                 //Retry Count
      PCS_dcdcPwmEnableLine = ((rx_frame.data.u8[6] >> 4) & (0x01U));
      PCS_dcdcSupportingFixedLvTarget = ((rx_frame.data.u8[6] >> 5) & (0x01U));
      PCS_ecuLogUploadRequest = ((rx_frame.data.u8[6] >> 6) & (0x03U));
      PCS_dcdcPrechargeRestartCnt = (rx_frame.data.u8[7] & (0x07U));
      PCS_dcdcInitialPrechargeSubState =
          ((rx_frame.data.u8[7] >> 3) &
           (0x1FU));  //0 "PWR_UP_INIT" 1 "STANDBY" 2 "12V_SUPPORT_ACTIVE" 3 "DIS_HVBUS" 4 "PCHG_FAST_DIS_HVBUS" 5 "PCHG_SLOW_DIS_HVBUS" 6 "PCHG_DWELL_CHARGE" 7 "PCHG_DWELL_WAIT" 8 "PCHG_DI_RECOVERY_WAIT" 9 "PCHG_ACTIVE" 10 "PCHG_FLT_FAST_DIS_HVBUS" 11 "SHUTDOWN" 12 "12V_SUPPORT_FAULTED" 13 "DIS_HVBUS_FAULTED" 14 "PCHG_FAULTED" 15 "CLEAR_FAULTS" 16 "FAULTED" 17 "NUM" ;
      break;
    case 0x252:  //Limit //594 BMS_powerAvailable:
      datalayer_battery->status.CAN_battery_still_alive = CAN_STILL_ALIVE;
      BMS_maxRegenPower = ((rx_frame.data.u8[1] << 8) |
                           rx_frame.data.u8[0]);  //0|16@1+ (0.01,0) [0|655.35] "kW"  //Example 4715 * 0.01 = 47.15kW
      BMS_maxDischargePower =
          ((rx_frame.data.u8[3] << 8) |
           rx_frame.data.u8[2]);  //16|16@1+ (0.013,0) [0|655.35] "kW"  //Example 2009 * 0.013 = 26.117???
      BMS_maxStationaryHeatPower =
          (((rx_frame.data.u8[5] & 0x03) << 8) |
           rx_frame.data.u8[4]);  //32|10@1+ (0.01,0) [0|10.23] "kW"  //Example 500 * 0.01 = 5kW
      BMS_hvacPowerBudget =
          (((rx_frame.data.u8[7] << 6) |
            ((rx_frame.data.u8[6] & 0xFC) >> 2)));  //50|10@1+ (0.02,0) [0|20.46] "kW"  //Example 1000 * 0.02 = 20kW?
      BMS_notEnoughPowerForHeatPump =
          ((rx_frame.data.u8[5] >> 2) & (0x01U));  //BMS_notEnoughPowerForHeatPump : 42|1@1+ (1,0) [0|1] ""  Receiver
      BMS_powerLimitState =
          (rx_frame.data.u8[6] &
           (0x01U));  //BMS_powerLimitsState : 48|1 in 2021.24.4..2025.14.7; moved to bit 42 only at
      //2025.20.3+. The bench pack's HVBMS bootloader (0xF180 identity, packid) is 0x31ca5286, the
      //pre-2024.44.3.1 boot -> firmware <=2024.38.7 -> bit 48 is correct here. Reading bit 42 on this
      //pack would return notEnoughPowerForHeatPump instead. Revisit if a 2025.20.3+ pack is used.
      BMS_inverterTQF = ((rx_frame.data.u8[7] >> 4) & (0x03U));
      break;
    case 0x132:  //battery amps/volts //HVBattAmpVolt  (tesla-m3-pack-findings fw 2019.20.4.2 names 0x132 = BMS_hvBusStatus)
      datalayer_battery->status.CAN_battery_still_alive = CAN_STILL_ALIVE;
      battery_volts = ((rx_frame.data.u8[1] << 8) | rx_frame.data.u8[0]) /
                      10;  //0|16@1+ (0.01,0) [0|655.35] "V" -> dV. Integer /10 (was *0.1): identical result
                           //for the unsigned raw, avoids software double emulation on this ~10Hz frame.
      battery_amps =
          ((rx_frame.data.u8[3] << 8) |
           rx_frame.data.u8
               [2]);  //SmoothBattCurrent : 16|16@1- (-0.1,0) [-3276.7|3276.7] "A"//Example 65492 (-4.3A) OR 225 (22.5A)
      //RawBattCurrent (32|16@1- (-0.05,822) "A") is not consumed anywhere, so it is not decoded.
      battery_charge_time_remaining =
          (((rx_frame.data.u8[7] & 0x0F) << 8) |
           rx_frame.data.u8[6]);  //ChargeHoursRemaining : 48|12@1+ (1,0) [0|4095] "Min"  //Example 228 * 0.1 = 22.8min
      if (battery_charge_time_remaining == 4095) {
        battery_charge_time_remaining = 0;
      }
      break;
    case 0x3D2:  //TotalChargeDischarge:
      datalayer_battery->status.CAN_battery_still_alive = CAN_STILL_ALIVE;
      battery_total_discharge = ((rx_frame.data.u8[3] << 24) | (rx_frame.data.u8[2] << 16) |
                                 (rx_frame.data.u8[1] << 8) | rx_frame.data.u8[0]);
      //0|32@1+ (0.001,0) [0|4294970] "kWh"
      battery_total_charge = ((rx_frame.data.u8[7] << 24) | (rx_frame.data.u8[6] << 16) | (rx_frame.data.u8[5] << 8) |
                              rx_frame.data.u8[4]);
      //32|32@1+ (0.001,0) [0|4294970] "kWh"
      break;
    case 0x372:  //882 BMS_log1: balancing state lives in mux 3
      datalayer_battery->status.CAN_battery_still_alive = CAN_STILL_ALIVE;
      mux = rx_frame.data.u8[0];  //BMS_log1MuxId : 0|8@1+ (1,0)
      if (mux == 3) {
        //BMS_balanceState m3 : 8|4@1+ (1,0). Same bit position in the 2025.20.8 and
        //2026.8.3 DBCs. No VAL_ table is published for this signal in any of them,
        //so the meanings below come from logged behaviour rather than from Tesla:
        //  1  balancing engaged -- the only state where BMS_brickBleedByAhMin is
        //     non-zero, and BMS_cumulativeBleedTime advances 1 s per second.
        //  12 voltage-sense-harness test -- lasts exactly 2 s, contactors open, and
        //     drives one brick ~250 mV away from the pack. That is injected stimulus
        //     from switching the bleed FETs, not a real imbalance.
        //  0  not balancing.
        //Anything else means this firmware does not match the frame layout we know,
        //so flag it rather than guess. That keeps a mis-decode visible instead of
        //silently reporting a wrong balancing state.
        BMS_balanceState = (rx_frame.data.u8[1] & 0x0F);
        BMS_vshTestNeeded = ((rx_frame.data.u8[1] >> 4) & 0x01);  //BMS_vshTestNeeded m3 : 12|1@1+
        BMS_balanceStateUnknown = (BMS_balanceState != BALANCE_STATE_IDLE) &&
                                  (BMS_balanceState != BALANCE_STATE_BALANCING) &&
                                  (BMS_balanceState != BALANCE_STATE_VSH_TEST);
      }
      break;
    case 0x3B2:  //946 BMS_log2: balancing accumulators
      datalayer_battery->status.CAN_battery_still_alive = CAN_STILL_ALIVE;
      mux = (rx_frame.data.u8[0] & 0x7F);  //BMS_log2MuxId : 0|7@1+ (1,0)
      if (mux == 35) {
        //BMS_balanceTimeThisCycle m35 : 24|16@1+ (0.5,0) "s". Resets when an episode
        //starts, but is only refreshed every 30 s, so it reads stale just after one
        //begins. Do not use it to decide whether balancing is active.
        BMS_balanceTimeThisCycle_s = (((rx_frame.data.u8[4] << 8) | rx_frame.data.u8[3]) / 2);
      } else if (mux == 39) {
        //BMS_cumulativeBleedTime m39 : 8|24@1+ (1,0) "s"
        BMS_cumulativeBleedTime_s = ((uint32_t)rx_frame.data.u8[3] << 16) |
                                    ((uint32_t)rx_frame.data.u8[2] << 8) | rx_frame.data.u8[1];
      } else if (mux == 65) {
        //BMS_cacEstimationSource m65 : 60|2@1+ (1,0)
        BMS_cacEstimationSource = (rx_frame.data.u8[7] >> 4) & 0x03;
      }
      break;
    case 0x332:  //min/max hist values //BattBrickMinMax:  (tesla-m3-pack-findings fw 2019.20.4.2 names 0x332 = BMS_bmbMinMax)
      datalayer_battery->status.CAN_battery_still_alive = CAN_STILL_ALIVE;
      mux = (rx_frame.data.u8[0] & 0x03);  //BattBrickMultiplexer M : 0|2@1+ (1,0) [0|0] ""

      if (mux == 1)  //Cell voltages
      {
        temp = ((rx_frame.data.u8[1] << 6) | (rx_frame.data.u8[0] >> 2));
        temp = (temp & 0xFFF);
        battery_cell_max_v = temp * 2;
        temp = ((rx_frame.data.u8[3] << 8) | rx_frame.data.u8[2]);
        temp = (temp & 0xFFF);
        battery_cell_min_v = temp * 2;
        cellvoltagesRead = true;
        //BattBrickVoltageMax m1 : 2|12@1+ (0.002,0) [0|0] "V"  Receiver ((_d[1] & (0x3FU)) << 6) | ((_d[0] >> 2) & (0x3FU));
        battery_BrickVoltageMax =
            ((rx_frame.data.u8[1] & (0x3F)) << 6) | ((rx_frame.data.u8[0] >> 2) & (0x3F));  //to datalayer_extended
        //BattBrickVoltageMin m1 : 16|12@1+ (0.002,0) [0|0] "V"  Receiver ((_d[3] & (0x0FU)) << 8) | (_d[2] & (0xFFU));
        battery_BrickVoltageMin =
            ((rx_frame.data.u8[3] & (0x0F)) << 8) | (rx_frame.data.u8[2] & (0xFF));  //to datalayer_extended
        //BattBrickVoltageMaxNum m1 : 32|7@1+ (1,1) [0|0] ""  Receiver
        battery_BrickVoltageMaxNum =
            1 + (rx_frame.data.u8[4] & 0x7F);  //(_d[4] & (0x7FU)); //This cell has highest voltage
        //BattBrickVoltageMinNum m1 : 40|7@1+ (1,1) [0|0] ""  Receiver
        battery_BrickVoltageMinNum =
            1 + (rx_frame.data.u8[5] & 0x7F);  //(_d[5] & (0x7FU)); //This cell has lowest voltage
      }
      if (mux == 0)  //Temperature sensors
      {              //BattBrickTempMax m0 : 16|8@1+ (0.5,-40) [0|0] "C" (_d[2] & (0xFFU));
        battery_max_temp = (rx_frame.data.u8[2] * 5) - 400;  //Temperature values have 40.0*C offset, 0.5*C /bit
        //BattBrickTempMin m0 : 24|8@1+ (0.5,-40) [0|0] "C" (_d[3] & (0xFFU));
        battery_min_temp =
            (rx_frame.data.u8[3] * 5) - 400;  //Multiply by 5 and remove offset to get C+1 (0x61*5=485-400=8.5*C)
        //BattBrickTempMaxNum m0 : 2|4@1+ (1,0) [0|0] "" ((_d[0] >> 2) & (0x0FU));
        battery_BrickTempMaxNum = ((rx_frame.data.u8[0] >> 2) & (0x0F));  //to datalayer_extended
        //BattBrickTempMinNum m0 : 8|4@1+ (1,0) [0|0] "" (_d[1] & (0x0FU));
        battery_BrickTempMinNum = (rx_frame.data.u8[1] & (0x0F));  //to datalayer_extended
        //BattBrickModelTMax m0 : 32|8@1+ (0.5,-40) [0|0] "C" (_d[4] & (0xFFU));
        battery_BrickModelTMax = (rx_frame.data.u8[4] & (0xFFU));  //to datalayer_extended
        //BattBrickModelTMin m0 : 40|8@1+ (0.5,-40) [0|0] "C" (_d[5] & (0xFFU));
        battery_BrickModelTMin = (rx_frame.data.u8[5] & (0xFFU));  //to datalayer_extended
      }
      break;
    case 0x312:  // 786 BMS_thermalStatus
      datalayer_battery->status.CAN_battery_still_alive = CAN_STILL_ALIVE;
      BMS_powerDissipation =
          ((rx_frame.data.u8[1] & (0x03U)) << 8) | (rx_frame.data.u8[0] & (0xFFU));  //0|10@1+ (0.02,0) [0|0] "kW"
      BMS_flowRequest = ((rx_frame.data.u8[2] & (0x01U)) << 6) |
                        ((rx_frame.data.u8[1] >> 2) & (0x3FU));  //10|7@1+ (0.3,0) [0|0] "LPM"
      BMS_inletActiveCoolTargetT = ((rx_frame.data.u8[3] & (0x03U)) << 7) |
                                   ((rx_frame.data.u8[2] >> 1) & (0x7FU));  //17|9@1+ (0.25,-25) [0|0] "DegC"
      BMS_inletPassiveTargetT = ((rx_frame.data.u8[4] & (0x07U)) << 6) |
                                ((rx_frame.data.u8[3] >> 2) & (0x3FU));  //26|9@1+ (0.25,-25) [0|0] "DegC"  X
      BMS_inletActiveHeatTargetT = ((rx_frame.data.u8[5] & (0x0FU)) << 5) |
                                   ((rx_frame.data.u8[4] >> 3) & (0x1FU));  //35|9@1+ (0.25,-25) [0|0] "DegC"
      BMS_packTMin = ((rx_frame.data.u8[6] & (0x1FU)) << 4) |
                     ((rx_frame.data.u8[5] >> 4) & (0x0FU));  //44|9@1+ (0.25,-25) [-25|100] "DegC"
      BMS_packTMax = ((rx_frame.data.u8[7] & (0x3FU)) << 3) |
                     ((rx_frame.data.u8[6] >> 5) & (0x07U));          //53|9@1+ (0.25,-25) [-25|100] "DegC"
      BMS_pcsNoFlowRequest = ((rx_frame.data.u8[7] >> 6) & (0x01U));  // 62|1@1+ (1,0) [0|0] ""
      BMS_noFlowRequest = ((rx_frame.data.u8[7] >> 7) & (0x01U));     //63|1@1+ (1,0) [0|0] ""
      break;
    case 0x2A4:  //676 PCS_thermalStatus
      datalayer_battery->status.CAN_battery_still_alive = CAN_STILL_ALIVE;
      // PCS_chgPhATemp : 0|11@1-
      PCS_chgPhATemp = (int16_t)(rx_frame.data.u8[0] | ((rx_frame.data.u8[1] & 0x07) << 8));
      if (PCS_chgPhATemp & 0x400)
        PCS_chgPhATemp |= 0xF800;
      // PCS_chgPhBTemp : 11|11@1-
      PCS_chgPhBTemp = (int16_t)((rx_frame.data.u8[1] >> 3) | ((rx_frame.data.u8[2] & 0x3F) << 5));
      if (PCS_chgPhBTemp & 0x400)
        PCS_chgPhBTemp |= 0xF800;
      // PCS_chgPhCTemp : 22|11@1-
      PCS_chgPhCTemp =
          (int16_t)((rx_frame.data.u8[2] >> 6) | (rx_frame.data.u8[3] << 2) | ((rx_frame.data.u8[4] & 0x01) << 10));
      if (PCS_chgPhCTemp & 0x400)
        PCS_chgPhCTemp |= 0xF800;
      // PCS_dcdcTemp : 33|11@1-
      PCS_dcdcTemp = (int16_t)((rx_frame.data.u8[4] >> 1) | ((rx_frame.data.u8[5] & 0x0F) << 7));
      if (PCS_dcdcTemp & 0x400)
        PCS_dcdcTemp |= 0xF800;
      // PCS_ambientTemp : 44|11@1-
      PCS_ambientTemp = (int16_t)((rx_frame.data.u8[5] >> 4) | ((rx_frame.data.u8[6] & 0x7F) << 4));
      if (PCS_ambientTemp & 0x400)
        PCS_ambientTemp |= 0xF800;
      break;
      // PCS_thermalStatus signal decoding uses 11-bit signed values (DBC: @1-)
      // with scaling: temperature = raw * 0.1 + 40.0
      // Temperatures below 40C require negative raw values (bit 10 set),
      // so sign extension from 11-bit to 16-bit is required after extraction.
    case 0x2C4:  // 708 PCS_logging: not all frames are listed, just ones relating to dcdc
      datalayer_battery->status.CAN_battery_still_alive = CAN_STILL_ALIVE;
      mux = (rx_frame.data.u8[0] & (0x1FU));
      //PCS_logMessageSelect = (rx_frame.data.u8[0] & (0x1FU));  //0|5@1+ (1,0) [0|0] ""
      if (mux == 6) {
        PCS_dcdcMaxLvOutputCurrent = ((rx_frame.data.u8[4] & (0xFFU)) << 4) |
                                     ((rx_frame.data.u8[3] >> 4) & (0x0FU));  //m6 : 28|12@1+ (0.1,0) [0|0] "A"  X
        PCS_dcdcCurrentLimit = ((rx_frame.data.u8[6] & (0x0FU)) << 8) |
                               (rx_frame.data.u8[5] & (0xFFU));  //m6 : 40|12@1+ (0.1,0) [0|0] "A"  X
        PCS_dcdcLvOutputCurrentTempLimit = ((rx_frame.data.u8[7] & (0xFFU)) << 4) |
                                           ((rx_frame.data.u8[6] >> 4) & (0x0FU));  //m6 : 52|12@1+ (0.1,0) [0|0] "A"  X
      }
      if (mux == 7) {
        PCS_dcdcUnifiedCommand = ((rx_frame.data.u8[1] & (0x7FU)) << 3) |
                                 ((rx_frame.data.u8[0] >> 5) & (0x07U));  //m7 : 5|10@1+ (0.001,0) [0|0] "1"  X
        PCS_dcdcCLAControllerOutput = ((rx_frame.data.u8[3] & (0x03U)) << 8) |
                                      (rx_frame.data.u8[2] & (0xFFU));  //m7 : 16|10@1+ (0.001,0) [0|0] "1"  X
        PCS_dcdcTankVoltage = (int16_t)(((rx_frame.data.u8[4] & 0x1F) << 6) | ((rx_frame.data.u8[3] >> 2) & 0x3F));
        if (PCS_dcdcTankVoltage & 0x400)
          PCS_dcdcTankVoltage |= 0xF800;  //m7 : 26|11@1- (1,0) [0|0] "V"  X
        PCS_dcdcTankVoltageTarget = ((rx_frame.data.u8[5] & (0x7FU)) << 3) |
                                    ((rx_frame.data.u8[4] >> 5) & (0x07U));  // m7 : 37|10@1+ (1,0) [0|0] "V"  X
        PCS_dcdcClaCurrentFreq = ((rx_frame.data.u8[7] & (0x0FU)) << 8) |
                                 (rx_frame.data.u8[6] & (0xFFU));  //P m7 : 48|12@1+ (0.0976563,0) [0|0] "kHz"  X
      }
      if (mux == 8) {
        PCS_dcdcTCommMeasured = ((rx_frame.data.u8[2] & (0xFFU)) << 8) |
                                (rx_frame.data.u8[1] & (0xFFU));  // m8 : 8|16@1- (0.00195313,0) [0|0] "us"  X
        PCS_dcdcShortTimeUs = ((rx_frame.data.u8[4] & (0xFFU)) << 8) |
                              (rx_frame.data.u8[3] & (0xFFU));  // m8 : 24|16@1+ (0.000488281,0) [0|0] "us"  X
        PCS_dcdcHalfPeriodUs = ((rx_frame.data.u8[6] & (0xFFU)) << 8) |
                               (rx_frame.data.u8[5] & (0xFFU));  // m8 : 40|16@1+ (0.000488281,0) [0|0] "us"  X
      }
      if (mux == 18) {
        PCS_dcdcIntervalMaxFrequency = ((rx_frame.data.u8[2] & (0x0FU)) << 8) |
                                       (rx_frame.data.u8[1] & (0xFFU));  // m18 : 8|12@1+ (1,0) [0|0] "kHz"  X
        PCS_dcdcIntervalMaxHvBusVolt = ((rx_frame.data.u8[4] & (0x1FU)) << 8) |
                                       (rx_frame.data.u8[3] & (0xFFU));  //m18 : 24|13@1+ (0.1,0) [0|0] "V"  X
        PCS_dcdcIntervalMaxLvBusVolt = ((rx_frame.data.u8[5] & (0x3FU)) << 3) |
                                       ((rx_frame.data.u8[4] >> 5) & (0x07U));  // m18 : 37|9@1+ (0.1,0) [0|0] "V"  X
        PCS_dcdcIntervalMaxLvOutputCurr = ((rx_frame.data.u8[7] & (0x0FU)) << 8) |
                                          (rx_frame.data.u8[6] & (0xFFU));  //m18 : 48|12@1+ (1,0) [0|0] "A"  X
      }
      if (mux == 19) {
        PCS_dcdcIntervalMinFrequency = ((rx_frame.data.u8[2] & (0x0FU)) << 8) |
                                       (rx_frame.data.u8[1] & (0xFFU));  //m19 : 8|12@1+ (1,0) [0|0] "kHz"  X
        PCS_dcdcIntervalMinHvBusVolt = ((rx_frame.data.u8[4] & (0x1FU)) << 8) |
                                       (rx_frame.data.u8[3] & (0xFFU));  //m19 : 24|13@1+ (0.1,0) [0|0] "V"  X
        PCS_dcdcIntervalMinLvBusVolt = ((rx_frame.data.u8[5] & (0x3FU)) << 3) |
                                       ((rx_frame.data.u8[4] >> 5) & (0x07U));  //m19 : 37|9@1+ (0.1,0) [0|0] "V"  X
        PCS_dcdcIntervalMinLvOutputCurr = ((rx_frame.data.u8[7] & (0x0FU)) << 8) |
                                          (rx_frame.data.u8[6] & (0xFFU));  // m19 : 48|12@1+ (1,0) [0|0] "A"  X
      }
      if (mux == 22) {
        PCS_dcdc12vSupportLifetimekWh = ((rx_frame.data.u8[3] & (0xFFU)) << 16) |
                                        ((rx_frame.data.u8[2] & (0xFFU)) << 8) |
                                        (rx_frame.data.u8[1] & (0xFFU));  // m22 : 8|24@1+ (0.01,0) [0|0] "kWh"  X
      }
      break;
    case 0x401:  // Cell stats  //BrickVoltages
      datalayer_battery->status.CAN_battery_still_alive = CAN_STILL_ALIVE;
      mux = (rx_frame.data.u8[0]);  //MultiplexSelector M : 0|8@1+ (1,0) [0|0] ""
      // mux is an untrusted bus byte (0-255) used to index cell_voltages_mV[mux*3 + 0..2]. A bad or
      // unexpected frame with mux >= MAX_AMOUNT_CELLS/3 would write past the array; guard against it.
      if ((uint16_t)mux * 3 + 2 >= MAX_AMOUNT_CELLS) {
        break;
      }
                                    //StatusFlags : 8|8@1+ (1,0) [0|0] ""
                                    //Brick0 m0 : 16|16@1+ (0.0001,0) [0|0] "V"
                                    //Brick1 m0 : 32|16@1+ (0.0001,0) [0|0] "V"
                                    //Brick2 m0 : 48|16@1+ (0.0001,0) [0|0] "V"
      // brick_volts, mux_zero_counter, mux_max are instance member variables (TESLA-BATTERY.h)
      if (rx_frame.data.u8[1] == 0x2A)  // full frame, 3 cell voltages
      {
        brick_volts = ((rx_frame.data.u8[3] << 8) | rx_frame.data.u8[2]) / 10;
        datalayer_battery->status.cell_voltages_mV[mux * 3] = brick_volts;
        brick_volts = ((rx_frame.data.u8[5] << 8) | rx_frame.data.u8[4]) / 10;
        datalayer_battery->status.cell_voltages_mV[1 + mux * 3] = brick_volts;
        brick_volts = ((rx_frame.data.u8[7] << 8) | rx_frame.data.u8[6]) / 10;
        datalayer_battery->status.cell_voltages_mV[2 + mux * 3] = brick_volts;

        mux_max = (mux > mux_max) ? mux : mux_max;
        if (mux_zero_counter < 2 && mux == 0u) {
          mux_zero_counter++;
          if (mux_zero_counter == 2u) {
            datalayer_battery->info.number_of_cells = 2 + 3 * mux_max + 1;
            mux_zero_counter++;
          }
        }
      } else if (rx_frame.data.u8[1] == 0x02)  // final, partial frame: only 1 cell voltage present
      {
        brick_volts = ((rx_frame.data.u8[3] << 8) | rx_frame.data.u8[2]) / 10;
        datalayer_battery->status.cell_voltages_mV[mux * 3] = brick_volts;

        // This is always the last frame in the sequence, so total cells = mux*3 + 1
        datalayer_battery->info.number_of_cells = mux * 3 + 1;
      }
      break;
    case 0x2d2:  //BMSVAlimits:  (tesla-m3-pack-findings fw 2019.20.4.2 names 0x2D2 = BMS_driveLimits)
      datalayer_battery->status.CAN_battery_still_alive = CAN_STILL_ALIVE;
      BMS_min_voltage = ((rx_frame.data.u8[1] << 8) |
                         rx_frame.data.u8[0]);  //0|16@1+ (0.01,0) [0|430] "V"  //Example 24148mv * 0.01 = 241.48 V
      BMS_max_voltage = ((rx_frame.data.u8[3] << 8) |
                         rx_frame.data.u8[2]);  //16|16@1+ (0.01,0) [0|430] "V"  //Example 40282mv * 0.01 = 402.82 V
      battery_max_charge_current = (((rx_frame.data.u8[5] & 0x3F) << 8) | rx_frame.data.u8[4]) *
                                   0.1;  //32|14@1+ (0.1,0) [0|1638.2] "A"  //Example 1301? * 0.1 = 130.1?
      battery_max_discharge_current = (((rx_frame.data.u8[7] & 0x3F) << 8) | rx_frame.data.u8[6]) *
                                      0.128;  //48|14@1+ (0.128,0) [0|2096.9] "A"  //Example 430? * 0.128 = 55.4?
      break;
    case 0x2b4:  //PCS_dcdcRailStatus:
      datalayer_battery->status.CAN_battery_still_alive = CAN_STILL_ALIVE;
      battery_dcdcLvBusVolt =
          (((rx_frame.data.u8[1] & 0x03) << 8) | rx_frame.data.u8[0]);  //0|10@1+ (0.0390625,0) [0|39.9609] "V"
      battery_dcdcHvBusVolt = (((rx_frame.data.u8[2] & 0x3F) << 6) |
                               ((rx_frame.data.u8[1] & 0xFC) >> 2));  //10|12@1+ (0.146484,0) [0|599.854] "V"
      battery_dcdcLvOutputCurrent =
          (((rx_frame.data.u8[4] & 0x0F) << 8) | rx_frame.data.u8[3]);  //24|12@1+ (0.1,0) [0|400] "A"
      break;
    case 0x292:  //BMS_socStatus
      datalayer_battery->status.CAN_battery_still_alive = CAN_STILL_ALIVE;
      battery_beginning_of_life =
          (((rx_frame.data.u8[6] & 0x03) << 8) | rx_frame.data.u8[5]) * 0.1;          //40|10@1+ (0.1,0) [0|102.3] "kWh"
      battery_soc_min = (((rx_frame.data.u8[1] & 0x03) << 8) | rx_frame.data.u8[0]);  //0|10@1+ (0.1,0) [0|102.3] "%"
      battery_soc_ui =
          (((rx_frame.data.u8[2] & 0x0F) << 6) | ((rx_frame.data.u8[1] & 0xFC) >> 2));  //10|10@1+ (0.1,0) [0|102.3] "%"
      battery_soc_max =
          (((rx_frame.data.u8[3] & 0x3F) << 4) | ((rx_frame.data.u8[2] & 0xF0) >> 4));  //20|10@1+ (0.1,0) [0|102.3] "%"
      battery_soc_ave =
          ((rx_frame.data.u8[4] << 2) | ((rx_frame.data.u8[3] & 0xC0) >> 6));  //30|10@1+ (0.1,0) [0|102.3] "%"
      battery_battTempPct =
          (((rx_frame.data.u8[7] & 0x03) << 6) | (rx_frame.data.u8[6] & 0x3F) >> 2);  //50|8@1+ (0.4,0) [0|100] "%"
      break;
    case 0x392:  //BMS_packConfig  (NOTE: tesla-m3-pack-findings fw 2019.20.4.2 maps CAN 0x392 = EPAS3P_alertMatrix; pack-config frame ID differs between firmwares - verify against target firmware)
      datalayer_battery->status.CAN_battery_still_alive = CAN_STILL_ALIVE;
      mux = (rx_frame.data.u8[0] & (0xFF));
      if (mux == 1) {
        battery_packConfigMultiplexer = (rx_frame.data.u8[0] & (0xff));  //0|8@1+ (1,0) [0|1] ""
        battery_moduleType =
            (rx_frame.data.u8[1] &
             (0x07));  //8|3@1+ (1,0) [0|4] ""//0 "UNKNOWN" 1 "E3_NCT" 2 "E1_NCT" 3 "E3_CT" 4 "E1_CT" 5 "E1_CP" ;//to datalayer_extended
        battery_packMass = (rx_frame.data.u8[2]) + 300;  //16|8@1+ (1,300) [342|469] "kg"
        battery_platformMaxBusVoltage =
            (((rx_frame.data.u8[4] & 0x03) << 8) | (rx_frame.data.u8[3]));  //24|10@1+ (0.1,375) [0|0] "V"
      }
      if (mux == 0) {
        battery_reservedConfig =
            (rx_frame.data.u8[1] &
             (0x1F));  //8|5@1+ (1,0) [0|31] ""//0 "BMS_CONFIG_0" 1 "BMS_CONFIG_1" 10 "BMS_CONFIG_10" 11 "BMS_CONFIG_11" 12 "BMS_CONFIG_12" 13 "BMS_CONFIG_13" 14 "BMS_CONFIG_14" 15 "BMS_CONFIG_15" 16 "BMS_CONFIG_16" 17 "BMS_CONFIG_17" 18 "BMS_CONFIG_18" 19 "BMS_CONFIG_19" 2 "BMS_CONFIG_2" 20 "BMS_CONFIG_20" 21 "BMS_CONFIG_21" 22 "BMS_CONFIG_22" 23 "BMS_CONFIG_23" 24 "BMS_CONFIG_24" 25 "BMS_CONFIG_25" 26 "BMS_CONFIG_26" 27 "BMS_CONFIG_27" 28 "BMS_CONFIG_28" 29 "BMS_CONFIG_29" 3 "BMS_CONFIG_3" 30 "BMS_CONFIG_30" 31 "BMS_CONFIG_31" 4 "BMS_CONFIG_4" 5 "BMS_CONFIG_5" 6 "BMS_CONFIG_6" 7 "BMS_CONFIG_7" 8 "BMS_CONFIG_8" 9 "BMS_CONFIG_9" ;
      }
      break;
    case 0x7AA:  //1962 HVP_debugMessage:
      datalayer_battery->status.CAN_battery_still_alive = CAN_STILL_ALIVE;
      mux = (rx_frame.data.u8[0] & (0x0FU));
      //HVP_debugMessageMultiplexer = (rx_frame.data.u8[0] & (0x0FU));  //0|4@1+ (1,0) [0|6] ""
      if (mux == 0) {
        HVP_gpioPassivePyroDepl = ((rx_frame.data.u8[0] >> 4) & (0x01U));       //: 4|1@1+ (1,0) [0|1] ""  Receiver
        HVP_gpioPyroIsoEn = ((rx_frame.data.u8[0] >> 5) & (0x01U));             //: 5|1@1+ (1,0) [0|1] ""  Receiver
        HVP_gpioCpFaultIn = ((rx_frame.data.u8[0] >> 6) & (0x01U));             //: 6|1@1+ (1,0) [0|1] ""  Receiver
        HVP_gpioPackContPowerEn = ((rx_frame.data.u8[0] >> 7) & (0x01U));       //: 7|1@1+ (1,0) [0|1] ""  Receiver
        HVP_gpioHvCablesOk = (rx_frame.data.u8[1] & (0x01U));                   //: 8|1@1+ (1,0) [0|1] ""  Receiver
        HVP_gpioHvpSelfEnable = ((rx_frame.data.u8[1] >> 1) & (0x01U));         //: 9|1@1+ (1,0) [0|1] ""  Receiver
        HVP_gpioLed = ((rx_frame.data.u8[1] >> 2) & (0x01U));                   //: 10|1@1+ (1,0) [0|1] ""  Receiver
        HVP_gpioCrashSignal = ((rx_frame.data.u8[1] >> 3) & (0x01U));           //: 11|1@1+ (1,0) [0|1] ""  Receiver
        HVP_gpioShuntDataReady = ((rx_frame.data.u8[1] >> 4) & (0x01U));        //: 12|1@1+ (1,0) [0|1] ""  Receiver
        HVP_gpioFcContPosAux = ((rx_frame.data.u8[1] >> 5) & (0x01U));          //: 13|1@1+ (1,0) [0|1] ""  Receiver
        HVP_gpioFcContNegAux = ((rx_frame.data.u8[1] >> 6) & (0x01U));          //: 14|1@1+ (1,0) [0|1] ""  Receiver
        HVP_gpioBmsEout = ((rx_frame.data.u8[1] >> 7) & (0x01U));               //: 15|1@1+ (1,0) [0|1] ""  Receiver
        HVP_gpioCpFaultOut = (rx_frame.data.u8[2] & (0x01U));                   //: 16|1@1+ (1,0) [0|1] ""  Receiver
        HVP_gpioPyroPor = ((rx_frame.data.u8[2] >> 1) & (0x01U));               //: 17|1@1+ (1,0) [0|1] ""  Receiver
        HVP_gpioShuntEn = ((rx_frame.data.u8[2] >> 2) & (0x01U));               //: 18|1@1+ (1,0) [0|1] ""  Receiver
        HVP_gpioHvpVerEn = ((rx_frame.data.u8[2] >> 3) & (0x01U));              //: 19|1@1+ (1,0) [0|1] ""  Receiver
        HVP_gpioPackCoontPosFlywheel = ((rx_frame.data.u8[2] >> 4) & (0x01U));  //: 20|1@1+ (1,0) [0|1] ""  Receiver
        HVP_gpioCpLatchEnable = ((rx_frame.data.u8[2] >> 5) & (0x01U));         //: 21|1@1+ (1,0) [0|1] ""  Receiver
        HVP_gpioPcsEnable = ((rx_frame.data.u8[2] >> 6) & (0x01U));             //: 22|1@1+ (1,0) [0|1] ""  Receiver
        HVP_gpioPcsDcdcPwmEnable = ((rx_frame.data.u8[2] >> 7) & (0x01U));      //: 23|1@1+ (1,0) [0|1] ""  Receiver
        HVP_gpioPcsChargePwmEnable = (rx_frame.data.u8[3] & (0x01U));           //: 24|1@1+ (1,0) [0|1] ""  Receiver
        HVP_gpioFcContPowerEnable = ((rx_frame.data.u8[3] >> 1) & (0x01U));     //: 25|1@1+ (1,0) [0|1] ""  Receiver
        HVP_gpioHvilEnable = ((rx_frame.data.u8[3] >> 2) & (0x01U));            //: 26|1@1+ (1,0) [0|1] ""  Receiver
        HVP_gpioSecDrdy = ((rx_frame.data.u8[3] >> 3) & (0x01U));               //: 27|1@1+ (1,0) [0|1] ""  Receiver
        HVP_hvp1v5Ref = ((rx_frame.data.u8[4] & (0xFFU)) << 4) |
                        ((rx_frame.data.u8[3] >> 4) & (0x0FU));  //: 28|12@1+ (0.1,0) [0|3] "V"  Receiver
        HVP_shuntCurrentDebug = ((rx_frame.data.u8[6] & (0xFFU)) << 8) |
                                (rx_frame.data.u8[5] & (0xFFU));     //: 40|16@1- (0.1,0) [-3276.8|3276.7] "A"  Receiver
        HVP_packCurrentMia = (rx_frame.data.u8[7] & (0x01U));        //: 56|1@1+ (1,0) [0|1] ""  Receiver
        HVP_auxCurrentMia = ((rx_frame.data.u8[7] >> 1) & (0x01U));  //: 57|1@1+ (1,0) [0|1] ""  Receiver
        HVP_currentSenseMia = ((rx_frame.data.u8[7] >> 2) & (0x03U));          //: 58|1@1+ (1,0) [0|1] ""  Receiver
        HVP_shuntRefVoltageMismatch = ((rx_frame.data.u8[7] >> 3) & (0x01U));  //: 59|1@1+ (1,0) [0|1] ""  Receiver
        HVP_shuntThermistorMia = ((rx_frame.data.u8[7] >> 4) & (0x01U));       //: 60|1@1+ (1,0) [0|1] ""  Receiver
        HVP_shuntHwMia = ((rx_frame.data.u8[7] >> 5) & (0x01U));               //: 61|1@1+ (1,0) [0|1] ""  Receiver
      }
      if (mux == 1) {
        HVP_dcLinkVoltage = ((rx_frame.data.u8[2] & (0xFFU)) << 8) |
                            (rx_frame.data.u8[1] & (0xFFU));  //: 8|16@1- (0.1,0) [-3276.8|3276.7] "V"  Receiver
        HVP_packVoltage = ((rx_frame.data.u8[4] & (0xFFU)) << 8) |
                          (rx_frame.data.u8[3] & (0xFFU));  //: 24|16@1- (0.1,0) [-3276.8|3276.7] "V"  Receiver
        HVP_fcLinkVoltage = ((rx_frame.data.u8[6] & (0xFFU)) << 8) |
                            (rx_frame.data.u8[5] & (0xFFU));  //: 40|16@1- (0.1,0) [-3276.8|3276.7] "V"  Receiver
      }
      if (mux == 2) {
        HVP_packContVoltage = ((rx_frame.data.u8[1] & (0xFFU)) << 4) |
                              ((rx_frame.data.u8[0] >> 4) & (0x0FU));  //: 4|12@1+ (0.1,0) [0|30] "V"  Receiver
        HVP_packNegativeV = ((rx_frame.data.u8[3] & (0xFFU)) << 8) |
                            (rx_frame.data.u8[2] & (0xFFU));  //: 16|16@1- (0.1,0) [-550|550] "V"  Receiver
        HVP_packPositiveV = ((rx_frame.data.u8[5] & (0xFFU)) << 8) |
                            (rx_frame.data.u8[4] & (0xFFU));  //: 32|16@1- (0.1,0) [-550|550] "V"  Receiver
        HVP_pyroAnalog = ((rx_frame.data.u8[7] & (0x0FU)) << 8) |
                         (rx_frame.data.u8[6] & (0xFFU));  //: 48|12@1+ (0.1,0) [0|3] "V"  Receiver
      }
      if (mux == 3) {
        HVP_dcLinkNegativeV = ((rx_frame.data.u8[2] & (0xFFU)) << 8) |
                              (rx_frame.data.u8[1] & (0xFFU));  //: 8|16@1- (0.1,0) [-550|550] "V"  Receiver
        HVP_dcLinkPositiveV = ((rx_frame.data.u8[4] & (0xFFU)) << 8) |
                              (rx_frame.data.u8[3] & (0xFFU));  //: 24|16@1- (0.1,0) [-550|550] "V"  Receiver
        HVP_fcLinkNegativeV = ((rx_frame.data.u8[6] & (0xFFU)) << 8) |
                              (rx_frame.data.u8[5] & (0xFFU));  //: 40|16@1- (0.1,0) [-550|550] "V"  Receiver
      }
      if (mux == 4) {
        HVP_fcContCoilCurrent = ((rx_frame.data.u8[1] & (0xFFU)) << 4) |
                                ((rx_frame.data.u8[0] >> 4) & (0x0FU));  //: 4|12@1+ (0.1,0) [0|7.5] "A"  Receiver
        HVP_fcContVoltage = ((rx_frame.data.u8[3] & (0x0FU)) << 8) |
                            (rx_frame.data.u8[2] & (0xFFU));  //: 16|12@1+ (0.1,0) [0|30] "V"  Receiver
        HVP_hvilInVoltage = ((rx_frame.data.u8[4] & (0xFFU)) << 4) |
                            ((rx_frame.data.u8[3] >> 4) & (0x0FU));  //: 28|12@1+ (0.1,0) [0|30] "V"  Receiver
        HVP_hvilOutVoltage = ((rx_frame.data.u8[6] & (0x0FU)) << 8) |
                             (rx_frame.data.u8[5] & (0xFFU));  //: 40|12@1+ (0.1,0) [0|30] "V"  Receiver
      }
      if (mux == 5) {
        HVP_fcLinkPositiveV = ((rx_frame.data.u8[2] & (0xFFU)) << 8) |
                              (rx_frame.data.u8[1] & (0xFFU));  //: 8|16@1- (0.1,0) [-550|550] "V"  Receiver
        HVP_packContCoilCurrent = ((rx_frame.data.u8[4] & (0x0FU)) << 8) |
                                  (rx_frame.data.u8[3] & (0xFFU));  //: 24|12@1+ (0.1,0) [0|7.5] "A"  Receiver
        HVP_battery12V = ((rx_frame.data.u8[5] & (0xFFU)) << 4) |
                         ((rx_frame.data.u8[4] >> 4) & (0x0FU));  //: 36|12@1+ (0.1,0) [0|30] "V"  Receiver
        HVP_shuntRefVoltageDbg = ((rx_frame.data.u8[7] & (0xFFU)) << 8) |
                                 (rx_frame.data.u8[6] & (0xFFU));  //: 48|16@1- (0.001,0) [-32.768|32.767] "V"  Receiver
      }
      if (mux == 6) {
        HVP_shuntAuxCurrentDbg = ((rx_frame.data.u8[2] & (0xFFU)) << 8) |
                                 (rx_frame.data.u8[1] & (0xFFU));  //: 8|16@1- (0.1,0) [-3276.8|3276.7] "A"  Receiver
        HVP_shuntBarTempDbg = ((rx_frame.data.u8[4] & (0xFFU)) << 8) |
                              (rx_frame.data.u8[3] & (0xFFU));  //: 24|16@1- (0.01,0) [-327.67|327.67] "C"  Receiver
        HVP_shuntAsicTempDbg = ((rx_frame.data.u8[6] & (0xFFU)) << 8) |
                               (rx_frame.data.u8[5] & (0xFFU));  //: 40|16@1- (0.01,0) [-327.67|327.67] "C"  Receiver
        HVP_shuntAuxCurrentStatus = (rx_frame.data.u8[7] & (0x03U));       //: 56|2@1+ (1,0) [0|3] ""  Receiver
        HVP_shuntBarTempStatus = ((rx_frame.data.u8[7] >> 2) & (0x03U));   //: 58|2@1+ (1,0) [0|3] ""  Receiver
        HVP_shuntAsicTempStatus = ((rx_frame.data.u8[7] >> 4) & (0x03U));  //: 60|2@1+ (1,0) [0|3] ""  Receiver
      }
      break;
    /* We ignore 0x3AA for now, as on later software/firmware this is a muxed frame so values aren't correct.
    // Reference (tesla-m3-pack-findings, firmware 2019.20.4.2): on that firmware HVP_alertMatrix1 is FLAT / non-muxed
    // with bitIndex = warnNum - 1 (w001 = byte0 bit0). The byte/bit layout below matches those findings exactly, and
    // that firmware additionally defines w052_bootErrataStatistics (byte6 bit3) and w053_hvVoltageSensorRationality
    // (byte6 bit4). Left disabled because Battery Emulator targets newer firmware where this frame is muxed.
    case 0x3aa:  //HVP_alertMatrix1
      battery_WatchdogReset = (rx_frame.data.u8[0] & 0x01);
      battery_PowerLossReset = ((rx_frame.data.u8[0] & 0x02) >> 1);
      battery_SwAssertion = ((rx_frame.data.u8[0] & 0x04) >> 2);
      battery_CrashEvent = ((rx_frame.data.u8[0] & 0x08) >> 3);
      battery_OverDchgCurrentFault = ((rx_frame.data.u8[0] & 0x10) >> 4);
      battery_OverChargeCurrentFault = ((rx_frame.data.u8[0] & 0x20) >> 5);
      battery_OverCurrentFault = ((rx_frame.data.u8[0] & 0x40) >> 6);
      battery_OverTemperatureFault = ((rx_frame.data.u8[0] & 0x80) >> 7);
      battery_OverVoltageFault = (rx_frame.data.u8[1] & 0x01);
      battery_UnderVoltageFault = ((rx_frame.data.u8[1] & 0x02) >> 1);
      battery_PrimaryBmbMiaFault = ((rx_frame.data.u8[1] & 0x04) >> 2);
      battery_SecondaryBmbMiaFault = ((rx_frame.data.u8[1] & 0x08) >> 3);
      battery_BmbMismatchFault = ((rx_frame.data.u8[1] & 0x10) >> 4);
      battery_BmsHviMiaFault = ((rx_frame.data.u8[1] & 0x20) >> 5);
      battery_CpMiaFault = ((rx_frame.data.u8[1] & 0x40) >> 6);
      battery_PcsMiaFault = ((rx_frame.data.u8[1] & 0x80) >> 7);
      battery_BmsFault = (rx_frame.data.u8[2] & 0x01);
      battery_PcsFault = ((rx_frame.data.u8[2] & 0x02) >> 1);
      battery_CpFault = ((rx_frame.data.u8[2] & 0x04) >> 2);
      battery_ShuntHwMiaFault = ((rx_frame.data.u8[2] & 0x08) >> 3);
      battery_PyroMiaFault = ((rx_frame.data.u8[2] & 0x10) >> 4);
      battery_hvsMiaFault = ((rx_frame.data.u8[2] & 0x20) >> 5);
      battery_hviMiaFault = ((rx_frame.data.u8[2] & 0x40) >> 6);
      battery_Supply12vFault = ((rx_frame.data.u8[2] & 0x80) >> 7);
      battery_VerSupplyFault = (rx_frame.data.u8[3] & 0x01);
      battery_HvilFault = ((rx_frame.data.u8[3] & 0x02) >> 1);
      battery_BmsHvsMiaFault = ((rx_frame.data.u8[3] & 0x04) >> 2);
      battery_PackVoltMismatchFault = ((rx_frame.data.u8[3] & 0x08) >> 3);
      battery_EnsMiaFault = ((rx_frame.data.u8[3] & 0x10) >> 4);
      battery_PackPosCtrArcFault = ((rx_frame.data.u8[3] & 0x20) >> 5);
      battery_packNegCtrArcFault = ((rx_frame.data.u8[3] & 0x40) >> 6);
      battery_ShuntHwAndBmsMiaFault = ((rx_frame.data.u8[3] & 0x80) >> 7);
      battery_fcContHwFault = (rx_frame.data.u8[4] & 0x01);
      battery_robinOverVoltageFault = ((rx_frame.data.u8[4] & 0x02) >> 1);
      battery_packContHwFault = ((rx_frame.data.u8[4] & 0x04) >> 2);
      battery_pyroFuseBlown = ((rx_frame.data.u8[4] & 0x08) >> 3);
      battery_pyroFuseFailedToBlow = ((rx_frame.data.u8[4] & 0x10) >> 4);
      battery_CpilFault = ((rx_frame.data.u8[4] & 0x20) >> 5);
      battery_PackContactorFellOpen = ((rx_frame.data.u8[4] & 0x40) >> 6);
      battery_FcContactorFellOpen = ((rx_frame.data.u8[4] & 0x80) >> 7);
      battery_packCtrCloseBlocked = (rx_frame.data.u8[5] & 0x01);
      battery_fcCtrCloseBlocked = ((rx_frame.data.u8[5] & 0x02) >> 1);
      battery_packContactorForceOpen = ((rx_frame.data.u8[5] & 0x04) >> 2);
      battery_fcContactorForceOpen = ((rx_frame.data.u8[5] & 0x08) >> 3);
      battery_dcLinkOverVoltage = ((rx_frame.data.u8[5] & 0x10) >> 4);
      battery_shuntOverTemperature = ((rx_frame.data.u8[5] & 0x20) >> 5);
      battery_passivePyroDeploy = ((rx_frame.data.u8[5] & 0x40) >> 6);
      battery_logUploadRequest = ((rx_frame.data.u8[5] & 0x80) >> 7);
      battery_packCtrCloseFailed = (rx_frame.data.u8[6] & 0x01);
      battery_fcCtrCloseFailed = ((rx_frame.data.u8[6] & 0x02) >> 1);
      battery_shuntThermistorMia = ((rx_frame.data.u8[6] & 0x04) >> 2);
      break;*/
    case 0x320:  //800 BMS_alertMatrix                                                //BMS_alertMatrix 800 BMS_alertMatrix: 8 VEH
      datalayer_battery->status.CAN_battery_still_alive = CAN_STILL_ALIVE;
      mux = (rx_frame.data.u8[0] & (0x0F));
      if (mux == 0) {                                      //mux0
        BMS_matrixIndex = (rx_frame.data.u8[0] & (0x0F));  // 0|4@1+ (1,0) [0|0] ""  X
        BMS_a001_Pack_Config_Mismatch =
            ((rx_frame.data.u8[0] >> 4) & (0x01));                      //4  (tesla-m3-pack-findings, fw 2019.20.4.2)
        BMS_a017_SW_Brick_OV = ((rx_frame.data.u8[2] >> 4) & (0x01));   //20|1@1+ (1,0) [0|0] ""  X
        BMS_a018_SW_Brick_UV = ((rx_frame.data.u8[2] >> 5) & (0x01));   //21|1@1+ (1,0) [0|0] ""  X
        BMS_a019_SW_Module_OT = ((rx_frame.data.u8[2] >> 6) & (0x01));  //22|1@1+ (1,0) [0|0] ""  X
        BMS_a021_SW_Dr_Limits_Regulation = (rx_frame.data.u8[3] & (0x01U));       //24|1@1+ (1,0) [0|0] ""  X
        BMS_a022_SW_Over_Current = ((rx_frame.data.u8[3] >> 1) & (0x01U));        //25|1@1+ (1,0) [0|0] ""  X
        BMS_a023_SW_Stack_OV = ((rx_frame.data.u8[3] >> 2) & (0x01U));            //26|1@1+ (1,0) [0|0] ""  X
        BMS_a024_SW_Islanded_Brick = ((rx_frame.data.u8[3] >> 3) & (0x01U));      //27|1@1+ (1,0) [0|0] ""  X
        BMS_a025_SW_PwrBalance_Anomaly = ((rx_frame.data.u8[3] >> 4) & (0x01U));  //28|1@1+ (1,0) [0|0] ""  X
        BMS_a026_SW_HFCurrent_Anomaly = ((rx_frame.data.u8[3] >> 5) & (0x01U));   //29|1@1+ (1,0) [0|0] ""  X
        BMS_a034_SW_Passive_Isolation = ((rx_frame.data.u8[4] >> 5) & (0x01U));   //37|1@1+ (1,0) [0|0] ""  X ?
        BMS_a035_SW_Isolation = ((rx_frame.data.u8[4] >> 6) & (0x01U));           //38|1@1+ (1,0) [0|0] ""  X
        BMS_a036_SW_HvpHvilFault =
            ((rx_frame.data.u8[4] >> 7) &
             (0x01U));  //39|1@1+ (1,0) [0|0] ""  X  (corrected byte4 bit6->bit7 per tesla-m3-pack-findings, fw 2019.20.4.2)
        BMS_a037_SW_Flood_Port_Open = (rx_frame.data.u8[5] & (0x01U));              //40|1@1+ (1,0) [0|0] ""  X
        BMS_a039_SW_DC_Link_Over_Voltage = ((rx_frame.data.u8[5] >> 2) & (0x01U));  //42|1@1+ (1,0) [0|0] ""  X
        BMS_a041_SW_Power_On_Reset = ((rx_frame.data.u8[5] >> 4) & (0x01U));        //44|1@1+ (1,0) [0|0] ""  X
        BMS_a042_SW_MPU_Error = ((rx_frame.data.u8[5] >> 5) & (0x01U));             //45|1@1+ (1,0) [0|0] ""  X
        BMS_a043_SW_Watch_Dog_Reset = ((rx_frame.data.u8[5] >> 6) & (0x01U));       //46|1@1+ (1,0) [0|0] ""  X
        BMS_a044_SW_Assertion = ((rx_frame.data.u8[5] >> 7) & (0x01U));             //47|1@1+ (1,0) [0|0] ""  X
        BMS_a045_SW_Exception = (rx_frame.data.u8[6] & (0x01U));                    //48|1@1+ (1,0) [0|0] ""  X
        BMS_a046_SW_Task_Stack_Usage = ((rx_frame.data.u8[6] >> 1) & (0x01U));      //49|1@1+ (1,0) [0|0] ""  X
        BMS_a047_SW_Task_Stack_Overflow = ((rx_frame.data.u8[6] >> 2) & (0x01U));   //50|1@1+ (1,0) [0|0] ""  X
        BMS_a048_SW_Log_Upload_Request = ((rx_frame.data.u8[6] >> 3) & (0x01U));    //51|1@1+ (1,0) [0|0] ""  X
        BMS_a050_SW_Brick_Voltage_MIA = ((rx_frame.data.u8[6] >> 5) & (0x01U));     //53|1@1+ (1,0) [0|0] ""  X
        BMS_a051_SW_HVC_Vref_Bad = ((rx_frame.data.u8[6] >> 6) & (0x01U));          //54|1@1+ (1,0) [0|0] ""  X
        BMS_a052_SW_PCS_MIA = ((rx_frame.data.u8[6] >> 7) & (0x01U));               //55|1@1+ (1,0) [0|0] ""  X
        BMS_a053_SW_ThermalModel_Sanity = (rx_frame.data.u8[7] & (0x01U));          //56|1@1+ (1,0) [0|0] ""  X
        BMS_a054_SW_Ver_Supply_Fault = ((rx_frame.data.u8[7] >> 1) & (0x01U));      //57|1@1+ (1,0) [0|0] ""  X
        BMS_a055_SW_HvChain_Model_Fault =
            ((rx_frame.data.u8[7] >> 2) & (0x01U));  //58  (tesla-m3-pack-findings, fw 2019.20.4.2)
        BMS_a059_SW_Pack_Voltage_Sensing = ((rx_frame.data.u8[7] >> 6) & (0x01U));  //62|1@1+ (1,0) [0|0] ""  X
        BMS_a060_SW_Leakage_Test_Failure = ((rx_frame.data.u8[7] >> 7) & (0x01U));  //63|1@1+ (1,0) [0|0] ""  X
      }
      if (mux == 1) {                                                              //mux1
        BMS_a061_robinBrickOverVoltage = ((rx_frame.data.u8[0] >> 4) & (0x01U));   //4|1@1+ (1,0) [0|0] ""  X
        BMS_a062_SW_BrickV_Imbalance = ((rx_frame.data.u8[0] >> 5) & (0x01U));     //5|1@1+ (1,0) [0|0] ""  X
        BMS_a063_SW_ChargePort_Fault = ((rx_frame.data.u8[0] >> 6) & (0x01U));     //6|1@1+ (1,0) [0|0] ""  X
        BMS_a064_SW_SOC_Imbalance = ((rx_frame.data.u8[0] >> 7) & (0x01U));        //7|1@1+ (1,0) [0|0] ""  X
        BMS_a069_SW_Low_Power = ((rx_frame.data.u8[1] >> 4) & (0x01U));            //12|1@1+ (1,0) [0|0] ""  X
        BMS_a071_SW_SM_TransCon_Not_Met = ((rx_frame.data.u8[1] >> 6) & (0x01U));  //14|1@1+ (1,0) [0|0] ""  X
        BMS_a075_SW_Chg_Disable_Failure = ((rx_frame.data.u8[2] >> 2) & (0x01U));  //18|1@1+ (1,0) [0|0] ""  X
        BMS_a076_SW_Dch_While_Charging = ((rx_frame.data.u8[2] >> 3) & (0x01U));   //19|1@1+ (1,0) [0|0] ""  X
        BMS_a077_SW_Charger_Regulation = ((rx_frame.data.u8[2] >> 4) & (0x01U));   //20|1@1+ (1,0) [0|0] ""  X
        BMS_a081_SW_Ctr_Close_Blocked = (rx_frame.data.u8[3] & (0x01U));           //24|1@1+ (1,0) [0|0] ""  X
        BMS_a082_SW_Ctr_Force_Open = ((rx_frame.data.u8[3] >> 1) & (0x01U));       //25|1@1+ (1,0) [0|0] ""  X
        BMS_a083_SW_Ctr_Close_Failure = ((rx_frame.data.u8[3] >> 2) & (0x01U));    //26|1@1+ (1,0) [0|0] ""  X
        BMS_a084_SW_Sleep_Wake_Aborted = ((rx_frame.data.u8[3] >> 3) & (0x01U));   //27|1@1+ (1,0) [0|0] ""  X
        BMS_a087_SW_Feim_Test_Blocked = ((rx_frame.data.u8[3] >> 6) & (0x01U));    //30|1@1+ (1,0) [0|0] ""  X
        BMS_a088_SW_VcFront_MIA_InDrive = ((rx_frame.data.u8[3] >> 7) & (0x01U));  //31|1@1+ (1,0) [0|0] ""  X
        BMS_a089_SW_VcFront_MIA = (rx_frame.data.u8[4] & (0x01U));                 //32|1@1+ (1,0) [0|0] ""  X
        BMS_a090_SW_Gateway_MIA = ((rx_frame.data.u8[4] >> 1) & (0x01U));          //33|1@1+ (1,0) [0|0] ""  X
        BMS_a091_SW_ChargePort_MIA = ((rx_frame.data.u8[4] >> 2) & (0x01U));       //34|1@1+ (1,0) [0|0] ""  X
        BMS_a092_SW_ChargePort_Mia_On_Hv =
            ((rx_frame.data.u8[4] >> 3) &
             (0x01U));  //35|1@1+ (1,0) [0|0] ""  X  (tesla-m3-pack-findings fw 2019.20.4.2 names this BMS_a092_SW_ChargePort_Mia_On_Hvs)
        BMS_a094_SW_Drive_Inverter_MIA = ((rx_frame.data.u8[4] >> 5) & (0x01U));  //37|1@1+ (1,0) [0|0] ""  X
        BMS_a099_SW_BMB_Communication = ((rx_frame.data.u8[5] >> 2) & (0x01U));   //42|1@1+ (1,0) [0|0] ""  X
        BMS_a105_SW_One_Module_Tsense = (rx_frame.data.u8[6] & (0x01U));          //48|1@1+ (1,0) [0|0] ""  X
        BMS_a106_SW_All_Module_Tsense = ((rx_frame.data.u8[6] >> 1) & (0x01U));   //49|1@1+ (1,0) [0|0] ""  X
        BMS_a107_SW_Stack_Voltage_MIA = ((rx_frame.data.u8[6] >> 2) & (0x01U));   //50|1@1+ (1,0) [0|0] ""  X
      }
      if (mux == 2) {                                                               //mux2
        BMS_a121_SW_NVRAM_Config_Error = ((rx_frame.data.u8[0] >> 4) & (0x01U));    // 4|1@1+ (1,0) [0|0] ""  X
        BMS_a122_SW_BMS_Therm_Irrational = ((rx_frame.data.u8[0] >> 5) & (0x01U));  //5|1@1+ (1,0) [0|0] ""  X
        BMS_a123_SW_Internal_Isolation = ((rx_frame.data.u8[0] >> 6) & (0x01U));    //6|1@1+ (1,0) [0|0] ""  X
        BMS_a126_SW_Thermistor_Failure =
            ((rx_frame.data.u8[1] >> 1) & (0x01U));                        //9  (tesla-m3-pack-findings, fw 2019.20.4.2)
        BMS_a127_SW_shunt_SNA = ((rx_frame.data.u8[1] >> 2) & (0x01U));    //10|1@1+ (1,0) [0|0] ""  X
        BMS_a128_SW_shunt_MIA = ((rx_frame.data.u8[1] >> 3) & (0x01U));    //11|1@1+ (1,0) [0|0] ""  X
        BMS_a129_SW_VSH_Failure = ((rx_frame.data.u8[1] >> 4) & (0x01U));  //12|1@1+ (1,0) [0|0] ""  X
        BMS_a130_IO_CAN_Error = ((rx_frame.data.u8[1] >> 5) & (0x01U));    //13|1@1+ (1,0) [0|0] ""  X
        BMS_a131_Bleed_FET_Failure = ((rx_frame.data.u8[1] >> 6) & (0x01U));      //14|1@1+ (1,0) [0|0] ""  X
        BMS_a132_HW_BMB_OTP_Uncorrctbl = ((rx_frame.data.u8[1] >> 7) & (0x01U));  //15|1@1+ (1,0) [0|0] ""  X
        BMS_a134_SW_Delayed_Ctr_Off = ((rx_frame.data.u8[2] >> 1) & (0x01U));     //17|1@1+ (1,0) [0|0] ""  X
        BMS_a135_HW_BMB_Diagnostics_Failure =
            ((rx_frame.data.u8[2] >> 2) & (0x01U));  //18  (tesla-m3-pack-findings, fw 2019.20.4.2)
        BMS_a136_SW_Module_OT_Warning = ((rx_frame.data.u8[2] >> 3) & (0x01U));     //19|1@1+ (1,0) [0|0] ""  X
        BMS_a137_SW_Brick_UV_Warning = ((rx_frame.data.u8[2] >> 4) & (0x01U));      //20|1@1+ (1,0) [0|0] ""  X
        BMS_a138_SW_Brick_OV_Warning = ((rx_frame.data.u8[2] >> 5) & (0x01U));      //21|1@1+ (1,0) [0|0] ""  X
        BMS_a139_SW_DC_Link_V_Irrational = ((rx_frame.data.u8[2] >> 6) & (0x01U));  //22|1@1+ (1,0) [0|0] ""  X
        BMS_a141_SW_BMB_Status_Warning = (rx_frame.data.u8[3] & (0x01U));           //24|1@1+ (1,0) [0|0] ""  X
        BMS_a143_SW_CAC_Change = ((rx_frame.data.u8[3] >> 2) & (0x01U));  //26  (tesla-m3-pack-findings, fw 2019.20.4.2)
        BMS_a144_Hvp_Config_Mismatch = ((rx_frame.data.u8[3] >> 3) & (0x01U));      //27|1@1+ (1,0) [0|0] ""  X
        BMS_a145_SW_SOC_Change = ((rx_frame.data.u8[3] >> 4) & (0x01U));            //28|1@1+ (1,0) [0|0] ""  X
        BMS_a146_SW_Brick_Overdischarged = ((rx_frame.data.u8[3] >> 5) & (0x01U));  //29|1@1+ (1,0) [0|0] ""  X
        BMS_a149_SW_Missing_Config_Block = (rx_frame.data.u8[4] & (0x01U));         //32|1@1+ (1,0) [0|0] ""  X
        BMS_a151_SW_external_isolation = ((rx_frame.data.u8[4] >> 2) & (0x01U));    //34|1@1+ (1,0) [0|0] ""  X
        BMS_a155_SW_Weak_short_impedence =
            ((rx_frame.data.u8[4] >> 6) & (0x01U));  //38  (tesla-m3-pack-findings, fw 2019.20.4.2)
        BMS_a156_SW_BMB_Vref_bad = ((rx_frame.data.u8[4] >> 7) & (0x01U));          //39|1@1+ (1,0) [0|0] ""  X
        BMS_a157_SW_HVP_HVS_Comms = (rx_frame.data.u8[5] & (0x01U));                //40|1@1+ (1,0) [0|0] ""  X
        BMS_a158_SW_HVP_HVI_Comms = ((rx_frame.data.u8[5] >> 1) & (0x01U));         //41|1@1+ (1,0) [0|0] ""  X
        BMS_a159_SW_HVP_ECU_Error = ((rx_frame.data.u8[5] >> 2) & (0x01U));         //42|1@1+ (1,0) [0|0] ""  X
        BMS_a161_SW_DI_Open_Request = ((rx_frame.data.u8[5] >> 4) & (0x01U));       //44|1@1+ (1,0) [0|0] ""  X
        BMS_a162_SW_No_Power_For_Support = ((rx_frame.data.u8[5] >> 5) & (0x01U));  //45|1@1+ (1,0) [0|0] ""  X
        BMS_a163_SW_Contactor_Mismatch = ((rx_frame.data.u8[5] >> 6) & (0x01U));    //46|1@1+ (1,0) [0|0] ""  X
        BMS_a164_SW_Uncontrolled_Regen = ((rx_frame.data.u8[5] >> 7) & (0x01U));    //47|1@1+ (1,0) [0|0] ""  X
        BMS_a165_SW_Pack_Partial_Weld = (rx_frame.data.u8[6] & (0x01U));            //48|1@1+ (1,0) [0|0] ""  X
        BMS_a166_SW_Pack_Full_Weld = ((rx_frame.data.u8[6] >> 1) & (0x01U));        //49|1@1+ (1,0) [0|0] ""  X
        BMS_a167_SW_FC_Partial_Weld = ((rx_frame.data.u8[6] >> 2) & (0x01U));       //50|1@1+ (1,0) [0|0] ""  X
        BMS_a168_SW_FC_Full_Weld = ((rx_frame.data.u8[6] >> 3) & (0x01U));          //51|1@1+ (1,0) [0|0] ""  X
        BMS_a169_SW_FC_Pack_Weld = ((rx_frame.data.u8[6] >> 4) & (0x01U));          //52|1@1+ (1,0) [0|0] ""  X
        BMS_a170_SW_Limp_Mode = ((rx_frame.data.u8[6] >> 5) & (0x01U));             //53|1@1+ (1,0) [0|0] ""  X
        BMS_a171_SW_Stack_Voltage_Sense = ((rx_frame.data.u8[6] >> 6) & (0x01U));   //54|1@1+ (1,0) [0|0] ""  X
        BMS_a173_SW_Charge_Component_Fault =
            (rx_frame.data.u8[7] & (0x01U));  //56  (tesla-m3-pack-findings, fw 2019.20.4.2)
        BMS_a174_SW_Charge_Failure = ((rx_frame.data.u8[7] >> 1) & (0x01U));    //57|1@1+ (1,0) [0|0] ""  X
        BMS_a176_SW_GracefulPowerOff = ((rx_frame.data.u8[7] >> 3) & (0x01U));  //59|1@1+ (1,0) [0|0] ""  X
        BMS_a178_SW_Uncontrolled_Regen_PwrB =
            ((rx_frame.data.u8[7] >> 5) & (0x01U));  //61  (tesla-m3-pack-findings, fw 2019.20.4.2)
        BMS_a179_SW_Hvp_12V_Fault = ((rx_frame.data.u8[7] >> 6) & (0x01U));      //62|1@1+ (1,0) [0|0] ""  X
        BMS_a180_SW_ECU_reset_blocked = ((rx_frame.data.u8[7] >> 7) & (0x01U));  //63|1@1+ (1,0) [0|0] ""  X
      }
      break;
    case 0x3a4: {  //932 PCS_alertMatrix — Tesla Model 3/Y
      // Alert matrix mapped from tesla-m3-pack-findings (firmware 2019.20.4.2); [mux,byte,bit] per extracted matrix.
      // Muxed frame: mux = data[0] & 0x0F. Read active = (data[byte] >> bit) & 1.
      datalayer_battery->status.CAN_battery_still_alive = CAN_STILL_ALIVE;
      mux = (rx_frame.data.u8[0] & (0x0F));
      if (mux == 0) {  //mux0
        PCS_a001_chgHwInputOc = ((rx_frame.data.u8[0] >> 4) & (0x01U));
        PCS_a002_chgHwOutputOc = ((rx_frame.data.u8[0] >> 5) & (0x01U));
        PCS_a003_chgHwInputOv = ((rx_frame.data.u8[0] >> 6) & (0x01U));
        PCS_a004_chgHwIntBusOv = ((rx_frame.data.u8[0] >> 7) & (0x01U));
        PCS_a005_chgOutputOv = (rx_frame.data.u8[1] & (0x01U));
        PCS_a006_chgPrechargeFailedScr = ((rx_frame.data.u8[1] >> 1) & (0x01U));
        PCS_a007_chgPhaseTempHot = ((rx_frame.data.u8[1] >> 2) & (0x01U));
        PCS_a008_chgPhaseOverTemp = ((rx_frame.data.u8[1] >> 3) & (0x01U));
        PCS_a009_chgPfcCurrentRegulation = ((rx_frame.data.u8[1] >> 4) & (0x01U));
        PCS_a010_chgIntBusVRegulation = ((rx_frame.data.u8[1] >> 5) & (0x01U));
        PCS_a011_chgLlcCurrentRegulation = ((rx_frame.data.u8[1] >> 6) & (0x01U));
        PCS_a012_chgPfcIBandTracerFault = ((rx_frame.data.u8[1] >> 7) & (0x01U));
        PCS_a013_chgPrechargeFailedBoost = (rx_frame.data.u8[2] & (0x01U));
        PCS_a014_chgTempRationality = ((rx_frame.data.u8[2] >> 1) & (0x01U));
        PCS_a015_chg12vUv = ((rx_frame.data.u8[2] >> 2) & (0x01U));
        PCS_a016_chgAllPhasesFaulted = ((rx_frame.data.u8[2] >> 3) & (0x01U));
        PCS_a017_chgWallPowerRemoval = ((rx_frame.data.u8[2] >> 4) & (0x01U));
        PCS_a018_chgUnknownGridConfig = ((rx_frame.data.u8[2] >> 5) & (0x01U));
        PCS_a019_acChargePowerLimited = ((rx_frame.data.u8[2] >> 6) & (0x01U));
        PCS_a020_chgEnableLineMismatch = ((rx_frame.data.u8[2] >> 7) & (0x01U));
        PCS_a021_hvpMia = (rx_frame.data.u8[3] & (0x01U));
        PCS_a022_bmsMia = ((rx_frame.data.u8[3] >> 1) & (0x01U));
        PCS_a023_cpMia = ((rx_frame.data.u8[3] >> 2) & (0x01U));
        PCS_a024_vcfrontMia = ((rx_frame.data.u8[3] >> 3) & (0x01U));
        PCS_a025_cpu2Malfunction = ((rx_frame.data.u8[3] >> 4) & (0x01U));
        PCS_a026_watchdogAlarmed = ((rx_frame.data.u8[3] >> 5) & (0x01U));
        PCS_a027_chgInsufficientCooling = ((rx_frame.data.u8[3] >> 6) & (0x01U));
        PCS_a028_chgOutputUv = ((rx_frame.data.u8[3] >> 7) & (0x01U));
        PCS_a029_chgPowerRationality = (rx_frame.data.u8[4] & (0x01U));
        PCS_a030_canRationality = ((rx_frame.data.u8[4] >> 1) & (0x01U));
        PCS_a031_uiMia = ((rx_frame.data.u8[4] >> 2) & (0x01U));
        PCS_a032_gtwMia = ((rx_frame.data.u8[4] >> 3) & (0x01U));
        PCS_a033_hvBusUv = ((rx_frame.data.u8[4] >> 4) & (0x01U));
        PCS_a034_hvBusOv = ((rx_frame.data.u8[4] >> 5) & (0x01U));
        PCS_a035_lvBusUv = ((rx_frame.data.u8[4] >> 6) & (0x01U));
        PCS_a036_lvBusOv = ((rx_frame.data.u8[4] >> 7) & (0x01U));
        PCS_a037_resonantTankOc = (rx_frame.data.u8[5] & (0x01U));
        PCS_a038_claFaulted = ((rx_frame.data.u8[5] >> 1) & (0x01U));
        PCS_a039_sdModuleClkFault = ((rx_frame.data.u8[5] >> 2) & (0x01U));
        PCS_a040_dcdcMaxPowerReached = ((rx_frame.data.u8[5] >> 3) & (0x01U));
        PCS_a041_dcdcOverTemp = ((rx_frame.data.u8[5] >> 4) & (0x01U));
        PCS_a042_dcdcEnableLineMismatch = ((rx_frame.data.u8[5] >> 5) & (0x01U));
        PCS_a043_hvBusPrechargeFailure = ((rx_frame.data.u8[5] >> 6) & (0x01U));
        PCS_a044_12vSupportRegulation = ((rx_frame.data.u8[5] >> 7) & (0x01U));
        PCS_a045_hvBusLowImpedance = (rx_frame.data.u8[6] & (0x01U));
        PCS_a046_hvBusHighImpedence = ((rx_frame.data.u8[6] >> 1) & (0x01U));
        PCS_a047_lvBusLowImpedance = ((rx_frame.data.u8[6] >> 2) & (0x01U));
        PCS_a048_lvBusHighImpedance = ((rx_frame.data.u8[6] >> 3) & (0x01U));
        PCS_a049_dcdcTempRationality = ((rx_frame.data.u8[6] >> 4) & (0x01U));
        PCS_a050_dcdc12VsupportFaulted = ((rx_frame.data.u8[6] >> 5) & (0x01U));
        PCS_a051_chgIntBusUv = ((rx_frame.data.u8[6] >> 6) & (0x01U));
        PCS_a052_acVoltageNotPresent = ((rx_frame.data.u8[6] >> 7) & (0x01U));
        PCS_a053_chgInputVDropHigh = (rx_frame.data.u8[7] & (0x01U));
        PCS_a054_chgInputVDropTooHigh = ((rx_frame.data.u8[7] >> 1) & (0x01U));
        PCS_a055_chgLineImedanceHigh = ((rx_frame.data.u8[7] >> 2) & (0x01U));
        PCS_a056_chgLineImedanceTooHigh = ((rx_frame.data.u8[7] >> 3) & (0x01U));
        PCS_a057_chgInputOverFreq = ((rx_frame.data.u8[7] >> 4) & (0x01U));
        PCS_a058_chgInputUnderFreq = ((rx_frame.data.u8[7] >> 5) & (0x01U));
        PCS_a059_chgInputOvRms = ((rx_frame.data.u8[7] >> 6) & (0x01U));
        PCS_a060_chgInputOvPeak = ((rx_frame.data.u8[7] >> 7) & (0x01U));
      }
      if (mux == 1) {  //mux1
        PCS_a061_chgVLineRationality = ((rx_frame.data.u8[0] >> 4) & (0x01U));
        PCS_a062_chgILineRationality = ((rx_frame.data.u8[0] >> 5) & (0x01U));
        PCS_a063_chgVOutRationality = ((rx_frame.data.u8[0] >> 6) & (0x01U));
        PCS_a064_chgIOutRationality = ((rx_frame.data.u8[0] >> 7) & (0x01U));
        PCS_a065_chgPllNotLocked = (rx_frame.data.u8[1] & (0x01U));
        PCS_a066_dcdcHvRationality = ((rx_frame.data.u8[1] >> 1) & (0x01U));
        PCS_a067_dcdcLvRationality = ((rx_frame.data.u8[1] >> 2) & (0x01U));
        PCS_a068_dcdcTankvRationality = ((rx_frame.data.u8[1] >> 3) & (0x01U));
        PCS_a069_chgPfcLineDidt = ((rx_frame.data.u8[1] >> 4) & (0x01U));
        PCS_a070_chgPfcLineDvdt = ((rx_frame.data.u8[1] >> 5) & (0x01U));
        PCS_a071_chgPfcILoopRationality = ((rx_frame.data.u8[1] >> 6) & (0x01U));
        PCS_a072_cpu2ClaStopped = ((rx_frame.data.u8[1] >> 7) & (0x01U));
        PCS_a073_unexpectedAcInputVoltage = (rx_frame.data.u8[2] & (0x01U));
        PCS_a074_hvBusDischargeFailure = ((rx_frame.data.u8[2] >> 1) & (0x01U));
        PCS_a075_hvBusDischargeTimeout = ((rx_frame.data.u8[2] >> 2) & (0x01U));
        PCS_a076_dcdcEnDeassertedErr = ((rx_frame.data.u8[2] >> 3) & (0x01U));
        PCS_a077_microGridEnergyLow = ((rx_frame.data.u8[2] >> 4) & (0x01U));
        PCS_a078_chgStopDcdcTooHot = ((rx_frame.data.u8[2] >> 5) & (0x01U));
        PCS_a079_eepromOperationError = ((rx_frame.data.u8[2] >> 6) & (0x01U));
        PCS_a080_damagedPhaseDetected = ((rx_frame.data.u8[2] >> 7) & (0x01U));
        PCS_a081_dcdcPchgTimeout = (rx_frame.data.u8[3] & (0x01U));
        PCS_a082_dcdcPchgUnsafeDiVoltage = ((rx_frame.data.u8[3] >> 1) & (0x01U));
        PCS_a083_triggerOdin = ((rx_frame.data.u8[3] >> 2) & (0x01U));
        PCS_a084_dcdcPchgStartVoltages = ((rx_frame.data.u8[3] >> 3) & (0x01U));
        PCS_a085_dcdcFetsNotSwitching = ((rx_frame.data.u8[3] >> 4) & (0x01U));
        PCS_a086_dcdcInsufficientCooling = ((rx_frame.data.u8[3] >> 5) & (0x01U));
        PCS_a087_nvramRecordStatusError = ((rx_frame.data.u8[3] >> 6) & (0x01U));
        PCS_a088_pchgParameters = ((rx_frame.data.u8[3] >> 7) & (0x01U));
        PCS_a089_hvBusDischargeIrrational = (rx_frame.data.u8[4] & (0x01U));
        PCS_a090_expectedAcVoltageSourceMissing = ((rx_frame.data.u8[4] >> 1) & (0x01U));
        PCS_a091_chgIntBusRationality = ((rx_frame.data.u8[4] >> 2) & (0x01U));
        PCS_a092_chgPowerLimitedByBusRipple = ((rx_frame.data.u8[4] >> 3) & (0x01U));
        PCS_a093_powerRailRationality = ((rx_frame.data.u8[4] >> 4) & (0x01U));
        PCS_a094_pcsDcdcNeedService = ((rx_frame.data.u8[4] >> 5) & (0x01U));
      }
      break;
    }
    case 0x31e: {  //798 CP_alertMatrix — Tesla Model 3/Y
      // Alert matrix mapped from tesla-m3-pack-findings (firmware 2019.20.4.2); [mux,byte,bit] per extracted matrix.
      // Muxed frame: mux = data[0] & 0x0F. Read active = (data[byte] >> bit) & 1.
      datalayer_battery->status.CAN_battery_still_alive = CAN_STILL_ALIVE;
      mux = (rx_frame.data.u8[0] & (0x0F));
      if (mux == 0) {  //mux0
        CP_a001_canRx = ((rx_frame.data.u8[0] >> 4) & (0x01U));
        CP_a002_canTx = ((rx_frame.data.u8[0] >> 5) & (0x01U));
        CP_a003_canError = ((rx_frame.data.u8[0] >> 6) & (0x01U));
        CP_a004_proximityRationality = ((rx_frame.data.u8[0] >> 7) & (0x01U));
        CP_a005_gbdcLiveDisconnect = (rx_frame.data.u8[1] & (0x01U));
        CP_a006_lostCommsBMS = ((rx_frame.data.u8[1] >> 1) & (0x01U));
        CP_a007_watchdog = ((rx_frame.data.u8[1] >> 2) & (0x01U));
        CP_a008_memoryError = ((rx_frame.data.u8[1] >> 3) & (0x01U));
        CP_a009_coverOpen = ((rx_frame.data.u8[1] >> 4) & (0x01U));
        CP_a010_pilotRationality = ((rx_frame.data.u8[1] >> 5) & (0x01U));
        CP_a011_eeprom = ((rx_frame.data.u8[1] >> 6) & (0x01U));
        CP_a012_ledDriver = ((rx_frame.data.u8[1] >> 7) & (0x01U));
        CP_a013_lostCommsGTW = (rx_frame.data.u8[2] & (0x01U));
        CP_a014_lostCommsCHG = ((rx_frame.data.u8[2] >> 1) & (0x01U));
        CP_a015_apsVov = ((rx_frame.data.u8[2] >> 2) & (0x01U));
        CP_a016_apsVuv = ((rx_frame.data.u8[2] >> 3) & (0x01U));
        CP_a017_fiveVov = ((rx_frame.data.u8[2] >> 4) & (0x01U));
        CP_a018_fiveVuv = ((rx_frame.data.u8[2] >> 5) & (0x01U));
        CP_a019_threeVov = ((rx_frame.data.u8[2] >> 6) & (0x01U));
        CP_a020_threeVuv = ((rx_frame.data.u8[2] >> 7) & (0x01U));
        CP_a021_zeroVov = (rx_frame.data.u8[3] & (0x01U));
        CP_a022_zeroVuv = ((rx_frame.data.u8[3] >> 1) & (0x01U));
        CP_a023_gbdcSessionFailed = ((rx_frame.data.u8[3] >> 2) & (0x01U));
        CP_a024_ledsUC = ((rx_frame.data.u8[3] >> 3) & (0x01U));
        CP_a025_ledsOC = ((rx_frame.data.u8[3] >> 4) & (0x01U));
        CP_a026_networkManagement = ((rx_frame.data.u8[3] >> 5) & (0x01U));
        CP_a027_doorSensorOutOfSpec = ((rx_frame.data.u8[3] >> 6) & (0x01U));
        CP_a028_insertEnableMismatch = ((rx_frame.data.u8[3] >> 7) & (0x01U));
        CP_a029_doorClosedProxPilot = (rx_frame.data.u8[4] & (0x01U));
        CP_a030_busOff = ((rx_frame.data.u8[4] >> 1) & (0x01U));
        CP_a031_doorClosedCommandedOpen = ((rx_frame.data.u8[4] >> 2) & (0x01U));
        CP_a032_doorOpenExpectedClosed = ((rx_frame.data.u8[4] >> 3) & (0x01U));
        CP_a033_spiOpen = ((rx_frame.data.u8[4] >> 4) & (0x01U));
        CP_a034_calibrationIncomplete = ((rx_frame.data.u8[4] >> 5) & (0x01U));
        CP_a035_latchMovement_1 = ((rx_frame.data.u8[4] >> 6) & (0x01U));
        CP_a036_latchNotDisengaged = ((rx_frame.data.u8[4] >> 7) & (0x01U));
        CP_a037_latchNotEngaged = (rx_frame.data.u8[5] & (0x01U));
        CP_a038_latchNotBlocking = ((rx_frame.data.u8[5] >> 1) & (0x01U));
        CP_a039_latchMovement_2 = ((rx_frame.data.u8[5] >> 2) & (0x01U));
        CP_a040_doNotUse = ((rx_frame.data.u8[5] >> 3) & (0x01U));
        CP_a041_doorSensorUnplugged = ((rx_frame.data.u8[5] >> 4) & (0x01U));
        CP_a042_doorAssemblyBroken = ((rx_frame.data.u8[5] >> 5) & (0x01U));
        CP_a043_doorPotIrrational = ((rx_frame.data.u8[5] >> 6) & (0x01U));
        CP_a044_lostCommsHVP = ((rx_frame.data.u8[5] >> 7) & (0x01U));
        CP_a045_lostCommsVCSEC = (rx_frame.data.u8[6] & (0x01U));
        CP_a046_lostCommsEVSE = ((rx_frame.data.u8[6] >> 1) & (0x01U));
        CP_a047_lostCommsVCFRONT = ((rx_frame.data.u8[6] >> 2) & (0x01U));
        CP_a048_lostCommsUI = ((rx_frame.data.u8[6] >> 3) & (0x01U));
        CP_a049_multipleCablesDetected = ((rx_frame.data.u8[6] >> 4) & (0x01U));
        CP_a050_latchNotConnected = ((rx_frame.data.u8[6] >> 5) & (0x01U));
        CP_a051_doorInductiveSensorMIA = ((rx_frame.data.u8[6] >> 6) & (0x01U));
        CP_a052_evseNotSupported = ((rx_frame.data.u8[6] >> 7) & (0x01U));
        CP_a053_proxLatchedNoPilot = (rx_frame.data.u8[7] & (0x01U));
        CP_a054_cableNotSecured = ((rx_frame.data.u8[7] >> 1) & (0x01U));
        CP_a055_chargeStoppedNoPilot = ((rx_frame.data.u8[7] >> 2) & (0x01U));
        CP_a056_proxDisconnected = ((rx_frame.data.u8[7] >> 3) & (0x01U));
        CP_a057_evseFaulted = ((rx_frame.data.u8[7] >> 4) & (0x01U));
        CP_a058_acChargingBlocked = ((rx_frame.data.u8[7] >> 5) & (0x01U));
        CP_a059_swcanError = ((rx_frame.data.u8[7] >> 6) & (0x01U));
        CP_a060_lostCommsPCS = ((rx_frame.data.u8[7] >> 7) & (0x01U));
      }
      if (mux == 1) {  //mux1
        CP_a061_uhfReceiverMIA = ((rx_frame.data.u8[0] >> 4) & (0x01U));
        CP_a062_scOutOfService = ((rx_frame.data.u8[0] >> 5) & (0x01U));
        CP_a063_scUpdateInProgress = ((rx_frame.data.u8[0] >> 6) & (0x01U));
        CP_a064_superchargingBlocked = ((rx_frame.data.u8[0] >> 7) & (0x01U));
        CP_a065_selfTestFailed = (rx_frame.data.u8[1] & (0x01U));
        CP_a066_proxLatchedIdlePilot = ((rx_frame.data.u8[1] >> 1) & (0x01U));
        CP_a067_gbdcConnFault = ((rx_frame.data.u8[1] >> 2) & (0x01U));
        CP_a068_doorSensorMismatch = ((rx_frame.data.u8[1] >> 3) & (0x01U));
        CP_a069_doorInductiveSensorError = ((rx_frame.data.u8[1] >> 4) & (0x01U));
        CP_a070_doorInductiveSensorReset = ((rx_frame.data.u8[1] >> 5) & (0x01U));
        CP_a071_exiDecodeFailure = ((rx_frame.data.u8[1] >> 6) & (0x01U));
        CP_a072_v2gEvccTimeout = ((rx_frame.data.u8[1] >> 7) & (0x01U));
        CP_a073_iecComboShutdown = (rx_frame.data.u8[2] & (0x01U));
        CP_a074_failedToEstablishV2gComm = ((rx_frame.data.u8[2] >> 1) & (0x01U));
        CP_a075_v2gCommsFailure = ((rx_frame.data.u8[2] >> 2) & (0x01U));
        CP_a076_LDC1612errorWatchdog = ((rx_frame.data.u8[2] >> 3) & (0x01U));
        CP_a077_invalidMacAddress = ((rx_frame.data.u8[2] >> 4) & (0x01U));
        CP_a078_latchNotDisengagedCold = ((rx_frame.data.u8[2] >> 5) & (0x01U));
        CP_a079_cableNotSecuredCold = ((rx_frame.data.u8[2] >> 6) & (0x01U));
        CP_a080_taskStackOverflow = ((rx_frame.data.u8[2] >> 7) & (0x01U));
        CP_a081_swException = (rx_frame.data.u8[3] & (0x01U));
        CP_a082_powerOnReset = ((rx_frame.data.u8[3] >> 1) & (0x01U));
        CP_a083_watchdogTraceData = ((rx_frame.data.u8[3] >> 2) & (0x01U));
        CP_a084_proximityPeDisconnected = ((rx_frame.data.u8[3] >> 3) & (0x01U));
        CP_a085_dcPinTempFaulted = ((rx_frame.data.u8[3] >> 4) & (0x01U));
        CP_a086_dcPinTempIrrational = ((rx_frame.data.u8[3] >> 5) & (0x01U));
        CP_a087_dcTempModelFault = ((rx_frame.data.u8[3] >> 6) & (0x01U));
        CP_a088_dcTempModelDeviation = ((rx_frame.data.u8[3] >> 7) & (0x01U));
        CP_a089_plcConfigMismatch = (rx_frame.data.u8[4] & (0x01U));
        CP_a090_ccsEvseLowIso = ((rx_frame.data.u8[4] >> 1) & (0x01U));
        CP_a091_wrongSuperchargerHandle = ((rx_frame.data.u8[4] >> 2) & (0x01U));
        CP_a092_modemAppLoadFailed = ((rx_frame.data.u8[4] >> 3) & (0x01U));
        CP_a093_modemLoadedWithReset = ((rx_frame.data.u8[4] >> 4) & (0x01U));
        CP_a094_inductiveResetSuccessful = ((rx_frame.data.u8[4] >> 5) & (0x01U));
        CP_a095_thermalDcLimitActive = ((rx_frame.data.u8[4] >> 6) & (0x01U));
        CP_a096_pilotWake = ((rx_frame.data.u8[4] >> 7) & (0x01U));
      }
      break;
    }
    case 0x72A:  //BMS_serialNumber
      datalayer_battery->status.CAN_battery_still_alive = CAN_STILL_ALIVE;
      //Pack serial number in ASCII: 00 54 47 33 32 31 32 30 (mux 0) .TG32120 + 01 32 30 30 33 41 48 58 (mux 1) .2003AHX = TG321202003AHX
      if (rx_frame.data.u8[0] == 0x00 && !parsed_battery_serialNumber) {  // Serial number 1-7
        battery_serialNumber[0] = rx_frame.data.u8[1];
        battery_serialNumber[1] = rx_frame.data.u8[2];
        battery_serialNumber[2] = rx_frame.data.u8[3];
        battery_serialNumber[3] = rx_frame.data.u8[4];
        battery_serialNumber[4] = rx_frame.data.u8[5];
        battery_serialNumber[5] = rx_frame.data.u8[6];
        battery_serialNumber[6] = rx_frame.data.u8[7];
      }
      if (rx_frame.data.u8[0] == 0x01 && !parsed_battery_serialNumber) {  // Serial number 8-14
        battery_serialNumber[7] = rx_frame.data.u8[1];
        battery_serialNumber[8] = rx_frame.data.u8[2];
        battery_serialNumber[9] = rx_frame.data.u8[3];
        battery_serialNumber[10] = rx_frame.data.u8[4];
        battery_serialNumber[11] = rx_frame.data.u8[5];
        battery_serialNumber[12] = rx_frame.data.u8[6];
        battery_serialNumber[13] = rx_frame.data.u8[7];
      }
      if (battery_serialNumber[6] != 0 && battery_serialNumber[12] != 0 &&
          !parsed_battery_serialNumber) {  // Serial number complete
        derive_manufacture_date_from_serial();
        parsed_battery_serialNumber = true;
      }
      break;
    case 0x300:  //BMS_info
      datalayer_battery->status.CAN_battery_still_alive = CAN_STILL_ALIVE;
      //Display internal BMS info and other build/version data
      if (rx_frame.data.u8[0] == 0x0A) {  // Mux 10: BUILD_HWID_COMPONENTID
        if (BMS_info_buildConfigId == 0) {
          BMS_info_buildConfigId = static_cast<uint16_t>(extract_signal_value(rx_frame.data.u8, 16, 16));
        }
        if (BMS_info_hardwareId == 0) {
          BMS_info_hardwareId = static_cast<uint16_t>(extract_signal_value(rx_frame.data.u8, 32, 16));
        }
        if (BMS_info_componentId == 0) {
          BMS_info_componentId = static_cast<uint16_t>(extract_signal_value(rx_frame.data.u8, 48, 16));
        }
      }
      /*
      if (rx_frame.data.u8[0] == 0x0B) { // Mux 11: PCBAID_ASSYID_USAGEID
        if (BMS_info_pcbaId == 0) {BMS_info_pcbaId = static_cast<uint8_t>(extract_signal_value(rx_frame.data.u8, 16, 8));}
        if (BMS_info_assemblyId == 0) {BMS_info_assemblyId = static_cast<uint8_t>(extract_signal_value(rx_frame.data.u8, 24, 8));}
        if (BMS_info_usageId == 0) {BMS_info_usageId = static_cast<uint16_t>(extract_signal_value(rx_frame.data.u8, 32, 16));}
        if (BMS_info_subUsageId == 0) {BMS_info_subUsageId = static_cast<uint16_t>(extract_signal_value(rx_frame.data.u8, 48, 16));}
      }
      if (rx_frame.data.u8[0] == 0x0D) { // Mux 13: APP_CRC
        if (BMS_info_platformType == 0) {BMS_info_platformType = static_cast<uint8_t>(extract_signal_value(rx_frame.data.u8, 8, 8));}
        if (BMS_info_appCrc == 0) {BMS_info_appCrc = static_cast<uint32_t>(extract_signal_value(rx_frame.data.u8, 32, 32));}
      }
      if (rx_frame.data.u8[0] == 0x12) { // Mux 18: BOOTLOADER_GITHASH
        if (BMS_info_bootGitHash == 0) {BMS_info_bootGitHash = static_cast<uint64_t>(extract_signal_value(rx_frame.data.u8, 10, 54));}
      }
      if (rx_frame.data.u8[0] == 0x14) { // Mux 20: UDS_PROTOCOL_BOOTCRC
        if (BMS_info_bootUdsProtoVersion == 0) {BMS_info_bootUdsProtoVersion = static_cast<uint8_t>(extract_signal_value(rx_frame.data.u8, 8, 8));}
        if (BMS_info_bootCrc == 0) {BMS_info_bootCrc = static_cast<uint32_t>(extract_signal_value(rx_frame.data.u8, 32, 32));}
      }
      */
      break;
    case 0x3C4:  //PCS_info
      datalayer_battery->status.CAN_battery_still_alive = CAN_STILL_ALIVE;
      //Display internal PCS info and other build/version data
      if (rx_frame.data.u8[0] == 0x0A) {  // Mux 10: BUILD_HWID_COMPONENTID
        if (PCS_info_buildConfigId == 0) {
          PCS_info_buildConfigId = static_cast<uint16_t>(extract_signal_value(rx_frame.data.u8, 16, 16));
        }
        if (PCS_info_hardwareId == 0) {
          PCS_info_hardwareId = static_cast<uint16_t>(extract_signal_value(rx_frame.data.u8, 32, 16));
        }
        if (PCS_info_componentId == 0) {
          PCS_info_componentId = static_cast<uint16_t>(extract_signal_value(rx_frame.data.u8, 48, 16));
        }
      }
      /*
      if (rx_frame.data.u8[0] == 0x0B) { // Mux 11: PCBAID_ASSYID_USAGEID
        if (PCS_info_pcbaId == 0) {PCS_info_pcbaId = static_cast<uint8_t>(extract_signal_value(rx_frame.data.u8, 16, 8));}
        if (PCS_info_assemblyId == 0) {PCS_info_assemblyId = static_cast<uint8_t>(extract_signal_value(rx_frame.data.u8, 24, 8));}
        if (PCS_info_usageId == 0) {PCS_info_usageId = static_cast<uint16_t>(extract_signal_value(rx_frame.data.u8, 32, 16));}
        if (PCS_info_subUsageId == 0) {PCS_info_subUsageId = static_cast<uint16_t>(extract_signal_value(rx_frame.data.u8, 48, 16));}
      }
      if (rx_frame.data.u8[0] == 0x0D) { // Mux 13: APP_CRC
        PCS_info_platformType = static_cast<uint8_t>(extract_signal_value(rx_frame.data.u8, 8, 8));
        PCS_info_appCrc = static_cast<uint32_t>(extract_signal_value(rx_frame.data.u8, 32, 32));
      }
      if (rx_frame.data.u8[0] == 0x10) { // Mux 16: SUBCOMPONENT
        PCS_info_cpu2AppCrc = static_cast<uint32_t>(extract_signal_value(rx_frame.data.u8, 32, 32));
      }
      if (rx_frame.data.u8[0] == 0x12) { // Mux 18: BOOTLOADER_GITHASH
        PCS_info_bootGitHash = static_cast<uint64_t>(extract_signal_value(rx_frame.data.u8, 10, 54));
      }
      if (rx_frame.data.u8[0] == 0x14) { // Mux 20: UDS_PROTOCOL_BOOTCRC
        PCS_info_bootUdsProtoVersion = static_cast<uint8_t>(extract_signal_value(rx_frame.data.u8, 8, 8));
        PCS_info_bootCrc = static_cast<uint32_t>(extract_signal_value(rx_frame.data.u8, 32, 32));
      }
      */
      //PCS Part Number in ASCII
      if (rx_frame.data.u8[0] == 0x19 && !parsed_PCS_partNumber) {  // Part number 1-7
        PCS_partNumber[0] = rx_frame.data.u8[1];
        PCS_partNumber[1] = rx_frame.data.u8[2];
        PCS_partNumber[2] = rx_frame.data.u8[3];
        PCS_partNumber[3] = rx_frame.data.u8[4];
        PCS_partNumber[4] = rx_frame.data.u8[5];
        PCS_partNumber[5] = rx_frame.data.u8[6];
        PCS_partNumber[6] = rx_frame.data.u8[7];
      }
      if (rx_frame.data.u8[0] == 0x1A && !parsed_PCS_partNumber) {  // Part number 8-14
        PCS_partNumber[7] = rx_frame.data.u8[1];
        PCS_partNumber[8] = rx_frame.data.u8[2];
        PCS_partNumber[9] = rx_frame.data.u8[3];
        PCS_partNumber[10] = rx_frame.data.u8[4];
        PCS_partNumber[11] = rx_frame.data.u8[5];
        PCS_partNumber[12] = rx_frame.data.u8[6];
      }
      if (PCS_partNumber[6] != 0 && PCS_partNumber[11] != 0) {
        parsed_PCS_partNumber = true;
      }
      break;
    case 0x310:  //HVP_info
      datalayer_battery->status.CAN_battery_still_alive = CAN_STILL_ALIVE;
      //Display internal HVP info and other build/version data
      if (rx_frame.data.u8[0] == 0x0A) {  // Mux 10: BUILD_HWID_COMPONENTID
        if (HVP_info_buildConfigId == 0) {
          HVP_info_buildConfigId = static_cast<uint16_t>(extract_signal_value(rx_frame.data.u8, 16, 16));
        }
        if (HVP_info_hardwareId == 0) {
          HVP_info_hardwareId = static_cast<uint16_t>(extract_signal_value(rx_frame.data.u8, 32, 16));
        }
        if (HVP_info_componentId == 0) {
          HVP_info_componentId = static_cast<uint16_t>(extract_signal_value(rx_frame.data.u8, 48, 16));
        }
      }
      /*
      if (rx_frame.data.u8[0] == 0x0B) { // Mux 11: PCBAID_ASSYID_USAGEID
        if (HVP_info_pcbaId == 0) {HVP_info_pcbaId = static_cast<uint8_t>(extract_signal_value(rx_frame.data.u8, 16, 8));}
        if (HVP_info_assemblyId == 0) {HVP_info_assemblyId = static_cast<uint8_t>(extract_signal_value(rx_frame.data.u8, 24, 8));}
        if (HVP_info_usageId == 0) {HVP_info_usageId = static_cast<uint16_t>(extract_signal_value(rx_frame.data.u8, 32, 16));}
        if (HVP_info_subUsageId == 0) {HVP_info_subUsageId = static_cast<uint16_t>(extract_signal_value(rx_frame.data.u8, 48, 16));}
      }
      if (rx_frame.data.u8[0] == 0x0D) { // Mux 13: APP_CRC
        HVP_info_platformType = static_cast<uint8_t>(extract_signal_value(rx_frame.data.u8, 8, 8));
        HVP_info_appCrc = static_cast<uint32_t>(extract_signal_value(rx_frame.data.u8, 32, 32));
      }
      if (rx_frame.data.u8[0] == 0x12) { // Mux 18: BOOTLOADER_GITHASH
        HVP_info_bootGitHash = static_cast<uint64_t>(extract_signal_value(rx_frame.data.u8, 10, 54));
      }
      if (rx_frame.data.u8[0] == 0x14) { // Mux 20: UDS_PROTOCOL_BOOTCRC
        HVP_info_bootUdsProtoVersion = static_cast<uint8_t>(extract_signal_value(rx_frame.data.u8, 8, 8));
        HVP_info_bootCrc = static_cast<uint32_t>(extract_signal_value(rx_frame.data.u8, 32, 32));
      }
      */
      break;
    default:
      break;
  }
}

void TeslaBattery::transmit_can(unsigned long currentMillis) {
  transmit_uds_can(currentMillis);

  // On-demand ECU firmware-version read, once the shared UDS channel is free.
  if (ecu_version_query_requested && !uds_is_busy()) {
    start_ecu_version_query();
  }
  // Queue a requested HVC backup as soon as the framework's sequence slot is free.
  if (hvc_bk_phase != HVC_BK_IDLE && !hvc_bk_queued && start_sequence(TESLA_UDS_HVC)) {
    hvc_bk_queued = true;
  }

  //Send any queued HVP/PCS ECU reset (requested from the web UI, gated on contactors-open above).
  if (pending_ecu_reset_req_id != 0) {
    send_ecu_reset(pending_ecu_reset_req_id);
    pending_ecu_reset_req_id = 0;
  }

  // Ensure we only send one message branch at a time, to reduce worst-case
  // runtime.
  // transmitPhase is an instance member variable (TESLA-BATTERY.h)
  if (++transmitPhase >= 5) {
    transmitPhase = 0;
  }

  //Send 10ms messages
  if (currentMillis - previousMillis10 >= INTERVAL_10_MS && transmitPhase == 0) {
    previousMillis10 = currentMillis;


    if (user_selected_tesla_digital_HVIL) {  //Special Digital HVIL mode for S/X 2024+ batteries
      if ((datalayer.system.status.inverter_allows_contactor_closing) &&
          (datalayer.system.status.system_status != FAULT)) {
        TESLA_1CF_digital_hvil.data.u8[6] = ((content_1CF_digital_hvil[index_1CF] & 0xFF00) >> 8);
        TESLA_1CF_digital_hvil.data.u8[7] = (content_1CF_digital_hvil[index_1CF] & 0x00FF);
        transmit_can_frame(&TESLA_1CF_digital_hvil);
        index_1CF = (index_1CF + 1) % 8;
        TESLA_118_digital_hvil.data.u8[0] = ((content_118_digital_hvil[index_118] & 0xFF00) >> 8);
        TESLA_118_digital_hvil.data.u8[1] = (content_118_digital_hvil[index_118] & 0x00FF);
        transmit_can_frame(&TESLA_118_digital_hvil);
        index_118 = (index_118 + 1) % 16;
      }
    } else {  //Normal handling of 118 message (Non digital HVIL version)
      //0x118 DI_systemStatus
      transmit_can_frame(&TESLA_118);
      index_1CF = 0;  //Stop broadcasting Digital HVIL 1CF and 118 to keep contactors open
      index_118 = 0;
    }

    //0x2E1 VCFRONT_status
    switch (muxNumber_TESLA_2E1) {
      case 0:
        transmit_can_frame(&TESLA_2E1_VEHICLE_AND_RAILS);
        break;
      case 1:
        transmit_can_frame(&TESLA_2E1_HOMELINK);
        break;
      case 2:
        transmit_can_frame(&TESLA_2E1_REFRIGERANT_SYSTEM);
        break;
      case 3:
        transmit_can_frame(&TESLA_2E1_LV_BATTERY_DEBUG);
        break;
      case 4:
        transmit_can_frame(&TESLA_2E1_MUX_5);
        break;
      case 5:
        transmit_can_frame(&TESLA_2E1_BODY_CONTROLS);
        break;
      default:
        break;
    }
    muxNumber_TESLA_2E1 = (muxNumber_TESLA_2E1 + 1) % 6;  //Cycle betweeen 0-1-2-3-4-5-0...
    //Generate next frames
    generateFrameCounterChecksum(TESLA_118, 8, 4, 0, 8);
  }

  //Send 50ms messages
  if (currentMillis - previousMillis50 >= INTERVAL_50_MS && transmitPhase == 1) {
    previousMillis50 = currentMillis;

    //0x221 VCFRONT_LVPowerState
    if (vehicleState == CAR_DRIVE) {
      if (alternateMux) {
        generateMuxFrameCounterChecksum(TESLA_221_DRIVE_Mux0, frameCounter_TESLA_221, 52, 4, 56, 8);
        transmit_can_frame(&TESLA_221_DRIVE_Mux0);
      } else {
        generateMuxFrameCounterChecksum(TESLA_221_DRIVE_Mux1, frameCounter_TESLA_221, 52, 4, 56, 8);
        transmit_can_frame(&TESLA_221_DRIVE_Mux1);
      }
    } else if (vehicleState == ACCESSORY) {
      if (alternateMux) {
        generateMuxFrameCounterChecksum(TESLA_221_ACCESSORY_Mux0, frameCounter_TESLA_221, 52, 4, 56, 8);
        transmit_can_frame(&TESLA_221_ACCESSORY_Mux0);
      } else {
        generateMuxFrameCounterChecksum(TESLA_221_ACCESSORY_Mux1, frameCounter_TESLA_221, 52, 4, 56, 8);
        transmit_can_frame(&TESLA_221_ACCESSORY_Mux1);
      }
    } else if (vehicleState == GOING_DOWN) {
      if (alternateMux) {
        generateMuxFrameCounterChecksum(TESLA_221_GOING_DOWN_Mux0, frameCounter_TESLA_221, 52, 4, 56, 8);
        transmit_can_frame(&TESLA_221_GOING_DOWN_Mux0);
      } else {
        generateMuxFrameCounterChecksum(TESLA_221_GOING_DOWN_Mux1, frameCounter_TESLA_221, 52, 4, 56, 8);
        transmit_can_frame(&TESLA_221_GOING_DOWN_Mux1);
      }
    } else if (vehicleState == CAR_OFF) {
      if (alternateMux) {
        generateMuxFrameCounterChecksum(TESLA_221_OFF_Mux0, frameCounter_TESLA_221, 52, 4, 56, 8);
        transmit_can_frame(&TESLA_221_OFF_Mux0);
      } else {
        generateMuxFrameCounterChecksum(TESLA_221_OFF_Mux1, frameCounter_TESLA_221, 52, 4, 56, 8);
        transmit_can_frame(&TESLA_221_OFF_Mux1);
      }
    }

    alternateMux ^= 1;  // Flips between 0 and 1. Used to Flip between sending Mux0 and Mux1 on each pass
    //Generate next new frame
    frameCounter_TESLA_221 = (frameCounter_TESLA_221 + 1) % 16;

    //0x3C2 VCLEFT_switchStatus
    if (TESLA_3C2_INITIAL_SENT == false) {
      transmit_can_frame(&TESLA_3C2_INITIAL);
      TESLA_3C2_INITIAL_SENT = true;
    } else {
      transmit_can_frame(alternateMux == 0 ? &TESLA_3C2_Mux0 : &TESLA_3C2_Mux1);
    }

    //0x39D IBST_status. First frame.
    if (TESLA_39D_INITIAL_SENT == false) {
      transmit_can_frame(&TESLA_39D_INITIAL);
      TESLA_39D_INITIAL_SENT = true;
    } else {
      transmit_can_frame(&TESLA_39D);
    }

    //0x3A1 VCFRONT_vehicleStatus: only start sending this once the pack advertises
    //>=100kW discharge (raw 7692 on 0x252), not just on contactor close - then stream it for the
    //rest of the power cycle.
    if (!tesla_3A1_armed && battery_contactor == 4 && BMS_maxDischargePower >= TESLA_3A1_ARM_DISCHARGE_RAW) {
      tesla_3A1_armed = true;
    }
    if (tesla_3A1_armed) {  // Contactors have closed AND drive power has ramped past ~100kW at least once

      // Frames to be sent only when contactors closed
      if (timeToMux3A1) {
        timeToMux3A1 = false;
        TESLA_3A1.data.u8[0] = 0xC3;
        TESLA_3A1.data.u8[1] = 0xFF;
        TESLA_3A1.data.u8[2] = 0xFF;
        TESLA_3A1.data.u8[3] = 0xFF;
        TESLA_3A1.data.u8[4] = 0x3D;
        TESLA_3A1.data.u8[5] = 0x00;
      } else {  //!timeToMux3A1
        TESLA_3A1.data.u8[0] = 0x08;
        TESLA_3A1.data.u8[1] = 0x62;
        TESLA_3A1.data.u8[2] = 0x0B;
        TESLA_3A1.data.u8[3] = 0x18;
        TESLA_3A1.data.u8[4] = 0x00;
        TESLA_3A1.data.u8[5] = 0x28;
        timeToMux3A1 = true;
      }
      TESLA_3A1.data.u8[6] = frame6_3A1[frameCounter_TESLA_3A1];
      TESLA_3A1.data.u8[7] = frame7_3A1[frameCounter_TESLA_3A1];
      //0x3A1 VCFRONT_vehicleStatus, critical otherwise VCFRONT_MIA triggered
      transmit_can_frame(&TESLA_3A1);
      frameCounter_TESLA_3A1 = (frameCounter_TESLA_3A1 + 1) % 16;
    }

    //Generate next frame
    generateFrameCounterChecksum(TESLA_39D, 8, 4, 0, 8);
  }

  //Send 100ms messages
  if (currentMillis - previousMillis100 >= INTERVAL_100_MS && transmitPhase == 2) {
    previousMillis100 = currentMillis;

    //0x102 VCLEFT_doorStatus, static
    transmit_can_frame(&TESLA_102);
    //0x103 VCRIGHT_doorStatus, static
    transmit_can_frame(&TESLA_103);
    //0x229 SCCM_rightStalk
    transmit_can_frame(&TESLA_229);
    //0x241 VCFRONT_coolant, static
    transmit_can_frame(&TESLA_241);
    //0x2D1 VCFRONT_okToUseHighPower, static
    transmit_can_frame(&TESLA_2D1);
    //0x2E8 EPBR_status queued BEFORE its 0x2A8 EPBL_status pair
    transmit_can_frame(&TESLA_2E8);
    //0x2A8 EPBL_status
    transmit_can_frame(&TESLA_2A8);
    //0x7FF GTW_carConfig
    switch (muxNumber_TESLA_7FF) {
      case 0:
        transmit_can_frame(&TESLA_7FF_Mux1);
        break;
      case 1:
        transmit_can_frame(&TESLA_7FF_Mux2);
        break;
      case 2:
        transmit_can_frame(&TESLA_7FF_Mux3);
        break;
      case 3:
        transmit_can_frame(&TESLA_7FF_Mux4);
        break;
      case 4:
        transmit_can_frame(&TESLA_7FF_Mux5);
        break;
      default:
        break;
    }
    muxNumber_TESLA_7FF = (muxNumber_TESLA_7FF + 1) % 5;  //Cycle betweeen 0-1-2-3-4-0...
    //Generate next frames
    generateTESLA_229(TESLA_229);
    //Both advance together
    generateFrameCounterChecksum(TESLA_2E8, 52, 4, 56, 8);
    generateFrameCounterChecksum(TESLA_2A8, 52, 4, 56, 8);
  }

  //Send 500ms messages
  if (currentMillis - previousMillis500 >= INTERVAL_500_MS && transmitPhase == 3) {
    previousMillis500 = currentMillis;

    transmit_can_frame(&TESLA_213);
    transmit_can_frame(&TESLA_284);
    transmit_can_frame(&TESLA_293);
    //0x313: first frame out of reset (counter 4); TESLA_313 is seeded to
    //counter 4 so the regenerate below carries the sequence on at 5.
    if (TESLA_313_INITIAL_SENT == false) {
      transmit_can_frame(&TESLA_313_INITIAL);
      TESLA_313_INITIAL_SENT = true;
    } else {
      transmit_can_frame(&TESLA_313);
    }
    transmit_can_frame(&TESLA_333);
    if (TESLA_334_INITIAL_SENT == false) {
      transmit_can_frame(&TESLA_334_INITIAL);
      TESLA_334_INITIAL_SENT = true;
    } else {
      transmit_can_frame(&TESLA_334);
    }
    transmit_can_frame(&TESLA_3B3);
    transmit_can_frame(&TESLA_55A);

    //Generate next frames
    generateTESLA_213(TESLA_213);
    generateFrameCounterChecksum(TESLA_293, 52, 4, 56, 8);
    generateFrameCounterChecksum(TESLA_313, 52, 4, 56, 8);
    generateFrameCounterChecksum(TESLA_334, 52, 4, 56, 8);
  }

  //Send 1000ms messages
  if (currentMillis - previousMillis1000 >= INTERVAL_1_S && transmitPhase == 4) {
    previousMillis1000 = currentMillis;

    transmit_can_frame(&TESLA_082);
    transmit_can_frame(&TESLA_321);

    //Generate next frames
    generateFrameCounterChecksum(TESLA_321, 52, 4, 56, 8);
  }
}

void printDebugIfActive(uint8_t symbol, const char* message) {
  //TODO: This function should later be removed in favour of the webserver diagnostics
  if (symbol == 1) {
    logging.print("Error: ");
    logging.println(message);
  }
}

void TeslaBattery::printFaultCodesIfActive() {
  if (battery_packCtrsClosingBlocked &&
      battery_packContactorSetState != 5) {  // Contactors blocked closing and not already closed
    logging.println("ERROR: Check high voltage connectors and interlock circuit, closing contactors not allowed!");
  }
  if (battery_pyroTestInProgress) {
    logging.println("ERROR: Please wait for pyro test to finish, HV cables successfully seated!");
  }
  if (datalayer.system.status.inverter_allows_contactor_closing == false) {
    logging.println(
        "ERROR: Solar inverter does not allow for contactor closing. Check communication connection to the inverter "
        "or "
        "disable the inverter protocol to proceed in Tesla battery testing mode.");
  }
  // Check each symbol and print debug information if its value is 1
  // 0X3AA: 938 HVP_alertMatrix1
  // We ignore 0x3AA for now, as on later software/firmware this is a muxed frame so values aren't correct.
  /*
  //printDebugIfActive(battery_WatchdogReset, "The processor has experienced a reset due to watchdog reset"); //Uncommented due to not affecting usage
  printDebugIfActive(battery_PowerLossReset, "The processor has experienced a reset due to power loss");
  printDebugIfActive(battery_SwAssertion, "An internal software assertion has failed");
  printDebugIfActive(battery_CrashEvent, "crash signal is detected by HVP");
  printDebugIfActive(battery_OverDchgCurrentFault,
                     "Pack discharge current is above the safe max discharge current limit!");
  printDebugIfActive(battery_OverChargeCurrentFault,
                     "Pack charge current is above the safe max charge current limit!");
  printDebugIfActive(battery_OverCurrentFault, "Pack current (discharge or charge) is above max current limit!");
  printDebugIfActive(battery_OverTemperatureFault,
                     "A pack module temperature is above the max temperature limit!");
  printDebugIfActive(battery_OverVoltageFault, "A brick voltage is above maximum voltage limit");
  printDebugIfActive(battery_UnderVoltageFault, "A brick voltage is below minimum voltage limit");
  printDebugIfActive(battery_PrimaryBmbMiaFault,
                     "Voltage and temperature readings from primary BMB chain are mia");
  printDebugIfActive(battery_SecondaryBmbMiaFault,
                     "Voltage and temperature readings from secondary BMB chain are mia");
  printDebugIfActive(battery_BmbMismatchFault,
                     "Primary and secondary BMB chain readings don't match with each other");
  printDebugIfActive(battery_BmsHviMiaFault, "BMS node is mia on HVS or HVI CAN");
  //printDebugIfActive(battery_CpMiaFault, "CP node is mia on HVS CAN"); //Uncommented due to not affecting usage
  printDebugIfActive(battery_PcsMiaFault, "PCS node is mia on HVS CAN");
  //printDebugIfActive(battery_BmsFault, "BmsFault is active"); //Uncommented due to not affecting usage
  printDebugIfActive(battery_PcsFault, "PcsFault is active");
  //printDebugIfActive(battery_CpFault, "CpFault is active"); //Uncommented due to not affecting usage
  printDebugIfActive(battery_ShuntHwMiaFault, "Shunt current reading is not available");
  printDebugIfActive(battery_PyroMiaFault, "Pyro squib is not connected");
  printDebugIfActive(battery_hvsMiaFault, "Pack contactor hw fault");
  printDebugIfActive(battery_hviMiaFault, "FC contactor hw fault");
  printDebugIfActive(battery_Supply12vFault, "Low voltage (12V) battery is below minimum voltage threshold");
  printDebugIfActive(battery_VerSupplyFault, "Energy reserve voltage supply is below minimum voltage threshold");
  printDebugIfActive(battery_HvilFault, "High Voltage Inter Lock fault is detected");
  printDebugIfActive(battery_BmsHvsMiaFault, "BMS node is mia on HVS or HVI CAN");
  printDebugIfActive(battery_PackVoltMismatchFault,
                     "Pack voltage doesn't match approximately with sum of brick voltages");
  //printDebugIfActive(battery_EnsMiaFault, "ENS line is not connected to HVC"); //Uncommented due to not affecting usage
  printDebugIfActive(battery_PackPosCtrArcFault, "HVP detectes series arc at pack contactor");
  printDebugIfActive(battery_packNegCtrArcFault, "HVP detectes series arc at FC contactor");
  printDebugIfActive(battery_ShuntHwAndBmsMiaFault, "ShuntHwAndBmsMiaFault is active");
  printDebugIfActive(battery_fcContHwFault, "fcContHwFault is active");
  printDebugIfActive(battery_robinOverVoltageFault, "robinOverVoltageFault is active");
  printDebugIfActive(battery_packContHwFault, "packContHwFault is active");
  printDebugIfActive(battery_pyroFuseBlown, "pyroFuseBlown is active");
  printDebugIfActive(battery_pyroFuseFailedToBlow, "pyroFuseFailedToBlow is active");
  //printDebugIfActive(battery_CpilFault, "CpilFault is active"); //Uncommented due to not affecting usage
  printDebugIfActive(battery_PackContactorFellOpen, "PackContactorFellOpen is active");
  printDebugIfActive(battery_FcContactorFellOpen, "FcContactorFellOpen is active");
  printDebugIfActive(battery_packCtrCloseBlocked, "packCtrCloseBlocked is active");
  printDebugIfActive(battery_fcCtrCloseBlocked, "fcCtrCloseBlocked is active");
  printDebugIfActive(battery_packContactorForceOpen, "packContactorForceOpen is active");
  printDebugIfActive(battery_fcContactorForceOpen, "fcContactorForceOpen is active");
  printDebugIfActive(battery_dcLinkOverVoltage, "dcLinkOverVoltage is active");
  printDebugIfActive(battery_shuntOverTemperature, "shuntOverTemperature is active");
  printDebugIfActive(battery_passivePyroDeploy, "passivePyroDeploy is active");
  printDebugIfActive(battery_logUploadRequest, "logUploadRequest is active");
  printDebugIfActive(battery_packCtrCloseFailed, "packCtrCloseFailed is active");
  printDebugIfActive(battery_fcCtrCloseFailed, "fcCtrCloseFailed is active");
  printDebugIfActive(battery_shuntThermistorMia, "shuntThermistorMia is active");
  */
  // 0x320 800 BMS_alertMatrix
  //TODO: This function should later be removed in favour of the webserver diagnostics
  printDebugIfActive(BMS_a017_SW_Brick_OV, "BMS_a017");
  printDebugIfActive(BMS_a018_SW_Brick_UV, "BMS_a018");
  printDebugIfActive(BMS_a019_SW_Module_OT, "BMS_a019");
  printDebugIfActive(BMS_a021_SW_Dr_Limits_Regulation, "BMS_a021");
  printDebugIfActive(BMS_a022_SW_Over_Current, "BMS_a022");
  printDebugIfActive(BMS_a023_SW_Stack_OV, "BMS_a023");
  printDebugIfActive(BMS_a024_SW_Islanded_Brick, "BMS_a024");
  printDebugIfActive(BMS_a025_SW_PwrBalance_Anomaly, "BMS_a025");
  printDebugIfActive(BMS_a026_SW_HFCurrent_Anomaly, "BMS_a026");
  printDebugIfActive(BMS_a034_SW_Passive_Isolation, "BMS_a034");
  printDebugIfActive(BMS_a035_SW_Isolation, "BMS_a035");
  printDebugIfActive(BMS_a036_SW_HvpHvilFault, "BMS_a036");
  printDebugIfActive(BMS_a037_SW_Flood_Port_Open, "BMS_a037");
  printDebugIfActive(BMS_a039_SW_DC_Link_Over_Voltage, "BMS_a039");
  printDebugIfActive(BMS_a041_SW_Power_On_Reset, "BMS_a041");
  printDebugIfActive(BMS_a042_SW_MPU_Error, "BMS_a042");
  printDebugIfActive(BMS_a043_SW_Watch_Dog_Reset, "BMS_a043");
  printDebugIfActive(BMS_a044_SW_Assertion, "BMS_a044");
  printDebugIfActive(BMS_a045_SW_Exception, "BMS_a045");
  printDebugIfActive(BMS_a046_SW_Task_Stack_Usage, "BMS_a046");
  printDebugIfActive(BMS_a047_SW_Task_Stack_Overflow, "BMS_a047");
  printDebugIfActive(BMS_a048_SW_Log_Upload_Request, "BMS_a048");
  printDebugIfActive(BMS_a050_SW_Brick_Voltage_MIA, "BMS_a050");
  printDebugIfActive(BMS_a051_SW_HVC_Vref_Bad, "BMS_a051");
  printDebugIfActive(BMS_a052_SW_PCS_MIA, "BMS_a052");
  printDebugIfActive(BMS_a053_SW_ThermalModel_Sanity, "BMS_a053");
  printDebugIfActive(BMS_a054_SW_Ver_Supply_Fault, "BMS_a054");
  printDebugIfActive(BMS_a059_SW_Pack_Voltage_Sensing, "BMS_a059");
  printDebugIfActive(BMS_a060_SW_Leakage_Test_Failure, "BMS_a060");
  printDebugIfActive(BMS_a061_robinBrickOverVoltage, "BMS_a061");
  printDebugIfActive(BMS_a062_SW_BrickV_Imbalance, "BMS_a062");
  printDebugIfActive(BMS_a063_SW_ChargePort_Fault, "BMS_a063");
  printDebugIfActive(BMS_a064_SW_SOC_Imbalance, "BMS_a064");
  printDebugIfActive(BMS_a069_SW_Low_Power, "BMS_a069");
  printDebugIfActive(BMS_a071_SW_SM_TransCon_Not_Met, "BMS_a071");
  printDebugIfActive(BMS_a075_SW_Chg_Disable_Failure, "BMS_a075");
  printDebugIfActive(BMS_a076_SW_Dch_While_Charging, "BMS_a076");
  printDebugIfActive(BMS_a077_SW_Charger_Regulation, "BMS_a077");
  printDebugIfActive(BMS_a081_SW_Ctr_Close_Blocked, "BMS_a081");
  printDebugIfActive(BMS_a082_SW_Ctr_Force_Open, "BMS_a082");
  printDebugIfActive(BMS_a083_SW_Ctr_Close_Failure, "BMS_a083");
  printDebugIfActive(BMS_a084_SW_Sleep_Wake_Aborted, "BMS_a084");
  printDebugIfActive(BMS_a087_SW_Feim_Test_Blocked, "BMS_a087");
  printDebugIfActive(BMS_a088_SW_VcFront_MIA_InDrive, "BMS_a088");
  printDebugIfActive(BMS_a089_SW_VcFront_MIA, "BMS_a089");
  printDebugIfActive(BMS_a090_SW_Gateway_MIA, "BMS_a090");
  printDebugIfActive(BMS_a091_SW_ChargePort_MIA, "BMS_a091");
  printDebugIfActive(BMS_a092_SW_ChargePort_Mia_On_Hv, "BMS_a092");
  printDebugIfActive(BMS_a094_SW_Drive_Inverter_MIA, "BMS_a094");
  printDebugIfActive(BMS_a099_SW_BMB_Communication, "BMS_a099");
  printDebugIfActive(BMS_a105_SW_One_Module_Tsense, "BMS_a105");
  printDebugIfActive(BMS_a106_SW_All_Module_Tsense, "BMS_a106");
  printDebugIfActive(BMS_a107_SW_Stack_Voltage_MIA, "BMS_a107");
  printDebugIfActive(BMS_a121_SW_NVRAM_Config_Error, "BMS_a121");
  printDebugIfActive(BMS_a122_SW_BMS_Therm_Irrational, "BMS_a122");
  printDebugIfActive(BMS_a123_SW_Internal_Isolation, "BMS_a123");
  printDebugIfActive(BMS_a127_SW_shunt_SNA, "BMS_a127");
  printDebugIfActive(BMS_a128_SW_shunt_MIA, "BMS_a128");
  printDebugIfActive(BMS_a129_SW_VSH_Failure, "BMS_a129");
  printDebugIfActive(BMS_a130_IO_CAN_Error, "BMS_a130");
  printDebugIfActive(BMS_a131_Bleed_FET_Failure, "BMS_a131");
  printDebugIfActive(BMS_a132_HW_BMB_OTP_Uncorrctbl, "BMS_a132");
  printDebugIfActive(BMS_a134_SW_Delayed_Ctr_Off, "BMS_a134");
  printDebugIfActive(BMS_a136_SW_Module_OT_Warning, "BMS_a136");
  printDebugIfActive(BMS_a137_SW_Brick_UV_Warning, "BMS_a137");
  printDebugIfActive(BMS_a139_SW_DC_Link_V_Irrational, "BMS_a139");
  printDebugIfActive(BMS_a141_SW_BMB_Status_Warning, "BMS_a141");
  printDebugIfActive(BMS_a144_Hvp_Config_Mismatch, "BMS_a144");
  printDebugIfActive(BMS_a145_SW_SOC_Change, "INFO: BMS_a145");
  printDebugIfActive(BMS_a146_SW_Brick_Overdischarged, "BMS_a146");
  printDebugIfActive(BMS_a149_SW_Missing_Config_Block, "BMS_a149");
  printDebugIfActive(BMS_a151_SW_external_isolation, "BMS_a151");
  printDebugIfActive(BMS_a156_SW_BMB_Vref_bad, "BMS_a156");
  printDebugIfActive(BMS_a157_SW_HVP_HVS_Comms, "BMS_a157");
  printDebugIfActive(BMS_a158_SW_HVP_HVI_Comms, "BMS_a158");
  printDebugIfActive(BMS_a159_SW_HVP_ECU_Error, "BMS_a159");
  printDebugIfActive(BMS_a161_SW_DI_Open_Request, "BMS_a161");
  printDebugIfActive(BMS_a162_SW_No_Power_For_Support, "BMS_a162");
  printDebugIfActive(BMS_a163_SW_Contactor_Mismatch, "BMS_a163");
  printDebugIfActive(BMS_a164_SW_Uncontrolled_Regen, "BMS_a164");
  printDebugIfActive(BMS_a165_SW_Pack_Partial_Weld, "BMS_a165");
  printDebugIfActive(BMS_a166_SW_Pack_Full_Weld, "BMS_a166");
  printDebugIfActive(BMS_a167_SW_FC_Partial_Weld, "BMS_a167");
  printDebugIfActive(BMS_a168_SW_FC_Full_Weld, "BMS_a168");
  printDebugIfActive(BMS_a169_SW_FC_Pack_Weld, "BMS_a169");
  printDebugIfActive(BMS_a170_SW_Limp_Mode, "BMS_a170");
  printDebugIfActive(BMS_a171_SW_Stack_Voltage_Sense, "BMS_a171");
  printDebugIfActive(BMS_a174_SW_Charge_Failure, "BMS_a174");
  printDebugIfActive(BMS_a176_SW_GracefulPowerOff, "BMS_a176");
  printDebugIfActive(BMS_a179_SW_Hvp_12V_Fault, "BMS_a179");
  printDebugIfActive(BMS_a180_SW_ECU_reset_blocked, "BMS_a180");
  // BMS alerts added from tesla-m3-pack-findings (firmware 2019.20.4.2)
  printDebugIfActive(BMS_a001_Pack_Config_Mismatch, "BMS_a001");
  printDebugIfActive(BMS_a055_SW_HvChain_Model_Fault, "BMS_a055");
  printDebugIfActive(BMS_a126_SW_Thermistor_Failure, "BMS_a126");
  printDebugIfActive(BMS_a135_HW_BMB_Diagnostics_Failure, "BMS_a135");
  printDebugIfActive(BMS_a143_SW_CAC_Change, "BMS_a143");
  printDebugIfActive(BMS_a155_SW_Weak_short_impedence, "BMS_a155");
  printDebugIfActive(BMS_a173_SW_Charge_Component_Fault, "BMS_a173");
  printDebugIfActive(BMS_a178_SW_Uncontrolled_Regen_PwrB, "BMS_a178");
  printFaultCodesPcsCp();
}

void TeslaBattery::printFaultCodesPcsCp() {
  // Print any active PCS (0x3A4) / CP (0x31E) alerts by their code (ECU prefix + array index),
  // reading the same datalayer arrays shown on the "More Battery Info" web page. The
  // human-readable names are no longer stored on the ESP32 (they live in the GitHub DTC JSON,
  // web_data/dtc/tesla_model3y_dtc.json); look the printed code up there, e.g. "PCS12" / "CP7".
  for (int i = 0; i < 94; i++) {
    if (datalayer_extended.tesla.PCS_alertMatrixActive[i]) {
      logging.printf("ERROR: Tesla alert PCS%d active\n", i);
    }
  }
  for (int i = 0; i < 96; i++) {
    if (datalayer_extended.tesla.CP_alertMatrixActive[i]) {
      logging.printf("ERROR: Tesla alert CP%d active\n", i);
    }
  }
}

static uint16_t tesla_build_security_key(uint8_t* out, const uint8_t* seed_response, uint16_t len, uint8_t key_subfn) {
  if (len < 18) {
    return 0;
  }
  out[0] = key_subfn;
  for (uint8_t i = 0; i < 16; i++) {
    out[i + 1] = seed_response[2 + i] ^ 0x35;
  }
  return 17;
}

void TeslaBattery::start_routine(uint16_t next_state, uint16_t routine_id) {
  uint8_t req[3] = {0x01, (uint8_t)(routine_id >> 8), (uint8_t)(routine_id & 0xFF)};
  send_sequence_message(next_state, SID::RoutineControl, req, sizeof(req));
}

void TeslaBattery::on_uds_sequence_step(uint16_t state, uint8_t sid, const uint8_t* data, uint16_t len) {
  const bool negative = (sid == kNegativeResponseSid);
  uint8_t key[17];

  switch (state) {
    case TESLA_UDS_BMS_RESET:
      send_sequence_message(TESLA_UDS_BMS_RESET_ECU, SID::DiagnosticSessionControl, (const uint8_t*)"\x03", 1);
      break;
    case TESLA_UDS_BMS_RESET_ECU:
      if (negative) {
        on_uds_sequence_timeout(state);
        break;
      }
      send_sequence_message(TESLA_UDS_BMS_RESET_DONE, SID::ECUReset, (const uint8_t*)"\x01", 1);
      break;
    case TESLA_UDS_BMS_RESET_DONE:
      if (negative) {
        on_uds_sequence_timeout(state);
        break;
      }
      set_event(EVENT_BMS_RESET_REQ_SUCCESS, 0);
      clear_event(EVENT_BMS_RESET_REQ_SUCCESS);
      break;

    case TESLA_UDS_SOC_RESET:
      send_sequence_message(TESLA_UDS_SOC_RESET_SESSION, SID::DiagnosticSessionControl, (const uint8_t*)"\x03", 1);
      break;
    case TESLA_UDS_SOC_RESET_SESSION:
      if (negative) {
        on_uds_sequence_timeout(state);
        break;
      }
      send_sequence_message(TESLA_UDS_SOC_RESET_KEY, SID::SecurityAccess, (const uint8_t*)"\x05", 1);
      break;
    case TESLA_UDS_SOC_RESET_KEY:
      if (negative || !tesla_build_security_key(key, data, len, SECURITY_LEVEL_5 + 1)) {
        on_uds_sequence_timeout(state);
        break;
      }
      send_sequence_message(TESLA_UDS_SOC_RESET_ROUTINE, SID::SecurityAccess, key, 17);
      break;
    case TESLA_UDS_SOC_RESET_ROUTINE:
      if (negative) {
        on_uds_sequence_timeout(state);
        break;
      }
      start_routine(TESLA_UDS_SOC_RESET_RESULT, RID_RESET_SOC);
      break;
    case TESLA_UDS_SOC_RESET_RESULT:
      if (negative || len < 5 || data[4] != 0x01) {
        on_uds_sequence_timeout(state);
        break;
      }
      set_event(EVENT_BATTERY_SOC_RESET_SUCCESS, 0, battery_index);
      clear_event(EVENT_BATTERY_SOC_RESET_SUCCESS, battery_index);
      send_sequence_message(TESLA_UDS_SOC_RESET_ECU, SID::DiagnosticSessionControl, (const uint8_t*)"\x01", 1);
      break;
    case TESLA_UDS_SOC_RESET_ECU:
      send_sequence_message(TESLA_UDS_SOC_RESET_DONE, SID::ECUReset, (const uint8_t*)"\x01", 1);
      break;
    case TESLA_UDS_SOC_RESET_DONE:
      break;

    case TESLA_UDS_ISO_CLEAR:
      send_sequence_message(TESLA_UDS_ISO_CLEAR_SESSION, SID::DiagnosticSessionControl, (const uint8_t*)"\x03", 1);
      break;
    case TESLA_UDS_ISO_CLEAR_SESSION:
      if (negative) {
        break;
      }
      {
        uint8_t seed_req[1] = {SECURITY_LEVEL_19};
        send_sequence_message(TESLA_UDS_ISO_CLEAR_KEY, SID::SecurityAccess, seed_req, 1);
      }
      break;
    case TESLA_UDS_ISO_CLEAR_KEY:
      if (negative || !tesla_build_security_key(key, data, len, SECURITY_LEVEL_19 + 1)) {
        break;
      }
      send_sequence_message(TESLA_UDS_ISO_CLEAR_COUNTERS, SID::SecurityAccess, key, 17);
      break;
    case TESLA_UDS_ISO_CLEAR_COUNTERS:
      if (negative) {
        break;
      }
      start_routine(TESLA_UDS_ISO_CLEAR_FLOOD, RID_RESET_ISOLATION_COUNTERS);
      break;
    case TESLA_UDS_ISO_CLEAR_FLOOD:
      start_routine(TESLA_UDS_ISO_CLEAR_INTERNAL, RID_CLEAR_BATTERY_COOLANT_FLOOD_ALERT);
      break;
    case TESLA_UDS_ISO_CLEAR_INTERNAL:
      start_routine(TESLA_UDS_ISO_CLEAR_DONE, RID_CLEAR_INTERNAL_ISOLATION_ALERT);
      break;
    case TESLA_UDS_ISO_CLEAR_DONE:
      break;

    // ODIN "Blocked Contactors - Attempt to Clear" (Gen3): VSH latch reset, isolation-condition clear
    // (counters + coolant flood + internal isolation), HVBMS reset, then HVP reset.
    // All HVBMS routines run under one extended session + security.
    case TESLA_UDS_CBC:
      send_sequence_message(TESLA_UDS_CBC_SESSION, SID::DiagnosticSessionControl, (const uint8_t*)"\x03", 1);
      break;
    case TESLA_UDS_CBC_SESSION:
      if (negative) {
        break;
      }
      {
        uint8_t seed_req[1] = {SECURITY_LEVEL_19};
        send_sequence_message(TESLA_UDS_CBC_KEY, SID::SecurityAccess, seed_req, 1);
      }
      break;
    case TESLA_UDS_CBC_KEY:
      if (negative || !tesla_build_security_key(key, data, len, SECURITY_LEVEL_19 + 1)) {
        break;
      }
      start_routine(TESLA_UDS_CBC_VSH, RID_RESET_VSH_FAULT);
      break;
    case TESLA_UDS_CBC_VSH:
      start_routine(TESLA_UDS_CBC_COUNTERS, RID_RESET_ISOLATION_COUNTERS);
      break;
    case TESLA_UDS_CBC_COUNTERS:
      start_routine(TESLA_UDS_CBC_FLOOD, RID_CLEAR_BATTERY_COOLANT_FLOOD_ALERT);
      break;
    case TESLA_UDS_CBC_FLOOD:
      start_routine(TESLA_UDS_CBC_INTERNAL, RID_CLEAR_INTERNAL_ISOLATION_ALERT);
      break;
    case TESLA_UDS_CBC_INTERNAL:
      // Reset the HVBMS itself (ECUReset on 0x602). Failure of the alert-clear routines is not fatal
      // to the attempt - ODIN still resets the ECUs - so we proceed regardless of `negative`.
      send_sequence_message(TESLA_UDS_CBC_HVBMS, SID::ECUReset, (const uint8_t*)"\x01", 1);
      break;
    case TESLA_UDS_CBC_HVBMS:
      // HVBMS reboot acknowledged (or timed out). Queue the HVP ECUReset (0x610), completing the set.
      pending_ecu_reset_req_id = HVP_UDS_REQUEST;
      break;
    case TESLA_UDS_CBC_DONE:
      break;

    case TESLA_UDS_QUERY:
      send_sequence_message(TESLA_UDS_QUERY_READ, SID::DiagnosticSessionControl, (const uint8_t*)"\x03", 1);
      break;
    case TESLA_UDS_QUERY_READ:
      if (negative) {
        break;
      }
      send_sequence_message(TESLA_UDS_QUERY_PARSE, SID::ReadDataByIdentifier, (const uint8_t*)"\xF0\x14", 2);
      break;
    case TESLA_UDS_QUERY_PARSE:
      if (negative || len < 15) {
        break;
      }
      for (uint8_t i = 0; i < 12; i++) {
        battery_partNumber[i] = data[3 + i];
      }
      parsed_battery_partNumber = true;
      break;

    case TESLA_UDS_VERSION:  // entry: open an extended session on the BMS
      send_sequence_message(TESLA_UDS_VERSION_SESSION, SID::DiagnosticSessionControl, (const uint8_t*)"\x03", 1);
      break;
    // The rest of the version chain: store whatever arrived, then let ecu_version_continue send
    // the next read (or stamp the fingerprint at the end). Negative responses store nothing.
    case TESLA_UDS_VERSION_SESSION:
    case TESLA_UDS_VERSION_F180:
    case TESLA_UDS_VERSION_0F00:
    case TESLA_UDS_VERSION_HVP_F180:
    case TESLA_UDS_VERSION_HVP_0F00:
    case TESLA_UDS_VERSION_PCS_F180:
    case TESLA_UDS_VERSION_PCS_0F00:
    case TESLA_UDS_VERSION_PCS_0F02:
    case TESLA_UDS_VERSION_PCS_F012:
      if (!negative) {
        EcuVersion& ev = (state <= TESLA_UDS_VERSION_0F00)       ? ecu_ver_bms
                         : (state <= TESLA_UDS_VERSION_HVP_0F00) ? ecu_ver_hvp
                                                                 : ecu_ver_pcs;
        ecu_version_store_did(ev, data, len);
      }
      ecu_version_continue(state);
      break;

    // ---- startup identity pass (HVBMS 0x602/0x612) ----
    case TESLA_UDS_IDENT:  // entry: open an extended session
      send_sequence_message(TESLA_UDS_IDENT_SESSION, SID::DiagnosticSessionControl, (const uint8_t*)"\x03", 1);
      break;
    case TESLA_UDS_IDENT_SESSION:  // session ack; request the pack serial 0xF015
      if (negative) {
        ident_requested = false;  // bounded retry; the 0x72A broadcast path still fills these in
        break;
      }
      send_sequence_message(TESLA_UDS_IDENT_SERIAL, SID::ReadDataByIdentifier, (const uint8_t*)"\xF0\x15", 2);
      break;
    case TESLA_UDS_IDENT_SERIAL:
      // 14 ASCII bytes, the same string 0x72A carries across its two muxes.
      if (!negative && len >= 3 + sizeof(battery_serialNumber) && data[1] == 0xF0 && data[2] == 0x15) {
        memcpy(battery_serialNumber, data + 3, sizeof(battery_serialNumber));
        // Must derive the date here too. Setting parsed_battery_serialNumber locks the 0x72A parser
        // out for the rest of the boot, and this read normally wins the race - so leaving it to the
        // broadcast meant the date was never computed at all.
        derive_manufacture_date_from_serial();
        parsed_battery_serialNumber = true;
        identity_from_uds = true;
      }
      send_sequence_message(TESLA_UDS_IDENT_PACKINFO, SID::ReadDataByIdentifier, (const uint8_t*)"\x04\x50", 2);
      break;
    case TESLA_UDS_IDENT_PACKINFO:
      // PACK_INFO carries BIRTHDATE_EPOCH_TIME, but it never produced a usable date on the bench pack
      // Response is read for parity/to keep the sequence, but nothing is kept from it currently.
      send_sequence_message(TESLA_UDS_IDENT_PACK_PN, SID::ReadDataByIdentifier, (const uint8_t*)"\xF0\x14", 2);
      break;
    case TESLA_UDS_IDENT_PACK_PN:
      // PACKAGE_PART_NUMBER is 20 bytes; a Tesla part number is the leading 12 ("1104420-00-F").
      if (!negative && len >= 15 && data[1] == 0xF0 && data[2] == 0x14) {
        memcpy(battery_partNumber, data + 3, sizeof(battery_partNumber));
        parsed_battery_partNumber = true;
      }
      send_sequence_message(TESLA_UDS_IDENT_BOARD_PN, SID::ReadDataByIdentifier, (const uint8_t*)"\xF0\x12", 2);
      break;
    case TESLA_UDS_IDENT_BOARD_PN:
      // BOARD_PART_NUMBER on the HVBMS - the BMS board itself, distinct from the pack package above.
      if (!negative && len >= 15 && data[1] == 0xF0 && data[2] == 0x12) {
        memcpy(BMS_partNumber, data + 3, sizeof(BMS_partNumber));
        parsed_BMS_partNumber = true;
      }
      // Finish with the PCS board part number (the HVP shares the HVC board, so it is not asked).
      retarget_uds(PCS_UDS_REQUEST, PCS_UDS_RESPONSE);
      {
        uint8_t req[2] = {(uint8_t)(DID_BOARD_PART_NUMBER >> 8), (uint8_t)DID_BOARD_PART_NUMBER};
        if (!send_sequence_message(TESLA_UDS_IDENT_PCS_PN, SID::ReadDataByIdentifier, req, 2, 3, 1)) {
          retarget_uds(BMS_UDS_REQUEST, BMS_UDS_RESPONSE);
        }
      }
      break;
    case TESLA_UDS_IDENT_PCS_PN:
      // The PCS also broadcasts its part number on 0x3C4; whichever lands first wins.
      if (!negative && len >= 15 && data[1] == 0xF0 && data[2] == 0x12 && !parsed_PCS_partNumber) {
        memcpy(PCS_partNumber, data + 3, 12);
        PCS_partNumber[12] = 0;
        parsed_PCS_partNumber = true;
      }
      retarget_uds(BMS_UDS_REQUEST, BMS_UDS_RESPONSE);
      break;

    case TESLA_UDS_BLEED:
      send_sequence_message(TESLA_UDS_BLEED_START, SID::DiagnosticSessionControl, (const uint8_t*)"\x03", 1);
      break;
    case TESLA_UDS_BLEED_START:
      if (negative) {
        break;
      }
      send_sequence_message(TESLA_UDS_BLEED_COUNT, SID::RoutineControl, (const uint8_t*)"\x01\x04\x34", 3);
      break;
    case TESLA_UDS_BLEED_COUNT:
      if (negative || len < 6) {
        break;
      }
      brick_bleed_pack_count = ((uint16_t)data[4] << 8) | data[5];
      if (brick_bleed_pack_count == 0 || brick_bleed_pack_count > TESLA_MAX_BRICKS) {
        brick_bleed_pack_count = TESLA_MAX_BRICKS;
      }
      request_bleed_results(1);
      break;
    case TESLA_UDS_BLEED_RESULT: {
      if (negative || len < 8) {
        break;
      }
      uint16_t start = ((uint16_t)data[4] << 8) | data[5];
      uint16_t end = ((uint16_t)data[6] << 8) | data[7];
      for (uint16_t b = start; b <= end && b <= brick_bleed_pack_count && b <= TESLA_MAX_BRICKS; b++) {
        uint16_t k = b - start;
        if ((uint16_t)(8 + k * 4 + 3) >= len) {
          break;
        }
        const uint8_t* p = &data[8 + k * 4];
        brick_bleed_time_s[b - 1] =
            ((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) | ((uint32_t)p[2] << 8) | p[3];
      }
      if (end >= start && end < brick_bleed_pack_count) {
        request_bleed_results(end + 1);
      } else {
        finalize_bleed_read();
      }
      break;
    }

    case TESLA_UDS_CAC_RESET:
      send_sequence_message(TESLA_UDS_CAC_RESET_ROUTINE, SID::DiagnosticSessionControl, (const uint8_t*)"\x03", 1);
      break;
    case TESLA_UDS_CAC_RESET_ROUTINE:
      if (negative) {
        set_event(EVENT_CAC_RESET_FAIL, 0);
        clear_event(EVENT_CAC_RESET_FAIL);
        break;
      }
      start_routine(TESLA_UDS_CAC_RESET_RESULT, RID_RESET_CAC);
      break;
    case TESLA_UDS_CAC_RESET_RESULT:
      if (negative || len < 5 || data[4] != 0x01) {
        logging.println("CAN UDS: CAC reset rejected");
        set_event(EVENT_CAC_RESET_FAIL, 0);
        clear_event(EVENT_CAC_RESET_FAIL);
        break;
      }
      logging.println("CAN UDS: CAC reset accepted");
      set_event(EVENT_CAC_RESET_SUCCESS, 0);
      clear_event(EVENT_CAC_RESET_SUCCESS);
      break;

    case TESLA_UDS_BRICKV:
      send_sequence_message(TESLA_UDS_BRICKV_READ, SID::DiagnosticSessionControl, (const uint8_t*)"\x03", 1);
      break;
    case TESLA_UDS_BRICKV_READ:
      if (negative) {
        break;
      }
      send_sequence_message(TESLA_UDS_BRICKV_PARSE, SID::ReadDataByIdentifier, (const uint8_t*)"\x04\x10", 2);
      break;
    case TESLA_UDS_BRICKV_PARSE: {
      //62 04 10, then PACK_CURRENT(4) + PACK_VOLTAGE(4), then TESLA_BRICKV_COUNT x u16 big-endian (0.1 mV/bit).
      if (negative || len < (uint16_t)(3 + 8 + 2 * TESLA_BRICKV_COUNT)) {
        break;
      }
      const uint8_t* v = &data[3 + 8];
      uint16_t count = 0;
      for (uint16_t k = 0; k < TESLA_BRICKV_COUNT && k < MAX_AMOUNT_CELLS; k++) {
        uint16_t mv = (uint16_t)(((uint16_t)v[2 * k] << 8) | v[2 * k + 1]) / 10;
        if (mv > 500) {  //a real brick is >0.5 V; 0 marks an unused slot on a smaller pack
          datalayer_battery->status.cell_voltages_mV[k] = mv;
          count = k + 1;
        }
      }
      if (count > 0) {
        datalayer_battery->info.number_of_cells = count;
      }
      break;
    }

    case TESLA_UDS_ISO_MON:
      send_sequence_message(TESLA_UDS_ISO_MON_SESSION, SID::DiagnosticSessionControl, (const uint8_t*)"\x03", 1);
      break;
    case TESLA_UDS_ISO_MON_SESSION:
      if (negative) {
        break;
      }
      {
        uint8_t seed_req[1] = {SECURITY_LEVEL_5};
        send_sequence_message(TESLA_UDS_ISO_MON_KEY, SID::SecurityAccess, seed_req, 1);
      }
      break;
    case TESLA_UDS_ISO_MON_KEY:
      if (negative || !tesla_build_security_key(key, data, len, SECURITY_LEVEL_5 + 1)) {
        break;
      }
      send_sequence_message(TESLA_UDS_ISO_MON_ROUTINE, SID::SecurityAccess, key, 17);
      break;
    case TESLA_UDS_ISO_MON_ROUTINE:
      if (negative) {
        break;
      }
      {
        uint8_t sub = isolation_monitor_disabling ? RoutineAction::Start : RoutineAction::Stop;
        uint8_t req[3] = {sub, (uint8_t)(RID_DISABLE_ISOLATION_MONITOR >> 8),
                          (uint8_t)(RID_DISABLE_ISOLATION_MONITOR & 0xFF)};
        send_sequence_message(TESLA_UDS_ISO_MON_RESULT, SID::RoutineControl, req, sizeof(req));
      }
      break;
    case TESLA_UDS_ISO_MON_RESULT:
      if (negative) {
        logging.println("CAN UDS: isolation monitor request failed");
        break;
      }
      if (!isolation_monitor_disabling) {
        logging.println("CAN UDS: isolation monitor re-enabled");
      } else if (len >= 5 && data[4] == 0x00) {
        logging.println("CAN UDS: isolation monitor disabled");
      } else {
        logging.println("CAN UDS: isolation monitor disable rejected");
      }
      break;

    case TESLA_UDS_HVC:  // entry
      if (hvc_bk_phase == HVC_BK_IDLE) {
        break;  // stale start; nothing to read
      }
      hvc_bk_step = 0;
      if (!hvc_send_current()) {
        finish_hvc_backup();
      }
      break;
    case TESLA_UDS_HVC_HVBMS:
    case TESLA_UDS_HVC_HVBMS_CAL:
    case TESLA_UDS_HVC_HVP:
    case TESLA_UDS_HVC_CAL_RT:
      hvc_handle_response(state, sid, data, len);
      break;

    default:
      break;
  }
}

void TeslaBattery::request_bleed_results(uint16_t start_brick) {
  uint8_t req[5] = {0x03, 0x04, 0x34, (uint8_t)(start_brick >> 8), (uint8_t)(start_brick & 0xFF)};
  send_sequence_message(TESLA_UDS_BLEED_RESULT, SID::RoutineControl, req, sizeof(req));
}

void TeslaBattery::finalize_bleed_read() {
  uint8_t active = 0;
  for (uint16_t i = 0; i < brick_bleed_pack_count && i < MAX_AMOUNT_CELLS; i++) {
    bool bleeding = brick_bleed_has_prev && (brick_bleed_time_s[i] > brick_bleed_prev_s[i]);
    datalayer_battery->status.cell_balancing_status[i] = bleeding;
    if (bleeding) {
      active++;
    }
    datalayer_extended.tesla.BMS_brickBleedTime_s[i] = brick_bleed_time_s[i];
    brick_bleed_prev_s[i] = brick_bleed_time_s[i];
  }
  brick_bleed_has_prev = true;
  datalayer_extended.tesla.BMS_brickCount = brick_bleed_pack_count;
  datalayer_extended.tesla.BMS_brickBleedActiveCount = active;
  datalayer_extended.tesla.BMS_brickBleedValid = true;
}

void TeslaBattery::on_uds_sequence_timeout(uint16_t state) {
  if (state >= TESLA_UDS_BMS_RESET && state <= TESLA_UDS_BMS_RESET_DONE) {
    set_event(EVENT_BMS_RESET_REQ_FAIL, 0);
    clear_event(EVENT_BMS_RESET_REQ_FAIL);
  } else if (state >= TESLA_UDS_SOC_RESET && state <= TESLA_UDS_SOC_RESET_DONE) {
    set_event(EVENT_BATTERY_SOC_RESET_FAIL, 0, battery_index);
    clear_event(EVENT_BATTERY_SOC_RESET_FAIL, battery_index);
  } else if (state >= TESLA_UDS_BRICKV && state <= TESLA_UDS_BRICKV_PARSE) {
    brickv_requested = false;  //allow a bounded retry; 0x401 remains the fallback
  } else if (state >= TESLA_UDS_VERSION && state <= TESLA_UDS_VERSION_PCS_F012) {
    ecu_version_continue(state);  //an ECU may be absent; move the chain on
  } else if (state >= TESLA_UDS_IDENT && state <= TESLA_UDS_IDENT_BOARD_PN) {
    ident_requested = false;  //allow a bounded retry; the 0x72A / 0x3C4 broadcasts remain the fallback
  } else if (state == TESLA_UDS_IDENT_PCS_PN) {
    retarget_uds(BMS_UDS_REQUEST, BMS_UDS_RESPONSE);  //PCS silent; the 0x3C4 broadcast remains the fallback
  } else if (state >= TESLA_UDS_HVC && state <= TESLA_UDS_HVC_CAL_RT) {
    hvc_handle_response(state, 0, nullptr, 0);
  } else if (state >= TESLA_UDS_CBC && state <= TESLA_UDS_CBC_DONE) {
    // The HVBMS ECUReset can reboot the ECU without answering; if we time out at that step, still
    // queue the HVP reset so the attempt completes. Earlier steps timing out are a genuine failure.
    if (state == TESLA_UDS_CBC_HVBMS) {
      pending_ecu_reset_req_id = HVP_UDS_REQUEST;
    } else {
      set_event(EVENT_CLEAR_BLOCKED_CONTACTORS_FAIL, 0);
      clear_event(EVENT_CLEAR_BLOCKED_CONTACTORS_FAIL);
    }
  }
}

// ---- ECU firmware-version fingerprint ------------------------------------------------------------

static void ver_append_token(char* buf, size_t cap, size_t& pos, const char* key, const char* did,
                             const uint8_t* bytes, uint8_t n) {
  if (n == 0 || pos >= cap) {
    return;
  }
  int w = snprintf(buf + pos, cap - pos, ";%s:%s=", key, did);
  if (w < 0 || (size_t)w >= cap - pos) {
    return;
  }
  pos += (size_t)w;
  static const char* hexchars = "0123456789abcdef";
  for (uint8_t i = 0; i < n && pos + 2 < cap; i++) {
    buf[pos++] = hexchars[bytes[i] >> 4];
    buf[pos++] = hexchars[bytes[i] & 0x0F];
  }
  buf[pos] = '\0';
}

void TeslaBattery::start_ecu_version_query() {
  ecu_version_query_requested = false;
  ecu_ver_bms = EcuVersion();
  ecu_ver_hvp = EcuVersion();
  ecu_ver_pcs = EcuVersion();
  datalayer_extended.tesla.ecu_version_query_state = 1;  // querying
  datalayer_extended.tesla.ecu_version_fingerprint[0] = '\0';
  if (!start_sequence(TESLA_UDS_VERSION)) {
    finish_ecu_version_query();  // channel busy despite the guard; report the empty result
  }
}

void TeslaBattery::ecu_version_store_did(EcuVersion& ev, const uint8_t* data, uint16_t len) {
  if (len < 3 || data[0] != 0x62) {
    return;  // session acks and negative responses store nothing
  }
  uint16_t did = (uint16_t)((data[1] << 8) | data[2]);
  const uint8_t* val = data + 3;
  uint16_t vlen = (uint16_t)(len - 3);
  if (did == DID_BOOT_SW_ID) {
    uint8_t n = (uint8_t)(vlen > sizeof(ev.f180) ? sizeof(ev.f180) : vlen);
    memcpy(ev.f180, val, n);
    ev.f180_len = n;
  } else if (did == DID_APP_CRC && vlen >= 4) {
    memcpy(ev.crc0f00, val, 4);
    ev.crc0f00_valid = true;
  } else if (did == DID_APP_CRC_CPU2 && vlen >= 4) {
    memcpy(ev.crc0f02, val, 4);
    ev.crc0f02_valid = true;
  } else if (did == DID_BOARD_PART_NUMBER && vlen >= 12 && !parsed_PCS_partNumber) {
    // Only the PCS is asked for 0xF012 here; the 0x3C4 broadcast may already have supplied it.
    memcpy(PCS_partNumber, val, 12);
    PCS_partNumber[12] = 0;
    parsed_PCS_partNumber = true;
  }
}

bool TeslaBattery::ecu_ver_send(uint16_t next_state, uint16_t did) {
  uint8_t req[2] = {(uint8_t)(did >> 8), (uint8_t)did};
  return send_sequence_message(next_state, SID::ReadDataByIdentifier, req, 2, 3, 1);
}

// Send the read that follows `state` in the BMS -> HVP -> PCS chain. Reached from both the
// response and the timeout path, so an absent ECU cannot stall the query.
void TeslaBattery::ecu_version_continue(uint16_t state) {
  uint16_t next = 0, did = 0;
  switch (state) {
    case TESLA_UDS_VERSION_SESSION:
      next = TESLA_UDS_VERSION_F180;
      did = DID_BOOT_SW_ID;
      break;
    case TESLA_UDS_VERSION_F180:
      next = TESLA_UDS_VERSION_0F00;
      did = DID_APP_CRC;
      break;
    case TESLA_UDS_VERSION_0F00:
      retarget_uds(HVP_UDS_REQUEST, HVP_UDS_RESPONSE);
      next = TESLA_UDS_VERSION_HVP_F180;
      did = DID_BOOT_SW_ID;
      break;
    case TESLA_UDS_VERSION_HVP_F180:
      next = TESLA_UDS_VERSION_HVP_0F00;
      did = DID_APP_CRC;
      break;
    case TESLA_UDS_VERSION_HVP_0F00:
      retarget_uds(PCS_UDS_REQUEST, PCS_UDS_RESPONSE);
      next = TESLA_UDS_VERSION_PCS_F180;
      did = DID_BOOT_SW_ID;
      break;
    case TESLA_UDS_VERSION_PCS_F180:
      next = TESLA_UDS_VERSION_PCS_0F00;
      did = DID_APP_CRC;
      break;
    case TESLA_UDS_VERSION_PCS_0F00:
      next = TESLA_UDS_VERSION_PCS_0F02;
      did = DID_APP_CRC_CPU2;
      break;
    case TESLA_UDS_VERSION_PCS_0F02:
      next = TESLA_UDS_VERSION_PCS_F012;
      did = DID_BOARD_PART_NUMBER;
      break;
    case TESLA_UDS_VERSION_PCS_F012:
    default:
      finish_ecu_version_query();
      return;
  }
  if (!ecu_ver_send(next, did)) {
    finish_ecu_version_query();
  }
}

void TeslaBattery::finish_ecu_version_query() {
  retarget_uds(BMS_UDS_REQUEST, BMS_UDS_RESPONSE);
  char* fp = datalayer_extended.tesla.ecu_version_fingerprint;
  size_t cap = sizeof(datalayer_extended.tesla.ecu_version_fingerprint);
  int w = snprintf(fp, cap, "TESLAVER1");
  size_t pos = (w > 0) ? (size_t)w : 0;
  ver_append_token(fp, cap, pos, "hvbms", "F180", ecu_ver_bms.f180, ecu_ver_bms.f180_len);
  if (ecu_ver_bms.crc0f00_valid) {
    ver_append_token(fp, cap, pos, "hvbms", "0F00", ecu_ver_bms.crc0f00, 4);
  }
  ver_append_token(fp, cap, pos, "hvp", "F180", ecu_ver_hvp.f180, ecu_ver_hvp.f180_len);
  if (ecu_ver_hvp.crc0f00_valid) {
    ver_append_token(fp, cap, pos, "hvp", "0F00", ecu_ver_hvp.crc0f00, 4);
  }
  ver_append_token(fp, cap, pos, "pcs", "F180", ecu_ver_pcs.f180, ecu_ver_pcs.f180_len);
  if (ecu_ver_pcs.crc0f00_valid) {
    ver_append_token(fp, cap, pos, "pcs", "0F00", ecu_ver_pcs.crc0f00, 4);
  }
  if (ecu_ver_pcs.crc0f02_valid) {
    ver_append_token(fp, cap, pos, "pcs", "0F02", ecu_ver_pcs.crc0f02, 4);
  }
  datalayer_extended.tesla.ecu_version_query_state = 2;  // complete
}

// Characters 4-5 of the serial are the build year, 6-8 the Julian day: "TG322221000ZD4" is day 221
// of 2022. Both the 0x72A broadcast and the UDS 0xF015 read produce the same 14 characters, so the
// date is derived here rather than in either parser.
void TeslaBattery::derive_manufacture_date_from_serial() {
  char yearStr[5];  // Full year string (including the "20" prefix)
  snprintf(yearStr, sizeof(yearStr), "20%c%c", battery_serialNumber[3], battery_serialNumber[4]);
  int year = atoi(yearStr);
  char dayStr[4];
  snprintf(dayStr, sizeof(dayStr), "%c%c%c", battery_serialNumber[5], battery_serialNumber[6],
           battery_serialNumber[7]);
  int day = atoi(dayStr);
  battery_manufactureDate = dayOfYearToDate(year, day);
}

// One-shot pack identity read at startup: pack serial, part numbers and PACK_INFO off the HVBMS,
// ending with the PCS board part number. Everything is security_level 0, so an extended
// session is enough - nothing written, no contactor movement.
void TeslaBattery::start_identity_query() {
  start_sequence(TESLA_UDS_IDENT);
}

//UDS ECUReset, hardReset with suppressed positive response (0x11 0x81), byte-for-byte the reboot
//ODIN issues after flashing. Accepted in the default session; no response to wait for.
void TeslaBattery::send_ecu_reset(uint16_t req_id) {
  CAN_frame f = {};
  f.FD = false;
  f.ext_ID = false;
  f.DLC = 8;
  f.ID = req_id;
  f.data.u8[0] = 0x02;  // ISO-TP single frame, 2 payload bytes
  f.data.u8[1] = 0x11;  // ECUReset
  f.data.u8[2] = 0x81;  // hardReset (0x01) + suppressPositiveResponse (0x80), matches ODIN
  f.data.u8[3] = 0x55;  // Tesla request padding
  f.data.u8[4] = 0x55;
  f.data.u8[5] = 0x55;
  f.data.u8[6] = 0x55;
  f.data.u8[7] = 0x55;
  transmit_can_frame(&f);
}

// HVC backup engine
namespace {
enum HvcScalarTag : uint8_t {
  TAG_ENERGY, TAG_PACK_INFO, TAG_HIST_COUNT, TAG_ASM_HVBMS, TAG_USE_HVBMS, TAG_CFG_HVBMS,
  TAG_PKG_PN, TAG_PKG_SN, TAG_BOARD_PN, TAG_BOARD_SN, TAG_ASM_HVP, TAG_USE_HVP, TAG_CFG_HVP,
  TAG_HVP_CAL,
};
// min_len is the shortest response that still decodes without inventing data; shorter positive
// replies are left MISSING (rather that than incorrect data in a restore backup). ASCII fields
// use 1, since packs answer them shorter than the declared length. ENERGY_COUNTERS is 28, not the
// ODX's 32: 2022 packs send the Ah pair as two uint16 and stop there, and ODIN's own clone backup
// declares 28 too - the length picks the layout at decode time.
struct HvcScalar {
  uint16_t did;
  uint8_t tag;
  uint8_t min_len;
};
const HvcScalar HVC_SCALARS[] = {
    {0x0420, TAG_ENERGY, 28},   {0x0450, TAG_PACK_INFO, 10}, {0x0436, TAG_HIST_COUNT, 8},
    {0xF01C, TAG_ASM_HVBMS, 1}, {0xF01D, TAG_USE_HVBMS, 2},  {0x0439, TAG_CFG_HVBMS, 64},
    {0xF014, TAG_PKG_PN, 1},    {0xF015, TAG_PKG_SN, 1},     {0xF012, TAG_BOARD_PN, 1},
    {0xF013, TAG_BOARD_SN, 1},
};
// Calibration records are read via the CALIBRATION_RECORD routine getResults (HVP 0x438, HVBMS
// 0x439) with a one-byte record id; the 0x0500+id DIDs are kept as an HVP fallback. HVBMS ids
// 0/1/6 are the required-for-recovery set.
constexpr uint16_t HVP_CAL_DID_BASE = 0x0500;
constexpr uint16_t RT_HVP_CAL = 0x0438;
constexpr uint16_t RT_HVBMS_CAL = 0x0439;  // routine, distinct from the 0x0439 CONFIGURATION_RECORD DID
constexpr uint8_t HVBMS_CAL_IDS[TESLA_HVBMS_NUM_CAL] = {0, 1, 6};
const HvcScalar HVC_SCALARS_HVP[] = {
    {0xF01C, TAG_ASM_HVP, 1}, {0xF01D, TAG_USE_HVP, 2}, {0x0439, TAG_CFG_HVP, 64},
    {0x0500, TAG_HVP_CAL, 4}, {0x0501, TAG_HVP_CAL, 4}, {0x0502, TAG_HVP_CAL, 4},
    {0x0503, TAG_HVP_CAL, 4}, {0x0504, TAG_HVP_CAL, 4}, {0x0505, TAG_HVP_CAL, 4},
    {0x0506, TAG_HVP_CAL, 4}, {0x0507, TAG_HVP_CAL, 4},
};
constexpr uint8_t N_HVC_SCALARS = sizeof(HVC_SCALARS) / sizeof(HVC_SCALARS[0]);
constexpr uint8_t N_HVC_SCALARS_HVP = sizeof(HVC_SCALARS_HVP) / sizeof(HVC_SCALARS_HVP[0]);
constexpr uint16_t HVC_STEP_TIMEOUT_TICKS = 3;  // per attempt, in the framework's 100ms ticks
constexpr uint8_t HVC_STEP_RETRIES = 2;
uint32_t rd_be32(const uint8_t* p) {
  return ((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) | ((uint32_t)p[2] << 8) | p[3];
}
uint16_t rd_be16(const uint8_t* p) {
  return (uint16_t)((p[0] << 8) | p[1]);
}
}  // namespace

bool TeslaBattery::start_hvc_backup() {
  if (hvc_bk_phase != HVC_BK_IDLE) {
    return false;
  }
  // A silent pack would just time every read out, with no way to tell a refusal from absence.
  if (datalayer_battery->status.CAN_battery_still_alive == 0) {
    logging.println("INFO: HVC backup refused - battery is not responding on CAN");
    return false;
  }
  // The backup may hold 0x221 in ACCESSORY, which is not something to do during a fault or stop.
  if (datalayer.system.status.system_status == FAULT || datalayer.system.info.equipment_stop_active) {
    logging.println("INFO: HVC backup refused - emulator faulted or equipment stop active");
    return false;
  }
  if (!hvc_backup) {
    hvc_backup = new TeslaHvcBackup();
    if (!hvc_backup) {
      logging.println("INFO: HVC backup refused - out of memory");
      return false;
    }
  }
  // Queue the sequence now if the framework's slot is free; otherwise transmit_can keeps trying,
  // so a backup asked for while another sequence occupies the channel waits instead of refusing.
  hvc_bk_queued = start_sequence(TESLA_UDS_HVC);
  // If the pack is on its way down (emulator paused, 0x221 walking to OFF), hold ACCESSORY for the
  // run and put the mode back afterwards. The contactors are left alone, as ODIN does.
  if (!hvc_bk_accessory_held && (vehicleState == GOING_DOWN || vehicleState == CAR_OFF)) {
    hvc_bk_saved_accessory_forced = accessory_mode_forced;
    hvc_bk_saved_balance_at_rest = balance_at_rest_enabled;
    hvc_bk_accessory_held = true;
    balance_at_rest_enabled = false;  // it would flip the mode back out from under us
    accessory_mode_forced = true;
    logging.println("INFO: HVC backup holding ACCESSORY to keep the pack awake");
  }
  *hvc_backup = TeslaHvcBackup{};
  hvc_backup_ready = false;
  hvc_backup_refused = false;
  hvc_bk_reads_total = 0;
  hvc_bk_reads_ok = 0;
  hvc_bk_phase = HVC_BK_HVBMS;
  hvc_bk_step = 0;
  return true;
}

// Undo the ACCESSORY hold the backup took, if it took one. Called however the run ends.
void TeslaBattery::release_hvc_backup_hold() {
  if (!hvc_bk_accessory_held) {
    return;
  }
  hvc_bk_accessory_held = false;
  accessory_mode_forced = hvc_bk_saved_accessory_forced;
  balance_at_rest_enabled = hvc_bk_saved_balance_at_rest;
  logging.println("INFO: HVC backup released the ACCESSORY hold");
}

bool TeslaBattery::hvc_send_current() {
  if (hvc_bk_phase == HVC_BK_HVBMS_CAL) {
    uint8_t req[4] = {0x03, (uint8_t)(RT_HVBMS_CAL >> 8), (uint8_t)RT_HVBMS_CAL, HVBMS_CAL_IDS[hvc_bk_step]};
    return send_sequence_message(TESLA_UDS_HVC_HVBMS_CAL, SID::RoutineControl, req, 4,
                                 HVC_STEP_TIMEOUT_TICKS, HVC_STEP_RETRIES);
  }
  const HvcScalar& sc =
      (hvc_bk_phase == HVC_BK_HVBMS) ? HVC_SCALARS[hvc_bk_step] : HVC_SCALARS_HVP[hvc_bk_step];
  if (hvc_bk_phase == HVC_BK_HVP && sc.tag == TAG_HVP_CAL) {
    uint8_t rec = (uint8_t)(sc.did - HVP_CAL_DID_BASE);
    uint8_t req[4] = {0x03, (uint8_t)(RT_HVP_CAL >> 8), (uint8_t)RT_HVP_CAL, rec};
    return send_sequence_message(TESLA_UDS_HVC_CAL_RT, SID::RoutineControl, req, 4,
                                 HVC_STEP_TIMEOUT_TICKS, HVC_STEP_RETRIES);
  }
  uint16_t state = (hvc_bk_phase == HVC_BK_HVBMS) ? TESLA_UDS_HVC_HVBMS : TESLA_UDS_HVC_HVP;
  uint8_t req[2] = {(uint8_t)(sc.did >> 8), (uint8_t)sc.did};
  return send_sequence_message(state, SID::ReadDataByIdentifier, req, 2, HVC_STEP_TIMEOUT_TICKS,
                               HVC_STEP_RETRIES);
}

void TeslaBattery::hvc_handle_response(uint16_t state, uint8_t sid, const uint8_t* data, uint16_t len) {
  if (!hvc_backup || hvc_bk_phase == HVC_BK_IDLE) {
    return;
  }
  const bool negative = (sid == kNegativeResponseSid);

  // HVBMS calibration record: 71 03 04 39 <uint32>. No DID exists, so a refusal is final.
  if (state == TESLA_UDS_HVC_HVBMS_CAL) {
    bool stored = false;
    if (data != nullptr && !negative && len >= 8 && data[0] == 0x71) {
      hvc_backup->hvbms_cal[hvc_bk_step] = rd_be32(data + 4);
      hvc_backup->hvbms_cal_present[hvc_bk_step] = true;
      stored = true;
    } else if (negative) {
      hvc_backup->record_nrc(RT_HVBMS_CAL, len >= 3 ? data[2] : 0);
    }
    hvc_score_and_advance(stored);
    return;
  }

  bool stored = false;
  if (data != nullptr && !negative) {
    stored = hvc_store_scalar(data, len, state == TESLA_UDS_HVC_CAL_RT);
  } else if (negative) {
    const HvcScalar& sc =
        (state == TESLA_UDS_HVC_HVBMS) ? HVC_SCALARS[hvc_bk_step] : HVC_SCALARS_HVP[hvc_bk_step];
    hvc_backup->record_nrc(state == TESLA_UDS_HVC_CAL_RT ? RT_HVP_CAL : sc.did,
                           len >= 3 ? data[2] : 0);
  }
  // An HVP calibration record whose routine read failed gets one more try as the 0x0500+id DID
  // before it is scored - the fallback is a second attempt at the same record, not a separate read.
  if (!stored && state == TESLA_UDS_HVC_CAL_RT) {
    const HvcScalar& sc = HVC_SCALARS_HVP[hvc_bk_step];
    uint8_t req[2] = {(uint8_t)(sc.did >> 8), (uint8_t)sc.did};
    if (send_sequence_message(TESLA_UDS_HVC_HVP, SID::ReadDataByIdentifier, req, 2,
                              HVC_STEP_TIMEOUT_TICKS, HVC_STEP_RETRIES)) {
      return;
    }
  }
  hvc_score_and_advance(stored);
}

bool TeslaBattery::hvc_store_scalar(const uint8_t* rx, uint16_t len, bool via_routine) {
  TeslaHvcBackup& b = *hvc_backup;
  const HvcScalar& sc =
      (hvc_bk_phase == HVC_BK_HVBMS) ? HVC_SCALARS[hvc_bk_step] : HVC_SCALARS_HVP[hvc_bk_step];
  const uint8_t* data = nullptr;
  uint16_t dlen = 0;
  if (!via_routine && len >= 3 && rx[0] == 0x62) {  // 62 did_hi did_lo <data>
    data = rx + 3;
    dlen = (uint16_t)(len - 3);
  } else if (via_routine && len >= 4 && rx[0] == 0x71) {  // 71 sub rid_hi rid_lo <data>
    data = rx + 4;
    dlen = (uint16_t)(len - 4);
  } else {
    return false;
  }
  if (dlen < sc.min_len) {
    if (!b.short_read_count) {
      b.short_read_did = sc.did;
      b.short_read_len = (uint8_t)(dlen > 255 ? 255 : dlen);
    }
    ++b.short_read_count;
    return false;
  }
  auto copy_ascii = [&](char* dst, uint8_t cap) {
    uint16_t k = 0;
    for (; k < dlen && k < (uint16_t)(cap - 1) && data[k]; k++) {
      dst[k] = (char)data[k];
    }
    dst[k] = 0;
  };
  switch (sc.tag) {
    case TAG_ENERGY: {
      uint8_t n_ec = (uint8_t)(dlen < 32 ? dlen : 32);
      memset(b.energy_counters, 0, sizeof(b.energy_counters));  // a 28-byte reply leaves no tail
      memcpy(b.energy_counters, data, n_ec);
      b.energy_counters_len = n_ec;
      b.energy_counters_present = true;
      break;
    }
    case TAG_PACK_INFO:
      memcpy(b.pack_info, data, dlen < 16 ? dlen : 16);
      b.pack_info_present = true;
      b.num_modules = data[1];
      b.num_bricks = data[3];
      break;
    case TAG_HIST_COUNT:
      memcpy(b.hist_page_count, data, dlen < 8 ? dlen : 8);
      b.hist_page_count_present = true;
      break;
    case TAG_ASM_HVBMS:
      b.assembly_id_hvbms = data[0];
      b.assembly_id_hvbms_present = true;
      break;
    case TAG_USE_HVBMS:
      b.usage_id_hvbms = rd_be16(data);
      b.usage_id_hvbms_present = true;
      break;
    case TAG_CFG_HVBMS:
      b.config_hvbms_len = (uint8_t)(dlen < 64 ? dlen : 64);
      memcpy(b.config_hvbms, data, b.config_hvbms_len);
      break;
    case TAG_PKG_PN:
      copy_ascii(b.package_pn, sizeof(b.package_pn));
      b.package_pn_present = true;
      break;
    case TAG_PKG_SN:
      copy_ascii(b.package_sn, sizeof(b.package_sn));
      b.package_sn_present = true;
      break;
    case TAG_BOARD_PN:
      copy_ascii(b.board_pn, sizeof(b.board_pn));
      b.board_pn_present = true;
      break;
    case TAG_BOARD_SN:
      copy_ascii(b.board_sn, sizeof(b.board_sn));
      b.board_sn_present = true;
      break;
    case TAG_ASM_HVP:
      b.assembly_id_hvp = data[0];
      b.assembly_id_hvp_present = true;
      break;
    case TAG_USE_HVP:
      b.usage_id_hvp = rd_be16(data);
      b.usage_id_hvp_present = true;
      break;
    case TAG_CFG_HVP:
      b.config_hvp_len = (uint8_t)(dlen < 64 ? dlen : 64);
      memcpy(b.config_hvp, data, b.config_hvp_len);
      break;
    case TAG_HVP_CAL: {
      uint8_t idx = (uint8_t)(HVC_SCALARS_HVP[hvc_bk_step].did - HVP_CAL_DID_BASE);
      if (idx >= TESLA_HVP_NUM_CAL) {
        return false;
      }
      b.hvp_cal[idx] = rd_be32(data);
      b.hvp_cal_present[idx] = true;
      b.hvp_cal_from_did[idx] = !via_routine;
      break;
    }
  }
  return true;
}

void TeslaBattery::hvc_score_and_advance(bool stored) {
  ++hvc_bk_reads_total;
  if (stored) {
    ++hvc_bk_reads_ok;
  }
  ++hvc_bk_step;
  if (hvc_bk_phase == HVC_BK_HVBMS && hvc_bk_step >= N_HVC_SCALARS) {
    hvc_bk_phase = HVC_BK_HVBMS_CAL;
    hvc_bk_step = 0;
  } else if (hvc_bk_phase == HVC_BK_HVBMS_CAL && hvc_bk_step >= TESLA_HVBMS_NUM_CAL) {
    hvc_bk_phase = HVC_BK_HVP;
    hvc_bk_step = 0;
    retarget_uds(HVP_UDS_REQUEST, HVP_UDS_RESPONSE);
  } else if (hvc_bk_phase == HVC_BK_HVP && hvc_bk_step >= N_HVC_SCALARS_HVP) {
    finish_hvc_backup();
    return;
  }
  if (!hvc_send_current()) {
    finish_hvc_backup();  // channel taken away mid-run; keep what was read
  }
}

void TeslaBattery::finish_hvc_backup() {
  retarget_uds(BMS_UDS_REQUEST, BMS_UDS_RESPONSE);
  hvc_bk_phase = HVC_BK_IDLE;
  hvc_bk_queued = false;
  hvc_backup_ready = true;
  ++hvc_bk_runs;
  release_hvc_backup_hold();
  logging.printf("INFO: HVC backup complete, %u of %u reads answered\n", (unsigned)hvc_bk_reads_ok,
                 (unsigned)hvc_bk_reads_total);
}

String TeslaBattery::hvc_backup_json() {
  if (hvc_backup && hvc_backup_ready) {
    return build_hvc_backup_json(*hvc_backup);
  }
  return String("{\"error\": \"no backup available\"}");
}

// The pack serial the backup itself carries (0xF015). Restricted to the characters a Tesla serial
// actually uses: this string arrives over CAN and ends up in a Content-Disposition header.
String TeslaBattery::hvc_backup_name() {
  if (!hvc_backup || !hvc_backup_ready || !hvc_backup->package_sn_present) {
    return String("");
  }
  char name[sizeof(hvc_backup->package_sn)] = {0};
  uint8_t k = 0;
  for (const char* p = hvc_backup->package_sn; *p && k < (uint8_t)(sizeof(name) - 1); p++) {
    char c = *p;
    bool safe = (c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z') || (c >= '0' && c <= '9') || c == '-' || c == '_';
    if (safe) {
      name[k++] = c;
    }
  }
  return String(name);
}

String TeslaBattery::hvc_backup_status_json() {
  const char* state = "idle";
  const char* phase = "";
  if (hvc_bk_phase != HVC_BK_IDLE) {
    state = "running";
    phase = (hvc_bk_phase == HVC_BK_HVBMS)       ? "HVBMS identity and counters"
            : (hvc_bk_phase == HVC_BK_HVBMS_CAL) ? "HVBMS calibration"
                                                 : "HVP identity and calibration";
  } else if (hvc_backup_refused) {
    state = "refused";
    phase = "Pack not readable - check it is on CAN and the emulator is not faulted or stopped";
  } else if (hvc_backup_ready) {
    state = "ready";
  }
  String json = "{\"state\":\"";
  json += state;
  json += "\",\"phase\":\"";
  json += phase;
  json += "\",\"ok\":";
  json += String(hvc_bk_reads_ok);
  json += ",\"reads\":";
  json += String(hvc_bk_reads_total);
  json += ",\"run\":";
  json += String(hvc_bk_runs);
  json += ",\"note\":\"\"}";
  return json;
}


void TeslaBattery::setup(void) {  // Performs one time setup at startup

  setup_uds(0x602, 0x612);

  if (allows_contactor_closing) {
    *allows_contactor_closing = true;
  }

  //0x7FF GTW CAN frame values
  //Mux1 (country, RHD) is now fixed.
  //Mux2: GTW_numberHVILNodes, VAL_ 2047 0 "HVIL_NODES_0" 2 "HVIL_NODES_2" 3 "HVIL_NODES_3" (never 1).
  //0 is different: user_selected_tesla_digital_HVIL
  const bool tesla_3Y_long_range = (user_selected_battery_type == BatteryType::TeslaModel3Y) &&
                                   (user_selected_tesla_GTW_packEnergy == TESLA_GTW_PACK_ENERGY_LR);
  write_signal_value(&TESLA_7FF_Mux2, 62, 2, tesla_3Y_long_range ? 3 : 2, false);
  //Mux3: mapRegion is fixed; chassisType follows the selected battery:
  //2 = Model 3 for 3/Y packs, 0 = Model S for S/X packs. The BMS does not mind which exact
  //car the pack came from, so the per-family value is enough.
  const uint16_t tesla_GTW_chassisType = (user_selected_battery_type == BatteryType::TeslaModel3Y) ? 2 : 0;
  write_signal_value(&TESLA_7FF_Mux3, 18, 3, tesla_GTW_chassisType, false);
  write_signal_value(&TESLA_7FF_Mux3, 32, 5, user_selected_tesla_GTW_packEnergy, false);

  switch (
      user_selected_tesla_GTW_packEnergy) {  //static const std::map<int, String> tesla_pack = {{0, "50 kWh"}, {2, "62 kWh"}, {1, "74 kWh"}, {3, "100 kWh"}};
    case 0:
      datalayer_battery->info.total_capacity_Wh = 50000;
      break;
    case 1:
      datalayer_battery->info.total_capacity_Wh = 74000;
      break;
    case 2:
      datalayer_battery->info.total_capacity_Wh = 62000;
      break;
    case 3:
      datalayer_battery->info.total_capacity_Wh = 100000;
      break;
    default:
      break;
  }

  //IF 3 / Y
  if (user_selected_battery_type == BatteryType::TeslaModel3Y) {
    strncpy(datalayer.system.info.battery_protocol, Name3Y, 63);
    if (datalayer_battery->info.chemistry == battery_chemistry_enum::LFP) {
      datalayer_battery->info.max_design_voltage_dV = MAX_PACK_VOLTAGE_3Y_LFP;
      datalayer_battery->info.min_design_voltage_dV = MIN_PACK_VOLTAGE_3Y_LFP;
      datalayer_battery->info.max_cell_voltage_mV = MAX_CELL_VOLTAGE_LFP;
      datalayer_battery->info.min_cell_voltage_mV = MIN_CELL_VOLTAGE_LFP;
      datalayer_battery->info.max_cell_voltage_deviation_mV = MAX_CELL_DEVIATION_LFP;
    } else {
      datalayer_battery->info.max_design_voltage_dV = MAX_PACK_VOLTAGE_3Y_NCMA;
      datalayer_battery->info.min_design_voltage_dV = MIN_PACK_VOLTAGE_3Y_NCMA;
      datalayer_battery->info.max_cell_voltage_mV = MAX_CELL_VOLTAGE_NCA_NCM;
      datalayer_battery->info.min_cell_voltage_mV = MIN_CELL_VOLTAGE_NCA_NCM;
      datalayer_battery->info.max_cell_voltage_deviation_mV = MAX_CELL_DEVIATION_NCA_NCM;
    }
  } else {  //S/X

    strncpy(datalayer.system.info.battery_protocol, NameSX, 63);
    datalayer.system.info.battery_protocol[63] = '\0';
    datalayer_battery->info.max_design_voltage_dV = MAX_PACK_VOLTAGE_SX_NCMA;
    datalayer_battery->info.min_design_voltage_dV = MIN_PACK_VOLTAGE_SX_NCMA;
    datalayer_battery->info.max_cell_voltage_mV = MAX_CELL_VOLTAGE_NCA_NCM;
    datalayer_battery->info.min_cell_voltage_mV = MIN_CELL_VOLTAGE_NCA_NCM;
    datalayer_battery->info.max_cell_voltage_deviation_mV = MAX_CELL_DEVIATION_NCA_NCM;
  }
}

// HVC backup JSON builder
// Reproduces Tesla/ODIN's HVC_backup.json (field order, names, encodings). The statistics a
// partial backup does not read (module life, brick SOC/CAC/bleed, histograms) render as MISSING
// so a restore tool still finds every block it looks up by name.
namespace {

uint32_t hvc_be32(const uint8_t* p) {
  return ((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) | ((uint32_t)p[2] << 8) | p[3];
}

uint16_t hvc_be16(const uint8_t* p) {
  return (uint16_t)(((uint16_t)p[0] << 8) | p[1]);
}

// "<int>.<frac>" with trailing zeros trimmed, like Python's repr of raw*10^-places.
String hvc_scaled(uint32_t whole, uint16_t frac, int places) {
  char buf[32];
  int n = snprintf(buf, sizeof(buf), "%u", (unsigned)whole);
  if (frac) {
    char fb[8];
    if (places == 3) {
      snprintf(fb, sizeof(fb), "%03u", (unsigned)frac);
    } else {
      snprintf(fb, sizeof(fb), "%02u", (unsigned)frac);
    }
    int fl = (int)strlen(fb);
    while (fl > 1 && fb[fl - 1] == '0') {
      fb[--fl] = '\0';
    }
    n += snprintf(buf + n, sizeof(buf) - n, ".%s", fb);
  }
  return String(buf);
}
String hvc_kwh(uint32_t raw) {  // ODIN emits kWh as raw*0.001
  return hvc_scaled(raw / 1000, (uint16_t)(raw % 1000), 3);
}
String hvc_hexstr(const uint8_t* p, uint16_t n) {
  String s;
  s.reserve(n * 2);
  static const char* H = "0123456789abcdef";
  char pair[3] = {0, 0, 0};
  for (uint16_t i = 0; i < n; i++) {
    pair[0] = H[p[i] >> 4];
    pair[1] = H[p[i] & 0x0F];
    s += pair;
  }
  return s;
}
// One table row: ["Name", value]. jsonValue is emitted verbatim (number, or quoted string).
void hvc_row(String& out, bool& first, const char* name, const String& jsonValue) {
  if (!first) {
    out += ",\n";
  }
  first = false;
  out += "          [\n            \"";
  out += name;
  out += "\",\n            ";
  out += jsonValue;
  out += "\n          ]";
}
String hvc_q(const String& s) {
  return "\"" + s + "\"";
}
String hvc_q(const char* s) {
  return String("\"") + s + "\"";
}

}  // namespace

String build_hvc_backup_json(const TeslaHvcBackup& b) {
  const char* MISSING = "\"MISSING\"";
  String out;
  out.reserve(8192);
  out += "{\n  \"data\": [\n";

  // ---- block 0: HVBMS Data table ----
  out += "    {\n      \"data\": {\n        \"headers\": [\n          \"Name\",\n          \"Data\"\n"
         "        ],\n        \"rows\": [\n";
  bool first = true;
  const uint8_t* e = b.energy_counters;
  bool ec = b.energy_counters_present;
  hvc_row(out, first, "Discharge Total kWh", ec ? hvc_kwh(hvc_be32(e + 0)) : String(MISSING));
  hvc_row(out, first, "Drive Discharge kWh", ec ? hvc_kwh(hvc_be32(e + 4)) : String(MISSING));
  hvc_row(out, first, "Charge Total kWh", ec ? hvc_kwh(hvc_be32(e + 8)) : String(MISSING));
  hvc_row(out, first, "AC Charge kWh", ec ? hvc_kwh(hvc_be32(e + 12)) : String(MISSING));
  hvc_row(out, first, "DC Charge kWh", ec ? hvc_kwh(hvc_be32(e + 16)) : String(MISSING));
  hvc_row(out, first, "Regen Charge kWh", ec ? hvc_kwh(hvc_be32(e + 20)) : String(MISSING));
  // DISCHARGE_AH / CHARGE_AH: the ODX types both as uint32 at bytes 24/28, but some packs send two
  // uint16 at 24/26 and end the response there. A response under 32 bytes is the narrow layout;
  // the value test catches a full-length response that is nonetheless packed narrow.
  bool narrow = b.energy_counters_len && b.energy_counters_len < 32;
  uint32_t dis_ah = hvc_be32(e + 24), chg_ah = hvc_be32(e + 28);
  if (narrow || ((dis_ah >> 16) != 0 && chg_ah == 0)) {
    dis_ah = (uint32_t)hvc_be16(e + 24);
    chg_ah = (uint32_t)hvc_be16(e + 26);
  }
  hvc_row(out, first, "Discharge Ah", ec ? String(dis_ah) : String(MISSING));
  hvc_row(out, first, "Charge Ah", ec ? String(chg_ah) : String(MISSING));

  // Module life data is not read by a partial backup.
  for (uint8_t m = 0; m < b.num_modules && m < TESLA_HVC_MAX_MODULES; m++) {
    String p = "Module " + String(m + 1) + " ";
    hvc_row(out, first, (p + "Birth Epoch Time").c_str(), String(MISSING));
    hvc_row(out, first, (p + "Discharge Total kWh").c_str(), String(MISSING));
    hvc_row(out, first, (p + "Charge Total kWh").c_str(), String(MISSING));
    hvc_row(out, first, (p + "AC Charge kWh").c_str(), String(MISSING));
    hvc_row(out, first, (p + "DC Charge kWh").c_str(), String(MISSING));
  }

  hvc_row(out, first, "Assembly ID", b.assembly_id_hvbms_present ? String(b.assembly_id_hvbms) : String(MISSING));
  hvc_row(out, first, "Usage ID", b.usage_id_hvbms_present ? String(b.usage_id_hvbms) : String(MISSING));
  hvc_row(out, first, "Configuration Record",
          b.config_hvbms_len ? hvc_q(hvc_hexstr(b.config_hvbms, b.config_hvbms_len)) : String(MISSING));
  // HVBMS calibration records 0/1/6, ODIN's required-for-recovery set (routine 0x439).
  {
    static const uint8_t ids[TESLA_HVBMS_NUM_CAL] = {0, 1, 6};
    for (uint8_t i = 0; i < TESLA_HVBMS_NUM_CAL; i++) {
      String nm = "Calibration Record " + String(ids[i]);
      hvc_row(out, first, nm.c_str(), b.hvbms_cal_present[i] ? String(b.hvbms_cal[i]) : String(MISSING));
    }
  }
  hvc_row(out, first, "Package Part Number", b.package_pn_present ? hvc_q(b.package_pn) : String(MISSING));
  hvc_row(out, first, "Package Serial Number", b.package_sn_present ? hvc_q(b.package_sn) : String(MISSING));
  out += "\n        ]\n      },\n      \"data_type\": \"Table\",\n      \"name\": \"HVBMS Data\",\n"
         "      \"units\": null\n    },\n";

  // ---- block 1: HVP Data table ----
  out += "    {\n      \"data\": {\n        \"headers\": [\n          \"Name\",\n          \"Data\"\n"
         "        ],\n        \"rows\": [\n";
  first = true;
  hvc_row(out, first, "Assembly ID", b.assembly_id_hvp_present ? String(b.assembly_id_hvp) : String(MISSING));
  hvc_row(out, first, "Usage ID", b.usage_id_hvp_present ? String(b.usage_id_hvp) : String(MISSING));
  hvc_row(out, first, "Configuration Record",
          b.config_hvp_len ? hvc_q(hvc_hexstr(b.config_hvp, b.config_hvp_len)) : String(MISSING));
  // Calibration records in ODIN's row order (0x0501..0x0507, then the gain record its table omits).
  // An erased record reads 0xFFFFFFFF - that is the pack's content, not a failed read.
  static const char* const HVP_CAL_NAMES[TESLA_HVP_NUM_CAL] = {
      "Shunt ASIC Gain",
      "Shunt Bar Resistance",
      "Shunt Thermal Coefficient Resistance A",
      "Shunt Thermal Coefficient Resistance B",
      "Shunt Thermal Coefficient Resistance C",
      "Shunt Thermal Coefficient Resistance D",
      "BMB ASIC Type",
      "HVIL Configuration",
  };
  for (uint8_t i = 1; i < TESLA_HVP_NUM_CAL; i++) {
    hvc_row(out, first, HVP_CAL_NAMES[i], b.hvp_cal_present[i] ? String(b.hvp_cal[i]) : String(MISSING));
  }
  hvc_row(out, first, HVP_CAL_NAMES[0], b.hvp_cal_present[0] ? String(b.hvp_cal[0]) : String(MISSING));
  out += "\n        ]\n      },\n      \"data_type\": \"Table\",\n      \"name\": \"HVP Data\",\n"
         "      \"units\": null\n    },\n";

  // ---- blocks 2-4: Readable Time, Board Serial / Part Number ----
  out += "    {\n      \"data\": \"";
  out += (b.readable_time[0] ? b.readable_time : "Not Available");
  out += "\",\n      \"data_type\": \"<class 'str'>\",\n      \"name\": \"Readable Time\",\n      \"units\": null\n    },\n";
  out += "    {\n      \"data\": ";
  out += b.board_sn_present ? hvc_q(b.board_sn) : String(MISSING);
  out += ",\n      \"data_type\": \"<class 'str'>\",\n      \"name\": \"Board Serial Number\",\n      \"units\": null\n    },\n";
  out += "    {\n      \"data\": ";
  out += b.board_pn_present ? hvc_q(b.board_pn) : String(MISSING);
  out += ",\n      \"data_type\": \"<class 'str'>\",\n      \"name\": \"Board Part Number\",\n      \"units\": null\n    },\n";

  // ---- block 5: Pack Info. Brick Group Counts use ODIN's limits: SOC 96, CAC 96, BLEED 48.
  auto groups = [](uint8_t bricks, uint8_t limit) -> uint8_t {
    if (!bricks || !limit) return 0;
    return (bricks + limit - 1) / limit;
  };
  out += "    {\n      \"data\": {\n";
  out += "        \"Brick Group Counts\": {\n          \"Bleed Time\": " + String(groups(b.num_bricks, 48)) +
         ",\n          \"CAC\": " + String(groups(b.num_bricks, 96)) +
         ",\n          \"SOC\": " + String(groups(b.num_bricks, 96)) + "\n        },\n";
  out += "        \"Histogram Page Counts\": {\n";
  for (uint8_t h = 0; h < TESLA_HVC_NUM_HISTOGRAMS; h++) {
    out += "          \"Histogram ID " + String(h + 1) + "\": " +
           String(b.hist_page_count_present ? b.hist_page_count[h] : 0);
    out += (h < TESLA_HVC_NUM_HISTOGRAMS - 1) ? ",\n" : "\n";
  }
  out += "        },\n";
  out += "        \"Number Of Bricks\": " + String(b.num_bricks) + ",\n";
  out += "        \"Number Of Modules\": " + String(b.num_modules) + "\n";
  out += "      },\n      \"data_type\": \"<class 'dict'>\",\n      \"name\": \"Pack Info\",\n      \"units\": null\n    },\n";

  // ---- blocks 6/7: Recovery Type / Version. PARTIAL tells a restore to recalibrate the skipped data.
  out += "    {\n      \"data\": \"PARTIAL\",\n      \"data_type\": \"<class 'str'>\",\n      \"name\": \"Recovery Type\",\n      \"units\": null\n    },\n";
  out += "    {\n      \"data\": \"1.0.1\",\n      \"data_type\": \"<class 'str'>\",\n      \"name\": \"Version\",\n      \"units\": null\n    },\n";

  // ---- blocks 8-10: per-brick statistics, not read by a partial backup ----
  auto missing_block = [&](const char* name) {
    out += "    {\n      \"data\": \"MISSING\",\n      \"data_type\": \"<class 'str'>\",\n      \"name\": \"";
    out += name;
    out += "\",\n      \"units\": null\n    },\n";
  };
  missing_block("Brick SOC Data");
  missing_block("Brick CAC Data");
  missing_block("Brick Bleed Time Data");

  // ---- block 11: read diagnostics (ours, appended after every block ODIN emits)
  auto hex4 = [](uint16_t v) {
    char buf[7];
    snprintf(buf, sizeof(buf), "0x%04X", v);
    return String(buf);
  };
  auto hex2 = [](uint8_t v) {
    char buf[5];
    snprintf(buf, sizeof(buf), "0x%02X", v);
    return String(buf);
  };
  out += "    {\n      \"data\": {\n";
  out += "        \"Energy Counters Bytes\": " + String(b.energy_counters_len) + ",\n";
  out += "        \"Reads Refused\": " + String(b.nrc_count) + ",\n";
  out += "        \"Negative Responses\": \"";
  if (!b.nrc_records) {
    out += "none";
  } else {
    for (uint8_t i = 0; i < b.nrc_records; i++) {
      if (i) out += ", ";
      out += hex4(b.nrc_did[i]) + " NRC " + hex2(b.nrc_code[i]);
    }
  }
  out += "\",\n";
  out += "        \"Calibration From DID Fallback\": \"";
  {
    bool any = false;
    for (uint8_t i = 0; i < TESLA_HVP_NUM_CAL; i++) {
      if (b.hvp_cal_present[i] && b.hvp_cal_from_did[i]) {
        if (any) out += ", ";
        out += hex4((uint16_t)(HVP_CAL_DID_BASE + i));
        any = true;
      }
    }
    if (!any) out += "none";
  }
  out += "\",\n";
  out += "        \"Short Reads\": " + String(b.short_read_count) + ",\n";
  out += "        \"First Short Read\": \"";
  out += b.short_read_count ? (hex4(b.short_read_did) + " " + String(b.short_read_len) + " bytes") : String("none");
  out += "\"\n";
  out += "      },\n      \"data_type\": \"<class 'dict'>\",\n      \"name\": \"Read Diagnostics\",\n"
         "      \"units\": null\n    }\n";

  out += "  ],\n  \"exit_code\": 0,\n  \"exit_reason\": null,\n"
         "  \"user_facing_msg\": \"Successfully read HVC backup data from Car Computer.\"\n}\n";
  return out;
}
