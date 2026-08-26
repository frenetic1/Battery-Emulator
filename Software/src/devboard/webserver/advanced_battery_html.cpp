#include "advanced_battery_html.h"
#include <Arduino.h>
#include <vector>
#include "../../battery/BATTERIES.h"
#include "../../datalayer/datalayer.h"
#include "../../datalayer/datalayer_extended.h"

// Progress readout for the "Create HVC Backup" command. The command itself is fire-and-forget over PUT
// and the read then takes several seconds of UDS traffic, so the page polls /hvc_backup_status and
// starts the download itself once the backup lands. Without this the button gives no feedback at all
// and the finished JSON is only reachable by typing the URL by hand.
static String hvc_backup_status_html(int ix) {
  String n = String(ix);
  String id = "hvcBk" + n;
  String status_url = "/hvc_backup_status?battery=" + n;
  String content = "<div id='" + id + "' style='margin: 5px 5px 10px 15px; font-size: 0.9em;'></div>";
  content += "<script>(function(){";
  // Arm only one poller per battery - two would each see the running->ready edge and download twice.
  content += "if (window['hvcBkArmed" + n + "']) { return; } window['hvcBkArmed" + n + "'] = 1;";
  content += "var el = document.getElementById('" + id + "');";
  content += "var url = '/hvc_backup.json?battery=" + n + "';";
  content += "var baseRun = -1;";
  content += "function draw(s) {";
  content += "  if (s.state == 'running') { el.innerHTML = 'Reading backup: ' + s.phase + ' (' + s.reads + ' reads)'; }";
  content +=
      "  else if (s.state == 'ready') { el.innerHTML = 'Backup ready (' + s.ok + '/' + s.reads + ' reads answered)'"
      " + (s.note ? \"<br><span style='color: #FC6;'>\" + s.note + '</span>' : ''); }";
  content +=
      "  else if (s.state == 'refused') { el.innerHTML = 'Backup refused: ' + s.phase + '.'; }";
  content += "  else { el.innerHTML = ''; }";
  // Download when the completed-run counter advances past what this page saw first. Watching for a
  // running->ready transition missed fast runs entirely (a partial read finishes between two
  // polls), while the counter also keeps an old backup in memory from downloading on page load.
  content += "  var run = s.run || 0;";
  content += "  if (baseRun < 0) { baseRun = run; return; }";
  content += "  if (s.state == 'ready' && run > baseRun) { baseRun = run; window.location.href = url; }";
  content += "}";
  content += "function poll() {";
  content += "  var xhr = new XMLHttpRequest();";
  content += "  xhr.open('GET', '" + status_url + "', true);";
  content += "  xhr.onload = function() { try { draw(JSON.parse(xhr.responseText)); } catch (e) {} };";
  content += "  xhr.send();";
  content += "}";
  content += "poll(); setInterval(poll, 1500);";
  content += "})();</script>";
  return content;
}

// Appended to the confirm popup of commands the selected battery runs under a contactor guard.
// Batteries whose implementation does not move contactors return nullptr and get no warning.
static const char* contactor_warning(Battery* b) {
  return b ? b->contactor_open_warning() : nullptr;
}

// Available generic battery commands that are taken into use based on what the selected battery supports.
std::vector<BatteryCommand> battery_commands = {
    {"clearIsolation", "Clear Isolation Condition", "clear any active isolation fault?",
     [](Battery* b) { return b && b->supports_clear_isolation(); }, [](Battery* b) { b->clear_isolation(); }},
    {"calibrateSOC", "Calibrate SOC", "calibrate SOC? Note this will calibrate BMS according to set targets",
     [](Battery* b) { return b && b->supports_calibrate_SOC(); }, [](Battery* b) { b->reset_SOC(); }},
    {"chademoRestart", "Restart", "restart the V2X session?",
     [](Battery* b) { return b && b->supports_chademo_restart(); }, [](Battery* b) { b->chademo_restart(); }},
    {"chademoStop", "Stop", "stop V2X?", [](Battery* b) { return b && b->supports_chademo_restart(); },
     [](Battery* b) { b->chademo_restart(); }},
    {"resetBMS", "BMS Reset", "Reset the BMS?", [](Battery* b) { return b && b->supports_reset_BMS(); },
     [](Battery* b) { b->reset_BMS(); }, false, nullptr, contactor_warning},
    {"resetHVP", "HVP/HVC Reset",
     "Reboot the HVP/HVC via UDS ECUReset? Requires the pack contactors to be OPEN first. Same reset the "
     "software update performs after flashing.",
     [](Battery* b) { return b && b->supports_reset_HVP(); }, [](Battery* b) { b->reset_HVP(); }, false, nullptr,
     contactor_warning},
    {"resetPCS", "PCS Reset", "Reboot the PCS via UDS ECUReset? Requires the pack contactors to be OPEN.",
     [](Battery* b) { return b && b->supports_reset_PCS(); }, [](Battery* b) { b->reset_PCS(); }, false, nullptr,
     contactor_warning},
    {"clearBlockedContactors", "Clear Blocked Contactors",
     "Attempt to clear blocked pack contactors? Runs this sequence over UDS: clear the VSH latch, "
     "clear the isolation condition, then reset the HVBMS and HVP. Requires the pack contactors to be OPEN. "
     "Even if every step passes, contactors may still be blocked by other conditions.",
     [](Battery* b) { return b && b->supports_clear_blocked_contactors(); },
     [](Battery* b) { b->clear_blocked_contactors(); }, false, nullptr, contactor_warning},
    {"hvcBackupPartial", "Create HVC Backup (Partial)",
     "Read a partial HVC/HVBMS backup over UDS - the fields a restore needs (identity, configuration, "
     "calibration, energy counters), marked RECOVERY_TYPE PARTIAL so a restore recalibrates the "
     "skipped statistics? Progress appears below the button and the JSON downloads automatically "
     "when it finishes.",
     [](Battery* b) { return b && b->supports_hvc_backup(); }, [](Battery* b) { b->request_hvc_backup_partial(); },
     false, hvc_backup_status_html},
    {"resetSOC", "SOC Reset", "Reset SOC?", [](Battery* b) { return b && b->supports_reset_SOC(); },
     [](Battery* b) { b->reset_SOC(); }, false, nullptr, contactor_warning},
    //CAC Reset button hidden for now; reset_CAC()/supports_reset_CAC() and the UDS sequence remain
    //in place - uncomment to expose it again.
    //{"resetCAC", "CAC Reset",
    // "Reset brick CAC? Tesla warning: This can cause premature Graceful Power Off - do not go below 20% SOC for 8 weeks.",
    // [](Battery* b) { return b && b->supports_reset_CAC(); }, [](Battery* b) { b->reset_CAC(); }, false, nullptr,
    // contactor_warning},
    {"disableIsoMon", "Disable Isolation Monitor",
     "Disable the BMS isolation monitor? The BMS restores it on "
     "power cycle.",
     [](Battery* b) { return b && b->supports_disable_isolation_monitor(); },
     [](Battery* b) { b->disable_isolation_monitor(); }},
    {"enableIsoMon", "Enable Isolation Monitor", "Re-enable the BMS isolation monitor?",
     [](Battery* b) { return b && b->supports_disable_isolation_monitor(); },
     [](Battery* b) { b->enable_isolation_monitor(); }},
    {"vehicleDrive", "DRIVE", "Set vehicle state to DRIVE and remove the charge/discharge limit?",
     [](Battery* b) { return b && b->supports_set_vehicle_mode(); }, [](Battery* b) { b->set_vehicle_mode_drive(); }},
    {"vehicleAccessory", "ACCESSORY", "Set vehicle state to ACCESSORY and allow no charge or discharge (0 W)?",
     [](Battery* b) { return b && b->supports_set_vehicle_mode(); },
     [](Battery* b) { b->set_vehicle_mode_accessory(); }},
    {"balanceAtRest", "Toggle Balance At Rest (experimental)",
     "toggle experimental auto mode: switches DRIVE/ACCESSORY by load so the pack can balance at "
     "rest?",
     [](Battery* b) { return b && b->supports_set_vehicle_mode(); },
     [](Battery* b) { b->toggle_balance_at_rest(); }},
    {"resetCrash", "Unlock crashed BMS",
     "reset crash data? Note this will unlock your BMS and enable contactor closing and SOC calculation.",
     [](Battery* b) { return b && b->supports_reset_crash(); }, [](Battery* b) { b->reset_crash(); }},
    {"resetNVROL", "Perform NVROL reset",
     "trigger an NVROL reset? Battery will be unavailable for 30 seconds while this is active!",
     [](Battery* b) { return b && b->supports_reset_NVROL(); }, [](Battery* b) { b->reset_NVROL(); }},
    {"resetContactor", "Perform contactor reset", "reset contactors?",
     [](Battery* b) { return b && b->supports_contactor_reset(); }, [](Battery* b) { b->reset_contactor(); }},
    {"resetDTC", "Erase DTC", "erase DTCs?", [](Battery* b) { return b && b->supports_reset_DTC(); },
     [](Battery* b) { b->reset_DTC(); }, true},
    {"startBalancing", "Balancing",
     "continue? Please charge battery fully for this to work. After a couple of minutes, battery will sleep and do "
     "balancing. It often takes many hours. There will be no progress indication.",
     [](Battery* b) { return b && b->supports_balancing() && !b->is_balancing_active(); },
     [](Battery* b) { b->initiate_balancing(); }},
    {"endBalancing", "Stop Balancing Mode", "end offline balancing?",
     [](Battery* b) { return b && b->supports_balancing() && b->is_balancing_active(); },
     [](Battery* b) { b->end_balancing(); }},
    {"startBalancingRequest", "Start Balancing", "request the BMS to start cell balancing?",
     [](Battery* b) { return b && b->supports_balancing_request(); }, [](Battery* b) { b->initiate_balancing(); }},
    {"stopBalancingRequest", "Stop Balancing", "request the BMS to stop cell balancing?",
     [](Battery* b) { return b && b->supports_balancing_request(); }, [](Battery* b) { b->end_balancing(); }},
    {"isolationTest", "Isolation Test", "start an isolation test?",
     [](Battery* b) { return b && b->supports_isolation_test(); }, [](Battery* b) { b->request_isolation_test(); }},
    {"queryEcuVersions", "Query ECU Versions", "read HVBMS/HVP/PCS firmware version identifiers over UDS?",
     [](Battery* b) { return b && b->supports_query_ecu_versions(); }, [](Battery* b) { b->query_ecu_versions(); }},
    {"readDTC", "Read DTC", nullptr, [](Battery* b) { return b && b->supports_read_DTC(); },
     [](Battery* b) { b->read_DTC(); }, true},
    {"resetBECM", "Restart BECM module", "restart BECM??", [](Battery* b) { return b && b->supports_reset_BECM(); },
     [](Battery* b) { b->reset_BECM(); }},
    {"contactorClose", "Close Contactors", "a contactor close request?",
     [](Battery* b) { return b && b->supports_contactor_close(); }, [](Battery* b) { b->request_close_contactors(); }},
    {"contactorOpen", "Open Contactors", "a contactor open request?",
     [](Battery* b) { return b && b->supports_contactor_close(); }, [](Battery* b) { b->request_open_contactors(); }},
    {"resetSOH", "Reset degradation data", "reset degradation data?",
     [](Battery* b) { return b && b->supports_reset_SOH(); }, [](Battery* b) { b->reset_SOH(); }},
    {"setFactoryMode", "Set Factory Mode", "set factory mode and disable isolation measurement?",
     [](Battery* b) { return b && b->supports_factory_mode_method(); }, [](Battery* b) { b->set_factory_mode(); }},
    {"toggleSOC", "Toggle SOC method",
     "toggle SOC method? This will toggle between ESTIMATED and MEASURED SOC methods.",
     [](Battery* b) { return b && b->supports_toggle_SOC_method(); }, [](Battery* b) { b->toggle_SOC_method(); }},
    {"resetEnergySavingMode", "Reset Energy Saving Mode", "reset energy saving mode to normal?",
     [](Battery* b) { return b && b->supports_energy_saving_mode_reset(); },
     [](Battery* b) { b->reset_energy_saving_mode(); }},
};

String advanced_battery_processor(const String& var) {
  if (var == "X") {
    String content = "";
    //Page format
    content += "<style>";
    content += "body { background-color: black; color: white; }";
    content +=
        "button { background-color: #505E67; color: white; border: none; padding: 10px 20px; margin: 5px; "
        "cursor: pointer; border-radius: 10px; }";
    content += "button:hover { background-color: #3A4A52; }";
    content += "h4 { margin: 0.6em 0; line-height: 1.2; }";
    content += "</style>";
    content += "<button onclick='goToMainPage()'>Back to main page</button>";

    // Start a new block with a specific background color
    content += "<div style='background-color: #303E47; padding: 10px; margin-bottom: 10px;border-radius: 50px'>";

    // Render buttons dynamically based on what commands the battery supports.
    auto render_command_buttons = [&content](Battery* batt, int ix) {
      for (const auto& cmd : battery_commands) {
        if (cmd.condition(batt)) {
          // Button for user action
          content += "<button onclick='ask" + String(cmd.identifier) + "(" + String(ix) + ")'>" + String(cmd.title) +
                     "</button>";

          // Script that calls the backend to perform the command
          content += "<script>";
          content += "function ask" + String(cmd.identifier) + "(batteryNum) { ";

          if (cmd.prompt) {
            String prompt = String(cmd.prompt);
            if (cmd.prompt_extra) {
              const char* extra = cmd.prompt_extra(batt);
              if (extra) {
                prompt += " " + String(extra);
              }
            }
            content += "if (window.confirm('Are you sure you want to " + prompt + "'))";
          }

          content += "{" + String(cmd.identifier) + "(batteryNum); } }";
          content += "function " + String(cmd.identifier) + "(batteryNum) {";
          content += "  var xhr = new XMLHttpRequest();";
          content += "  xhr.open('PUT', '/" + String(cmd.identifier) + "', true);";
          if (cmd.reload_after) {
            content += "  xhr.onload = function(){ setTimeout(function(){ location.reload(); }, 1500); };";
          }
          // Send index of the battery as PUT content
          content += "  xhr.send(batteryNum);";
          content += "}";
          content += "</script>";

          if (cmd.extra_html) {
            content += cmd.extra_html(ix);
          }
        }
      }
    };

    // Renders battery specific status for battery 2/3. Only renderers that
    // read per-instance data (renders_own_battery_data) are used; legacy
    // renderers hardcode the main battery datalayer and would show battery 1
    // values here, so a notice is shown instead until they are reworked.
    auto render_secondary_battery_status = [&content](Battery* batt) {
      BatteryHtmlRenderer& renderer = batt->get_status_renderer();
      if (renderer.renders_own_battery_data()) {
        content += renderer.get_status_html();
      } else {
        content +=
            "<h4 style='color: #f39c12;'>⚠️ Advanced detailed info is currently limited to the Main Battery.</h4>";
      }
    };

    if (battery) {
      content += battery->get_status_renderer().get_status_html();
      render_command_buttons(battery, 0);
    }

    if (battery2) {
      content += "<hr>";
      content += "<h4>Values from battery 2</h4>";
      render_secondary_battery_status(battery2);
      render_command_buttons(battery2, 1);
    }

    if (battery3) {
      content += "<hr>";
      content += "<h4>Values from battery 3</h4>";
      render_secondary_battery_status(battery3);
      render_command_buttons(battery3, 2);
    }

    content += "</div>";

    content += "<script>";
    content += "function exportLog() { window.location.href = '/export_log'; }";
    content += "function goToMainPage() { window.location.href = '/'; }";
    content += "</script>";

    return content;
  }
  return String();
}
