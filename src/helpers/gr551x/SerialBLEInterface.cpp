#include "SerialBLEInterface.h"
#include <gr551x_ble.h>
#include <string.h>
#include <stdio.h>

extern "C" {
#include "ble_prf_types.h"
#include "ble_prf_utils.h"
}

#define ATT_128_PRIMARY_SERVICE BLE_ATT_16_TO_128_ARRAY(BLE_ATT_DECL_PRIMARY_SERVICE)
#define ATT_128_CHARACTERISTIC  BLE_ATT_16_TO_128_ARRAY(BLE_ATT_DECL_CHARACTERISTIC)
#define ATT_128_CLIENT_CHAR_CFG BLE_ATT_16_TO_128_ARRAY(BLE_ATT_DESC_CLIENT_CHAR_CFG)

// Nordic UART Service, little-endian 128-bit UUIDs
#define NUS_UUID(b) 0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0, 0x93, 0xF3, 0xA3, 0xB5, b, 0x00, 0x40, 0x6E
#define NUS_SERVICE_UUID  NUS_UUID(0x01)
#define NUS_RX_UUID       NUS_UUID(0x02)    // central -> peripheral (write)
#define NUS_TX_UUID       NUS_UUID(0x03)    // peripheral -> central (notify)

#define BLE_ADV_INTERVAL_MIN   32     // 20 ms (units: 0.625 ms)
#define BLE_ADV_INTERVAL_MAX   244    // 152.5 ms
#define BLE_HEALTH_CHECK_INTERVAL 10000

enum { NUS_IDX_SVC, NUS_IDX_RX_CHAR, NUS_IDX_RX_VAL, NUS_IDX_TX_CHAR, NUS_IDX_TX_VAL, NUS_IDX_TX_CFG, NUS_IDX_NB };

static const uint8_t s_svc_uuid[] = { NUS_SERVICE_UUID };
static const uint16_t s_char_mask = 0xFFFF;

// Attribute table, filled in onStackReady(): with a PIN, the NUS characteristics require an
// authenticated (MITM) encrypted link, so the central has to pair with the passkey first.
static ble_gatts_attm_desc_128_t s_attr_tab[NUS_IDX_NB];
static const uint8_t s_uuid_svc[16]  = ATT_128_PRIMARY_SERVICE;
static const uint8_t s_uuid_char[16] = ATT_128_CHARACTERISTIC;
static const uint8_t s_uuid_cccd[16] = ATT_128_CLIENT_CHAR_CFG;
static const uint8_t s_uuid_rx[16]   = { NUS_RX_UUID };
static const uint8_t s_uuid_tx[16]   = { NUS_TX_UUID };

static void setAttr(int idx, const uint8_t* uuid, uint16_t perm, uint16_t ext_perm, uint16_t max_size) {
  memcpy(s_attr_tab[idx].uuid, uuid, 16);
  s_attr_tab[idx].perm = perm;
  s_attr_tab[idx].ext_perm = ext_perm;
  s_attr_tab[idx].max_size = max_size;
}

static SerialBLEInterface* instance = nullptr;
static ble_gatts_create_db_t s_db;
static uint16_t s_start_hdl;
static volatile int s_conn_idx = -1;
static volatile uint16_t s_cccd;       // TX notifications enabled by the central
static volatile uint16_t s_mtu = 23;
static char s_dev_name[32 + 16];
static ble_gap_adv_param_t s_adv_param;
static ble_gap_adv_time_param_t s_adv_time;
static volatile bool s_advertising;
static uint32_t s_pin;                 // 0 = no pairing required
static volatile bool s_secured;

// Small event log (BLE events arrive in interrupt context), read with serialBLEDumpLog()
struct LogEntry { uint32_t ms; uint16_t evt; uint16_t status; uint8_t a; uint8_t b; };
#define LOG_SIZE 24
static LogEntry s_log[LOG_SIZE];
static volatile uint8_t s_log_pos, s_log_count;
static void logEvt(uint16_t evt, uint16_t status, uint8_t a = 0, uint8_t b = 0) {
  LogEntry& e = s_log[s_log_pos];
  e.ms = millis(); e.evt = evt; e.status = status; e.a = a; e.b = b;
  s_log_pos = (s_log_pos + 1) % LOG_SIZE;
  if (s_log_count < LOG_SIZE) s_log_count++;
}

void serialBLEDumpLog(Print& out) {
  out.printf("BLE pin=%s conn=%d cccd=%u secured=%d mtu=%u\r\n", s_pin ? "on" : "off", s_conn_idx, s_cccd,
             s_secured, s_mtu);
  uint8_t start = (s_log_pos + LOG_SIZE - s_log_count) % LOG_SIZE;
  for (uint8_t i = 0; i < s_log_count; i++) {
    const LogEntry& e = s_log[(start + i) % LOG_SIZE];
    out.printf("  %lu ms evt=0x%04x status=0x%02x %u %u\r\n", (unsigned long)e.ms, e.evt, e.status, e.a, e.b);
  }
}

static uint8_t attrIndex(uint16_t handle) {
  return prf_find_idx_by_handle(handle, s_start_hdl, NUS_IDX_NB, (uint8_t*)&s_char_mask);
}

static void prfEvent(const ble_evt_t* e) {
  switch (e->evt_id) {
    case BLE_GATTS_EVT_READ_REQUEST: {
      const ble_gatts_evt_read_t& rq = e->evt.gatts_evt.params.read_req;
      ble_gatts_read_cfm_t cfm;
      cfm.handle = rq.handle;
      if (attrIndex(rq.handle) == NUS_IDX_TX_CFG) {
        cfm.length = sizeof(uint16_t);
        cfm.value = (uint8_t*)&s_cccd;
        cfm.status = BLE_SUCCESS;
      } else {
        cfm.length = 0;
        cfm.status = BLE_ATT_ERR_INVALID_HANDLE;
      }
      ble_gatts_read_cfm(e->evt.gatts_evt.index, &cfm);
      break;
    }
    case BLE_GATTS_EVT_WRITE_REQUEST: {
      const ble_gatts_evt_write_t& wr = e->evt.gatts_evt.params.write_req;
      ble_gatts_write_cfm_t cfm;
      cfm.handle = wr.handle;
      cfm.status = BLE_SUCCESS;
      switch (attrIndex(wr.handle)) {
        case NUS_IDX_RX_VAL:
          if (instance) instance->onRx(wr.value, wr.length);
          break;
        case NUS_IDX_TX_CFG:
          s_cccd = wr.length >= 2 ? (uint16_t)(wr.value[0] | (wr.value[1] << 8)) : 0;
          BLE_DEBUG_PRINTLN("CCCD=%u", s_cccd);
          break;
        default:
          cfm.status = BLE_ATT_ERR_INVALID_HANDLE;
          break;
      }
      ble_gatts_write_cfm(e->evt.gatts_evt.index, &cfm);
      break;
    }
    case BLE_GATTS_EVT_NTF_IND:
      if (instance) instance->_txBusy = false;
      break;
    case BLE_GATTS_EVT_CCCD_RECOVERY:
      if (attrIndex(e->evt.gatts_evt.params.cccd_recovery.handle) == NUS_IDX_TX_CFG)
        s_cccd = e->evt.gatts_evt.params.cccd_recovery.cccd_val;
      break;
    default:
      break;
  }
}

static void advStart() {
  if (!instance || !instance->_isEnabled || s_conn_idx >= 0 || s_advertising || gr551x_ble_stopped()) return;
  if (ble_gap_adv_start(0, &s_adv_time) == SDK_SUCCESS) s_advertising = true;
}

static void onSecEvent(const ble_evt_t* e);

static void gapEvent(const ble_evt_t* e) {
  if (e->evt_id != BLE_GATTS_EVT_NTF_IND)
    logEvt(e->evt_id, e->evt_status, e->evt.gapc_evt.index,
           e->evt_id == BLE_GAPC_EVT_DISCONNECTED ? e->evt.gapc_evt.params.disconnected.reason : 0);
  switch (e->evt_id) {
    case BLE_GAPM_EVT_ADV_START:
      s_advertising = (e->evt_status == BLE_SUCCESS);
      break;
    case BLE_GAPM_EVT_ADV_STOP:
      s_advertising = false;
      break;
    case BLE_GAPC_EVT_CONNECTED:
      s_advertising = false;
      s_conn_idx = e->evt.gapc_evt.index;
      s_mtu = 23;
      s_cccd = 0;
      s_secured = false;
      // pairing is started by the central when it touches a protected characteristic
      if (instance) { instance->_txBusy = false; }
      BLE_DEBUG_PRINTLN("connected idx=%d", s_conn_idx);
      break;
    case BLE_GAPC_EVT_DISCONNECTED:
      BLE_DEBUG_PRINTLN("disconnected reason=0x%02x", e->evt.gapc_evt.params.disconnected.reason);
      s_conn_idx = -1;
      s_cccd = 0;
      s_secured = false;
      if (instance) instance->_txBusy = false;
      advStart();
      break;
    case BLE_GAPC_EVT_CONN_PARAM_UPDATE_REQ:
      ble_gap_conn_param_update_reply(e->evt.gapc_evt.index, true);
      break;
    case BLE_GATT_COMMON_EVT_MTU_EXCHANGE:
      s_mtu = e->evt.gatt_common_evt.params.mtu_exchange.mtu;
      BLE_DEBUG_PRINTLN("MTU=%u", s_mtu);
      break;
    default:
      onSecEvent(e);    // BLE_SEC_EVT_* (pairing)
      break;
  }
}

static void onSecEvent(const ble_evt_t* e) {
  switch (e->evt_id) {
    case BLE_SEC_EVT_LINK_ENC_REQUEST: {
      const ble_sec_evt_enc_req_t& rq = e->evt.sec_evt.params.enc_req;
      ble_sec_cfm_enc_t cfm;
      memset(&cfm, 0, sizeof(cfm));
      cfm.req_type = rq.req_type;
      cfm.accept = true;
      if (rq.req_type == BLE_SEC_TK_REQ) {
        // static passkey: the device has no display, the user types the PIN on the phone
        cfm.data.tk.key[0] = (uint8_t)(s_pin);
        cfm.data.tk.key[1] = (uint8_t)(s_pin >> 8);
        cfm.data.tk.key[2] = (uint8_t)(s_pin >> 16);
        cfm.data.tk.key[3] = (uint8_t)(s_pin >> 24);
      }
      logEvt(0xE002, ble_sec_enc_cfm(e->evt.sec_evt.index, &cfm), rq.req_type);
      BLE_DEBUG_PRINTLN("security request type=%d", (int)rq.req_type);
      break;
    }
    case BLE_SEC_EVT_LINK_ENCRYPTED:
      s_secured = (e->evt_status == BLE_SUCCESS);
      BLE_DEBUG_PRINTLN("link encrypted status=0x%02x", e->evt_status);
      break;
    default:
      break;
  }
}

static void onStackReady() {
  uint16_t lvl = s_pin ? BLE_GATTS_AUTH : BLE_GATTS_NOAUTH;
  uint16_t val_ext = BLE_GATTS_ATT_VAL_LOC_USER | BLE_GATTS_ATT_UUID_TYPE_SET(BLE_GATTS_UUID_TYPE_128);
  setAttr(NUS_IDX_SVC, s_uuid_svc, BLE_GATTS_READ_PERM_UNSEC, 0, 0);
  setAttr(NUS_IDX_RX_CHAR, s_uuid_char, BLE_GATTS_READ_PERM_UNSEC, 0, 0);
  setAttr(NUS_IDX_RX_VAL, s_uuid_rx, BLE_GATTS_WRITE_REQ_PERM(lvl) | BLE_GATTS_WRITE_CMD_PERM(lvl), val_ext, MAX_FRAME_SIZE);
  setAttr(NUS_IDX_TX_CHAR, s_uuid_char, BLE_GATTS_READ_PERM_UNSEC, 0, 0);
  setAttr(NUS_IDX_TX_VAL, s_uuid_tx, BLE_GATTS_NOTIFY_PERM(lvl), val_ext, MAX_FRAME_SIZE);
  setAttr(NUS_IDX_TX_CFG, s_uuid_cccd, BLE_GATTS_READ_PERM_UNSEC | BLE_GATTS_WRITE_REQ_PERM(lvl), 0, 0);

  if (s_pin) {
    ble_sec_param_t sec;
    memset(&sec, 0, sizeof(sec));
    sec.level = BLE_SEC_MODE1_LEVEL3;              // encrypted + MITM
    sec.io_cap = BLE_SEC_IO_DISPLAY_ONLY;          // static passkey entered on the phone
    sec.oob = false;
    sec.auth = BLE_SEC_AUTH_BOND | BLE_SEC_AUTH_MITM | BLE_SEC_AUTH_SEC_CON;
    sec.key_size = 16;
    sec.ikey_dist = BLE_SEC_KDIST_ENCKEY | BLE_SEC_KDIST_IDKEY;
    sec.rkey_dist = BLE_SEC_KDIST_ENCKEY | BLE_SEC_KDIST_IDKEY;
    ble_gap_pair_enable(true);
    logEvt(0xE000, ble_sec_params_set(&sec));
  }

  memset(&s_db, 0, sizeof(s_db));
  s_start_hdl = PRF_INVALID_HANDLE;
  s_db.shdl = &s_start_hdl;
  s_db.uuid = s_svc_uuid;
  s_db.attr_tab_cfg = (uint8_t*)&s_char_mask;
  s_db.max_nb_attr = NUS_IDX_NB;
  s_db.srvc_perm = BLE_GATTS_SRVC_UUID_TYPE_SET(BLE_GATTS_UUID_TYPE_128);
  s_db.attr_tab_type = BLE_GATTS_SERVICE_TABLE_TYPE_128;
  s_db.attr_tab.attr_tab_128 = s_attr_tab;
  ble_gatts_prf_add(&s_db, prfEvent);

  ble_gap_device_name_set(BLE_GAP_WRITE_PERM_DISABLE, (uint8_t*)s_dev_name, (uint16_t)strlen(s_dev_name));

  static const uint8_t adv_data[] = { 0x11, BLE_GAP_AD_TYPE_COMPLETE_LIST_128_BIT_UUID, NUS_SERVICE_UUID };
  static uint8_t scan_rsp[31];
  size_t n = strlen(s_dev_name);
  if (n > sizeof(scan_rsp) - 2) n = sizeof(scan_rsp) - 2;
  scan_rsp[0] = (uint8_t)(n + 1);
  scan_rsp[1] = BLE_GAP_AD_TYPE_COMPLETE_NAME;
  memcpy(&scan_rsp[2], s_dev_name, n);

  s_adv_param.adv_intv_min = BLE_ADV_INTERVAL_MIN;
  s_adv_param.adv_intv_max = BLE_ADV_INTERVAL_MAX;
  s_adv_param.adv_mode     = BLE_GAP_ADV_TYPE_ADV_IND;
  s_adv_param.chnl_map     = BLE_GAP_ADV_CHANNEL_37_38_39;
  s_adv_param.disc_mode    = BLE_GAP_DISC_MODE_GEN_DISCOVERABLE;
  s_adv_param.filter_pol   = BLE_GAP_ADV_ALLOW_SCAN_ANY_CON_ANY;
  ble_gap_adv_param_set(0, BLE_GAP_OWN_ADDR_STATIC, &s_adv_param);
  ble_gap_adv_data_set(0, BLE_GAP_ADV_DATA_TYPE_DATA, adv_data, sizeof(adv_data));
  ble_gap_adv_data_set(0, BLE_GAP_ADV_DATA_TYPE_SCAN_RSP, scan_rsp, (uint16_t)(n + 2));
  ble_gap_l2cap_params_set(247, 247, 1);
  ble_gap_data_length_set(251, 2120);
  ble_gap_pref_phy_set(BLE_GAP_PHY_ANY, BLE_GAP_PHY_ANY);
  s_adv_time.duration = 0;
  s_adv_time.max_adv_evt = 0;
  advStart();
}

void SerialBLEInterface::begin(const char* prefix, char* name, uint32_t pin_code) {
  instance = this;
  s_pin = pin_code;

  if (strcmp(name, "@@MAC") == 0) {
    ble_gap_bdaddr_t addr;
    if (ble_gap_addr_get(&addr) == SDK_SUCCESS) {
      const uint8_t* a = addr.gap_addr.addr;
      sprintf(name, "%02X%02X%02X%02X%02X%02X", a[5], a[4], a[3], a[2], a[1], a[0]);  // modify (IN-OUT param)
    }
  }
  snprintf(s_dev_name, sizeof(s_dev_name), "%s%s", prefix, name);

  gr551x_ble_add_handler(gapEvent);
  gr551x_ble_on_ready(onStackReady);
}

void SerialBLEInterface::clearBuffers() {
  uint32_t primask = __get_PRIMASK();
  __disable_irq();
  send_queue_len = 0;
  recv_queue_len = 0;
  __set_PRIMASK(primask);
}

void SerialBLEInterface::enable() {
  if (_isEnabled) return;
  _isEnabled = true;
  clearBuffers();
  _last_health_check = millis();
  if (gr551x_ble_ready()) advStart();
}

void SerialBLEInterface::disable() {
  _isEnabled = false;
  if (s_conn_idx >= 0) ble_gap_disconnect((uint8_t)s_conn_idx);
  if (s_advertising) ble_gap_adv_stop(0);
}

bool SerialBLEInterface::isConnected() const {
  return s_conn_idx >= 0 && s_cccd == PRF_CLI_START_NTF && (s_secured || !s_pin);
}

bool SerialBLEInterface::isWriteBusy() const {
  return send_queue_len >= (FRAME_QUEUE_SIZE * 2 / 3);
}

void SerialBLEInterface::onRx(const uint8_t* data, uint16_t len) {
  if (len == 0 || len > MAX_FRAME_SIZE) {
    BLE_DEBUG_PRINTLN("onRx: bad frame len=%u", len);
    return;
  }
  if (recv_queue_len >= FRAME_QUEUE_SIZE) {
    BLE_DEBUG_PRINTLN("onRx: recv queue full, dropping");
    return;
  }
  recv_queue[recv_queue_len].len = (uint8_t)len;
  memcpy(recv_queue[recv_queue_len].buf, data, len);
  recv_queue_len = recv_queue_len + 1;
}

size_t SerialBLEInterface::writeFrame(const uint8_t src[], size_t len) {
  if (len > MAX_FRAME_SIZE || len == 0 || !isConnected()) return 0;
  if (send_queue_len >= FRAME_QUEUE_SIZE) {
    BLE_DEBUG_PRINTLN("writeFrame(), send_queue is full!");
    return 0;
  }
  send_queue[send_queue_len].len = (uint8_t)len;
  memcpy(send_queue[send_queue_len].buf, src, len);
  send_queue_len = send_queue_len + 1;
  return len;
}

size_t SerialBLEInterface::checkRecvFrame(uint8_t dest[]) {
  // send: one queued frame per call, once the previous notification has completed
  if (send_queue_len > 0) {
    if (!isConnected()) {
      send_queue_len = 0;
    } else if (!_txBusy && send_queue[0].len <= s_mtu - 3) {
      ble_gatts_noti_ind_t ntf;
      ntf.type = BLE_GATT_NOTIFICATION;
      ntf.handle = prf_find_handle_by_idx(NUS_IDX_TX_VAL, s_start_hdl, (uint8_t*)&s_char_mask);
      ntf.length = send_queue[0].len;
      ntf.value = send_queue[0].buf;
      _txBusy = true;
      if (ble_gatts_noti_ind((uint8_t)s_conn_idx, &ntf) == SDK_SUCCESS) {
        BLE_DEBUG_PRINTLN("writeBytes: sz=%u, hdr=%u", (unsigned)ntf.length, (unsigned)ntf.value[0]);
        uint32_t primask = __get_PRIMASK();
        __disable_irq();
        send_queue_len--;
        memmove(&send_queue[0], &send_queue[1], send_queue_len * sizeof(Frame));
        __set_PRIMASK(primask);
      } else {
        _txBusy = false;    // retry on next call
      }
    }
  }

  // receive
  if (recv_queue_len > 0) {
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    size_t len = recv_queue[0].len;
    memcpy(dest, recv_queue[0].buf, len);
    recv_queue_len--;
    memmove(&recv_queue[0], &recv_queue[1], recv_queue_len * sizeof(Frame));
    __set_PRIMASK(primask);
    BLE_DEBUG_PRINTLN("readBytes: sz=%u, hdr=%u", (unsigned)len, (unsigned)dest[0]);
    return len;
  }

  // advertising watchdog
  unsigned long now = millis();
  if (_isEnabled && s_conn_idx < 0 && now - _last_health_check >= BLE_HEALTH_CHECK_INTERVAL) {
    _last_health_check = now;
    if (!s_advertising) {
      BLE_DEBUG_PRINTLN("advertising watchdog: restarting");
      advStart();
    }
  }
  return 0;
}
