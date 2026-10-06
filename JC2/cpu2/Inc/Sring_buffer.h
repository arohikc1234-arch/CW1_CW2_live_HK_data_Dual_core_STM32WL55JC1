#ifndef __SRING_BUFFER_H
#define __SRING_BUFFER_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#define IPC_RING_TX_BASE      0x20008300UL
#define RING_TX_MAGIC         0x54585242UL  /* "TXRB" */
#define RING_TX_DEPTH         4U            /* holds DEPTH-1 packets */
#define TELEM_VALUE_COUNT     5U            /* 5 scaled int16 = 10 bytes */

#define TELEM_ID_B1           1U
#define TELEM_ID_B2           2U

struct __attribute__((packed)) tx_packet_s
{
  int16_t  data[TELEM_VALUE_COUNT];   /* 10 bytes */
  uint16_t id;                        /* 2 bytes */
  uint16_t seq;                       /* 2 bytes */
  uint16_t crc16;                     /* 2 bytes */
};

_Static_assert(sizeof(struct tx_packet_s) == 16, "tx_packet_s size");

struct shared_ring_tx
{
  volatile uint32_t magic;
  volatile uint32_t write_index;
  volatile uint32_t read_index;
  struct tx_packet_s slots[RING_TX_DEPTH];
};

#define SHARED_RING_TX \
  ((volatile struct shared_ring_tx *)IPC_RING_TX_BASE)

static inline int16_t telem_to_i16(float v, float scale)
{
  float x = v * scale;
  x += (x >= 0.0f) ? 0.5f : -0.5f;
  if (x > 32767.0f)  x = 32767.0f;
  if (x < -32768.0f) x = -32768.0f;
  return (int16_t)x;
}

#define IPC_RING_RX_BASE      0x20008500UL
#define RING_RX_MAGIC         0x52585242UL  /* "RXRB" */
#define RING_RX_DEPTH         4U
#define CMD_PAYLOAD_LEN       13U

#define CMD_TYPE_HK           0x01
#define CMD_TYPE_ADCS         0x03
#define CMD_TYPE_CAMERA       0x04
#define CMD_TYPE_EPDM         0x05

struct __attribute__((packed)) rx_command_s
{
  uint8_t  len;
  uint8_t  cmd[CMD_PAYLOAD_LEN];
  uint16_t crc16;
};

struct shared_ring_rx
{
  volatile uint32_t magic;
  volatile uint32_t write_index;
  volatile uint32_t read_index;
  struct rx_command_s slots[RING_RX_DEPTH];
};

#define SHARED_RING_RX \
  ((volatile struct shared_ring_rx *)IPC_RING_RX_BASE)

#ifndef IPCC_BASE
#  define IPCC_BASE           0x58000C00UL
#endif
#define IPCC_C1SCR_OFFSET     0x0008UL
#define IPCC_C1TOC2SR_OFFSET  0x000CUL
#define IPCC_C2SCR_OFFSET     0x0018UL
#define IPCC_C2TOC1SR_OFFSET  0x001CUL

#define IPCC_C1SCR     (*(volatile uint32_t *)(IPCC_BASE + IPCC_C1SCR_OFFSET))
#define IPCC_C1TOC2SR  (*(volatile uint32_t *)(IPCC_BASE + IPCC_C1TOC2SR_OFFSET))
#define IPCC_C2SCR     (*(volatile uint32_t *)(IPCC_BASE + IPCC_C2SCR_OFFSET))
#define IPCC_C2TOC1SR  (*(volatile uint32_t *)(IPCC_BASE + IPCC_C2TOC1SR_OFFSET))

#define IPCC_CH_HANDSHAKE     1U
#define IPCC_CH_BEACON        2U
#define IPCC_CH_COMMAND       3U

#define IPCC_RX_BIT(ch)       (1UL << ((ch) - 1U))
#define IPCC_TX_BIT(ch)       (1UL << (((ch) - 1U) + 16U))

static inline void ipcc_m4_send(uint32_t ch)
{
  IPCC_C1SCR = IPCC_TX_BIT(ch);
  __asm__ volatile ("dmb" ::: "memory");
}

static inline bool ipcc_m4_received(uint32_t ch)
{
  return (IPCC_C2TOC1SR & IPCC_RX_BIT(ch)) != 0U;
}

static inline void ipcc_m4_clear(uint32_t ch)
{
  IPCC_C1SCR = IPCC_RX_BIT(ch);
  __asm__ volatile ("dmb" ::: "memory");
}

static inline void ipcc_m0_send(uint32_t ch)
{
  IPCC_C2SCR = IPCC_TX_BIT(ch);
  __asm__ volatile ("dmb" ::: "memory");
}

static inline bool ipcc_m0_received(uint32_t ch)
{
  return (IPCC_C1TOC2SR & IPCC_RX_BIT(ch)) != 0U;
}

static inline void ipcc_m0_clear(uint32_t ch)
{
  IPCC_C2SCR = IPCC_RX_BIT(ch);
  __asm__ volatile ("dmb" ::: "memory");
}

#define IPC_RADIO_LOG_BASE    0x20008900UL
#define RADIO_LOG_MAGIC       0x524C4F47UL
#define RADIO_LOG_DEPTH       8U
#define RADIO_LOG_TEXT_MAX    88U

#define RADIO_EVT_CW          0x01
#define RADIO_EVT_B1          0x02
#define RADIO_EVT_B2          0x03
#define RADIO_EVT_RX_CMD      0x04
#define RADIO_EVT_TX_ACK      0x05
#define RADIO_EVT_TX_NACK     0x06
#define RADIO_EVT_INFO        0x07

struct radio_log_msg_s
{
  uint32_t timestamp_ms;
  uint8_t  event_type;
  char     text[RADIO_LOG_TEXT_MAX];
};

struct shared_radio_log_s
{
  volatile uint32_t magic;
  volatile uint32_t write_index;
  volatile uint32_t read_index;
  struct radio_log_msg_s slots[RADIO_LOG_DEPTH];
};

#define SHARED_RADIO_LOG \
  ((volatile struct shared_radio_log_s *)IPC_RADIO_LOG_BASE)

void rb_ipc_init(void);
bool live_telem_update_float(uint16_t beacon_id, int16_t d1, int16_t d2, int16_t d3, int16_t d4, int16_t d5);
bool live_telem_get_float(uint16_t *id, int16_t *d1, int16_t *d2, int16_t *d3, int16_t *d4, int16_t *d5);
bool rb_tx_read(struct tx_packet_s *pkt);
bool rb_tx_empty(void);
bool rb_tx_full(void);
bool rb_rx_write(const struct rx_command_s *cmd);
bool rb_rx_read(struct rx_command_s *cmd);
bool rb_rx_empty(void);
bool rb_rx_full(void);
void radio_log_init(void);
bool radio_log_write(uint8_t event_type, const char *msg);
bool radio_log_read(struct radio_log_msg_s *msg);
bool radio_log_empty(void);

#endif /* __SRING_BUFFER_H */
