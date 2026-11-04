/*
 * See bridge_protocol.h for the wire format.
 */

#include "bridge_protocol.h"
#include <string.h>

static inline uint16_t rd16(const uint8_t *p) {
  return (uint16_t)(p[0] | (p[1] << 8));
}

static bool header_is_valid(uint16_t header) {
  return header == BRIDGE_HDR_KEYBOARD || header == BRIDGE_HDR_MOUSE_MOVE ||
         header == BRIDGE_HDR_MOUSE_REL_MOVE || header == BRIDGE_HDR_SYSTEM_KEY ||
         header == BRIDGE_HDR_STATUS_REQ;
}

static bridge_evt_type_t header_to_evt(uint16_t header) {
  switch (header) {
    case BRIDGE_HDR_KEYBOARD:
      return BRIDGE_EVT_KEYBOARD;
    case BRIDGE_HDR_MOUSE_MOVE:
      return BRIDGE_EVT_MOUSE_MOVE;
    case BRIDGE_HDR_MOUSE_REL_MOVE:
      return BRIDGE_EVT_MOUSE_REL_MOVE;
    case BRIDGE_HDR_SYSTEM_KEY:
      return BRIDGE_EVT_SYSTEM_KEY;
    case BRIDGE_HDR_STATUS_REQ:
      return BRIDGE_EVT_STATUS_REQ;
    default:
      return BRIDGE_EVT_NONE;
  }
}

// Drop the oldest byte and let the remaining ones re-synchronise.
static void drop_first_byte(bridge_parser_t *parser) {
  parser->len--;
  if (parser->len > 0) {
    memmove(parser->buf, parser->buf + 1, parser->len);
  }
}

void bridge_parser_init(bridge_parser_t *parser) {
  parser->len = 0;
}

bool bridge_parser_feed(bridge_parser_t *parser, uint8_t byte, bridge_evt_t *evt) {
  if (parser->len >= BRIDGE_FRAME_LEN) {
    // Should not happen, but never let the buffer overflow.
    parser->len = 0;
  }
  parser->buf[parser->len++] = byte;

  while (parser->len >= 2) {
    const uint16_t header = rd16(parser->buf);
    if (!header_is_valid(header)) {
      drop_first_byte(parser);
      continue;
    }

    if (parser->len < BRIDGE_FRAME_LEN) {
      return false; // waiting for the rest of the frame
    }

    if (rd16(parser->buf + BRIDGE_FRAME_LEN - 2) != BRIDGE_FRAME_TAIL) {
      drop_first_byte(parser);
      continue;
    }

    evt->type = header_to_evt(header);
    switch (evt->type) {
      case BRIDGE_EVT_KEYBOARD:
        memcpy(&evt->keyboard, parser->buf + 2, sizeof(evt->keyboard));
        break;
      case BRIDGE_EVT_SYSTEM_KEY:
        memcpy(&evt->system, parser->buf + 2, sizeof(evt->system));
        break;
      case BRIDGE_EVT_STATUS_REQ:
        memcpy(&evt->status_req, parser->buf + 2, sizeof(evt->status_req));
        break;
      case BRIDGE_EVT_MOUSE_REL_MOVE:
        memcpy(&evt->mouse_rel, parser->buf + 2, sizeof(evt->mouse_rel));
        break;
      case BRIDGE_EVT_MOUSE_MOVE:
      default:
        memcpy(&evt->mouse, parser->buf + 2, sizeof(evt->mouse));
        break;
    }

    parser->len = 0;
    return true;
  }

  return false;
}
