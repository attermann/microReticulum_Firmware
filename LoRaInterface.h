// Copyright (C) 2026, Chad Attermann

// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.

#pragma once

#include "Config.h"
#include "Boards.h"
// CBA NOTE: Can *NOT* include "Utilities.h" here due to potential conflict with <MsgPack.h>
//#include "Utilities.h"

#include <microReticulum.h>

extern uint32_t lora_bitrate;
extern volatile uint8_t queue_height;
extern volatile uint16_t queued_bytes;
extern volatile uint16_t queue_cursor;
extern volatile uint16_t current_packet_start;
typedef struct FIFOBuffer16 FIFOBuffer16;
extern FIFOBuffer16 packet_starts;
extern FIFOBuffer16 packet_lengths;
extern uint8_t packet_queue[];
extern bool fifo16_isfull(const FIFOBuffer16 *f);
extern void fifo16_push(FIFOBuffer16 *f, uint16_t c);


class LoRaInterface : public RNS::InterfaceImpl {
public:
	LoRaInterface(const char *name) : RNS::InterfaceImpl(name) {
		_IN = true;
		_OUT = true;
		_HW_MTU = 508;
    _bitrate = lora_bitrate;
	}
	LoRaInterface() : LoRaInterface("LoRaInterface") {}
	virtual ~LoRaInterface() {
		_name = "deleted";
	}
protected:
	virtual void handle_incoming(const RNS::Bytes& data) {
    TRACEF("LoRaInterface.handle_incoming: (%u bytes) data: %s", data.size(), data.toHex().c_str());
    TRACE("LoRaInterface.handle_incoming: sending packet to rns...");
    try {
      InterfaceImpl::handle_incoming(data);
    }
    catch (const std::bad_alloc&) {
      ERROR("LoRaInterface::handle_incoming: bad_alloc - out of memory");
    }
    catch (std::exception& e) {
      ERRORF("LoRaInterface::handle_incoming: %s", e.what());
    }
  }
	virtual bool send_outgoing(const RNS::Bytes& data) {
    // CBA NOTE header will be addded later by transmit function
    TRACEF("LoRaInterface.send_outgoing: (%u bytes) data: %s", data.size(), data.toHex().c_str());
    try {
      TRACE("LoRaInterface.send_outgoing: adding packet to outgoing queue...");
      for (size_t i = 0; i < data.size(); i++) {
          if (queue_height < CONFIG_QUEUE_MAX_LENGTH && queued_bytes < CONFIG_QUEUE_SIZE) {
              queued_bytes++;
              packet_queue[queue_cursor++] = data.data()[i];
              if (queue_cursor == CONFIG_QUEUE_SIZE) queue_cursor = 0;
          }
      }
      if (!fifo16_isfull(&packet_starts) && queued_bytes < CONFIG_QUEUE_SIZE) {
          uint16_t s = current_packet_start;
          int16_t e = queue_cursor-1; if (e == -1) e = CONFIG_QUEUE_SIZE-1;
          uint16_t l;

          if (s != e) {
              l = (s < e) ? e - s + 1 : CONFIG_QUEUE_SIZE - s + e + 1;
          } else {
              l = 1;
          }

          if (l >= MIN_L) {
              queue_height++;

              fifo16_push(&packet_starts, s);
              fifo16_push(&packet_lengths, l);

              current_packet_start = queue_cursor;
          }

      }
      // Perform post-send housekeeping
      InterfaceImpl::handle_outgoing(data);
    }
    catch (const std::bad_alloc&) {
      ERROR("LoRaInterface::send_outgoing: bad_alloc - out of memory");
      return false;
    }
    catch (std::exception& e) {
      ERRORF("LoRaInterface::send_outgoing: %s", e.what());
      return false;
    }
    return true;
  }
};
