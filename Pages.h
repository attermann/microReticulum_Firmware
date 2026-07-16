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

#include <Arduino.h>

#include "Config.h"

#include <microReticulum/Interface.h>
#include <microReticulum/Identity.h>
#include <microReticulum/Utilities/OS.h>
#include <microReticulum/Bytes.h>
#ifdef HAS_BME
#include "BME680.h"
#endif
// CBA NOTE Thge following <MsgPack.h> include *MUST* precede the "Utilities.h" include
#include <MsgPack.h>

#include <string>

extern RNS::Interface lora_interface;
#if HAS_WIFI && defined(UDP_TRANSPORT)
extern RNS::Interface udp_interface;
extern IPAddress wr_device_ip;
extern uint16_t udp_port;
extern uint8_t wifi_mode;
extern char wr_ssid[];
#endif
extern RNS::Destination nomadnet_destination;

void add_interface_details(RNS::Bytes& content, const RNS::Interface& interface) {
  content << "    \"mode\": \"";
  switch (interface.mode()) {
    case RNS::Type::Interface::MODE_ACCESS_POINT:
      content << "ACCESS_POINT";
      break;
    case RNS::Type::Interface::MODE_BOUNDARY:
      content << "BOUNDARY";
      break;
    case RNS::Type::Interface::MODE_FULL:
      content << "FULL";
      break;
    case RNS::Type::Interface::MODE_GATEWAY:
      content << "GATEWAY";
      break;
    case RNS::Type::Interface::MODE_POINT_TO_POINT:
      content << "POINT_TO_POINT";
      break;
    case RNS::Type::Interface::MODE_ROAMING:
      content << "ROAMING";
      break;
    case RNS::Type::Interface::MODE_NONE:
      content << "NONE";
      break;
    default:
      break;
  }
  content << "\",\n";
  content << "    \"bitrate\": " << std::to_string(interface.bitrate()) << ",\n";
  content << "    \"packets_sent\": " << std::to_string(interface.tx()) << ",\n";
  content << "    \"packets_received\": " << std::to_string(interface.rx()) << ",\n";
  content << "    \"bytes_sent\": " << std::to_string(interface.txbytes()) << ",\n";
  content << "    \"bytes_received\": " << std::to_string(interface.rxbytes()) << ",\n";
  content << "    \"queued_announces\": " << std::to_string(interface.announce_queue().size()) << ",\n";
}

// Request handler for NomadNet pages. Signature is fixed by
// RNS::RequestHandler::response_generator (see src/Destination.h).
//
// The Link::request contract is that the return value must already be
// msgpack-encoded bytes — Link::handle_request splices it verbatim into
// the response envelope (Link.cpp:994). Python RNS auto-encodes
// arbitrary return values; in C++ we encode manually here.
RNS::Bytes serve_page(
	const RNS::Bytes& path,
	const RNS::Bytes& data,
	const RNS::Bytes& request_id,
	const RNS::Bytes& link_id,
	const RNS::Identity& remote_identity,
	double requested_at
) {

	std::string category;
	{
		MsgPack::Unpacker u;
		u.feed(data.data(), data.size());
		std::map<String, String> params;
		if (u.isMap()) {
			u.deserialize(params);
			for (const auto& [key, value] : params) {
				TRACEF("Param: key=%s, value=%s", key.c_str(), value.c_str());
			}
		}

		auto iter = params.find("var_c");
		if (iter != params.end()) {
			category = (*iter).second.c_str();
		}
	}

	VERBOSEF("Serving page %s with category \"%s\" to link <%s> with identity <%s>", path.toString().c_str(), category.c_str(), link_id.toHex().c_str(), (remote_identity ? remote_identity.hash().toHex().c_str() : RNS::Bytes{}.toHex().c_str()));
	MsgPack::Packer packer;
  {
    RNS::Bytes content;
    if (path == "/page/index.mu") {
      content = "> microReticulum Stats\n\n";
      content << ">> Memory\n";
      content << "`!`[• Heap Memory`:/page/stack.mu`c=heap]`\n";
      content << "`!`[• Memory Pools`:/page/stack.mu`c=pool]`\n";
      content << "`!`[• Memory Allocators`:/page/stack.mu`c=alloc]`\n";
      content << ">> Storage\n";
      content << "`!`[• Flash Memory`:/page/stack.mu`c=flash]`\n";
      content << "`!`[• Object Stores`:/page/stack.mu`c=store]`\n";
      content << "`!`[• Transport Metrics`:/page/stack.mu`c=metrics]`\n";
      content << ">> Device\n";
      content << "`!`[• General`:/page/device.mu`c=general]`\n";
      content << "`!`[• Interface`:/page/device.mu`c=interfaces]`\n";
#ifdef HAS_BME
      if (BME680::bme_installed) {
        content << "`!`[• Telemetry`:/page/telemetry.mu]`\n";
      }
#endif
      if (remote_identity) content << "\n🛡️ Verified identity: " << remote_identity.hash().toHex() << "\n";
      else content << "\n⚠️ Unknown identity. Identity must be provided for access to this site.\n";
    }
    else if (path == "/page/stack.mu") {
  	  if (category == "heap") {
        content = "{\n";
        content << "  \"heap_size\": " << std::to_string(RNS::Utilities::Memory::heap_size()) << ",\n";
        content << "  \"heap_free\": " << std::to_string(RNS::Utilities::Memory::heap_available()) << ",\n";
        content << "  \"heap_freepct\": " << std::to_string((unsigned)((double)RNS::Utilities::Memory::heap_available() / (double)RNS::Utilities::Memory::heap_size() * 100.0)) << ",\n";
#if defined(ESP32)
        content << "  \"heap_minfree\": " << std::to_string(ESP.getMinFreeHeap()) << ",\n";
        content << "  \"heap_maxalloc\": " << std::to_string(ESP.getMaxAllocHeap()) << ",\n";
        content << "  \"heap_fragmented\": " << std::to_string((unsigned)(100.0 - (double)ESP.getMaxAllocHeap() / (double)ESP.getFreeHeap() * 100.0)) << ",\n";
        content << "  \"psram_size\": " << std::to_string(ESP.getPsramSize()) << ",\n";
        content << "  \"psram_free\": " << std::to_string(ESP.getFreePsram()) << ",\n";
        content << "  \"psram_freepct\": " << std::to_string((ESP.getPsramSize() > 0) ? (unsigned)((double)ESP.getFreePsram() / (double)ESP.getPsramSize() * 100.0) : 0) << ",\n";
        content << "  \"psram_minfree\": " << std::to_string(ESP.getMinFreePsram()) << ",\n";
        content << "  \"psram_maxalloc\": " << std::to_string(ESP.getMaxAllocPsram()) << ",\n";
        content << "  \"psram_fragmented\": " << std::to_string((ESP.getFreePsram() > 0) ? (unsigned)(100.0 - (double)ESP.getMaxAllocPsram() / (double)ESP.getFreePsram() * 100.0) : 0) << ",\n";
#elif defined(ARDUINO_ARCH_NRF52) || defined(ARDUINO_NRF52_ADAFRUIT)
        //HEAD("Heap Stats", LOG_TRACE);
        //if (loglevel() == LOG_TRACE) {
        //	dbgMemInfo();
        //}
#endif
      	content << "}";
	    }
      else if (category == "flash") {
        size_t flash_size = RNS::Utilities::OS::storage_size();
        size_t flash_free = RNS::Utilities::OS::storage_available();
        uint8_t flash_freepct = 0;
        if (flash_size > 0) flash_freepct = (uint8_t)((double)flash_free / (double)flash_size * 100.0);
        //size_t fs_size = RNS::Utilities::OS::get_filesystem().storageSize();
        //size_t fs_free = RNS::Utilities::OS::get_filesystem().storageAvailable();
        //uint8_t fs_freepct = 0;
        //if (fs_size > 0) fs_freepct = (uint8_t)((double)fs_free / (double)fs_size * 100.0);

        content = "{\n";
        content << "  \"flash_size\": " << std::to_string(flash_size) << ",\n";
        content << "  \"flash_free\": " << std::to_string(flash_free) << ",\n";
        content << "  \"flash_freepct\": " << std::to_string(flash_freepct) << ",\n";
        //content << "  \"fs_size\": " << std::to_string(fs_size) << ",\n";
        //content << "  \"fs_free\": " << std::to_string(fs_free) << ",\n";
        //content << "  \"fs_freepct\": " << std::to_string(fs_freepct) << ",\n";
      	content << "}";
      }
      else if (category == "pool") {
        content = "NOT YET IMPLEMENTED\n";
      }
      else if (category == "alloc") {
        content = "NOT YET IMPLEMENTED\n";
      }
      else if (category == "store") {
        uint32_t destination_path_responses = 0;
        for (auto& [destination_hash, destination] : RNS::Transport::destinations()) {
          destination_path_responses += destination.path_responses().size();
        }
        uint32_t interface_announces = 0;
        for (auto& interface : RNS::Transport::get_interfaces()) {
          interface_announces += interface.announce_queue().size();
        }

        content = "{\n";
        content << "  \"paths\": " << std::to_string(RNS::Transport::new_path_table().size()) << ",\n";
        content << "  \"destinations\": " << std::to_string(RNS::Transport::destinations().size()) << ",\n";
        content << "  \"announces\": " << std::to_string(RNS::Transport::announce_table().size()) << ",\n";
        content << "  \"held_announces\": " << std::to_string(RNS::Transport::held_announces().size()) << ",\n";

        content << "  \"path_requests\": " << std::to_string(RNS::Transport::path_requests().size()) << ",\n";
        content << "  \"discovery_path_requests\": " << std::to_string(RNS::Transport::discovery_path_requests().size()) << ",\n";
        content << "  \"pending_local_path_requests\": " << std::to_string(RNS::Transport::pending_local_path_requests().size()) << ",\n";
        content << "  \"discovery_pr_tags\": " << std::to_string(RNS::Transport::discovery_pr_tags().size()) << ",\n";
        content << "  \"control_destinations\": " << std::to_string(RNS::Transport::control_destinations().size()) << ",\n";
        content << "  \"control_hashes\": " << std::to_string(RNS::Transport::control_hashes().size()) << ",\n";

        content << "  \"packet_hashes\": " << std::to_string(RNS::Transport::packet_hashlist().size()) << ",\n";
        content << "  \"reverse_hashes\": " << std::to_string(RNS::Transport::reverse_table().size()) << ",\n";
        content << "  \"receipts\": " << std::to_string(RNS::Transport::receipts().size()) << ",\n";

        content << "  \"links\": " << std::to_string(RNS::Transport::link_table().size()) << ",\n";
        content << "  \"pending_links\": " << std::to_string(RNS::Transport::pending_links().size()) << ",\n";
        content << "  \"active_links\": " << std::to_string(RNS::Transport::active_links().size()) << ",\n";
        content << "  \"tunnels\": " << std::to_string(RNS::Transport::tunnels().size()) << ",\n";

        content << "  \"known_destinations\": " << std::to_string(RNS::Identity::known_destinations().size()) << ",\n";
        content << "  \"destination_path_responses\": " << std::to_string(destination_path_responses) << ",\n";
        content << "  \"queued_announces\": " << std::to_string(interface_announces) << ",\n";

        content << "}";
      }
      else if (category == "metrics") {
        content = "{\n";
        content << "  \"packets_sent\": " << std::to_string(RNS::Transport::packets_sent()) << ",\n";
        content << "  \"packets_received\": " << std::to_string(RNS::Transport::packets_received()) << ",\n";
        content << "  \"paths_added\": " << std::to_string(RNS::Transport::paths_added()) << ",\n";
        content << "  \"paths_updated\": " << std::to_string(RNS::Transport::paths_updated()) << ",\n";
        content << "  \"paths_failed\": " << std::to_string(RNS::Transport::paths_failed()) << ",\n";
      	content << "}";
      }
      else {
        content = "CATEGORY NOT FOUND\n";
      }
    }
    else if (path == "/page/device.mu") {
  	  if (category == "general") {
        content = "{\n";
        content << "  \"firmware_version\": \"" << std::to_string(MAJ_VERS) << "." << std::to_string(MIN_VERS) << "\",\n";
        content << "  \"battery_voltage\": " << std::to_string((float)((uint16_t)(battery_voltage*100)) / 100.0) << ",\n";
        content << "  \"battery_percent\": " << std::to_string(((uint8_t)battery_percent)) << ",\n";
        content << "  \"battery_state\": \"";
        switch (battery_state) {
          case BATTERY_STATE_CHARGING:
            content << "CHARGING";
            break;
          case BATTERY_STATE_CHARGED:
            content << "CHARGED";
            break;
          case BATTERY_STATE_DISCHARGING:
            content << "DISCHARGING";
            break;
          case BATTERY_STATE_UNKNOWN:
            content << "UNKNOWN";
            break;
          default:
            break;
        }
        content << "\",\n";
        content << "  \"transport_identity\": \"" << (RNS::Transport::identity() ? RNS::Transport::identity().hash().toHex() : RNS::Bytes{}.toHex()) << "\",\n";
        content << "  \"probe_destination\": \"" << (RNS::Transport::probe_destination() ? RNS::Transport::probe_destination().hash().toHex() : RNS::Bytes{}.toHex()) << "\",\n";
        content << "  \"mgmt_destination\": \"" << (RNS::Transport::remote_management_destination() ? RNS::Transport::remote_management_destination().hash().toHex() : RNS::Bytes{}.toHex()) << "\",\n";
        content << "  \"nomadnet_destination\": \"" << (nomadnet_destination ? nomadnet_destination.hash().toHex() : RNS::Bytes{}.toHex()) << "\",\n";
      	content << "}";
      }
  	  else if (category == "interfaces") {
        content = "{\n";
#if defined(LORA_TRANSPORT)
        content << "  \"" << lora_interface.name().c_str() << "\": {\n";
        content << "    \"frequency\": " << std::to_string(lora_freq) << ",\n";
        content << "    \"bandwidth\": " << std::to_string(lora_bw) << ",\n";
        content << "    \"tx_power\": " << std::to_string(lora_txp) << ",\n";
        content << "    \"spreading_factor\": " << std::to_string(lora_sf) << ",\n";
        content << "    \"coding_rate\": " << std::to_string(lora_cr) << ",\n";
        content << "    \"current_rssi\": " << std::to_string(current_rssi) << ",\n";
        content << "    \"noise_floor\": " << std::to_string((int16_t)noise_floor) << ",\n";
        content << "    \"last_rssi\": " << std::to_string((int16_t)last_rssi) << ",\n";
        content << "    \"last_snr\": " << std::to_string((int8_t)((int8_t)last_snr_raw) / 4.0f) << ",\n";
        add_interface_details(content, lora_interface);
      	content << "  },\n";
#endif
#if defined(UDP_TRANSPORT)
        if (wifi_mode != WR_WIFI_OFF && udp_interface) {
          content << "  \"" << udp_interface.name().c_str() << "\": {\n";
          content << "    \"ip_addr\": \"" << wr_device_ip.toString().c_str() << "\",\n";
          content << "    \"udp_port\": " << std::to_string(udp_port) << ",\n";
          content << "    \"wifi_ssid\": \"" << wr_ssid << "\",\n";
          add_interface_details(content, udp_interface);
      	  content << "  },\n";
        }
#endif
      	content << "}";
      }
      else {
        content = "CATEGORY NOT FOUND\n";
      }
    }
#ifdef HAS_BME
    else if (path == "/page/telemetry.mu") {
      if (!BME680::bme.performReading()) {
        content = "> Telemetry\n\n`!`❌ Failed to perform BME680 reading.`\n";
      } else {
        content = "> Environmental Telemetry\n\n";
        content << "🌡️ Temperature: " << std::to_string((float)BME680::bme.temperature) << " °C\n";
        content << "💧 Humidity: " << std::to_string((float)BME680::bme.humidity) << " %\n";
        content << "⏲️ Pressure: " << std::to_string((float)(BME680::bme.pressure / 100.0)) << " hPa\n";
        content << "💨 Gas Resistance: " << std::to_string((float)(BME680::bme.gas_resistance / 1000.0)) << " KOhms\n";
      }
    }
#endif
    else {
      content = "PATH NOT FOUND\n";
    }
    packer.packBinary(content.data(), content.size());
  }
	return RNS::Bytes(packer.data(), packer.size());
}
