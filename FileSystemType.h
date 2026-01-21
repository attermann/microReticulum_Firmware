#pragma once

#ifdef HAS_RNS

// CBA This header file was required to break-out defined and includes here that could
//  not be include in FileSystem.h due to conlficts with SPIFFS in Console.h

#include "Boards.h"

#define FS_TYPE_SPIFFS 0
#define FS_TYPE_LITTLEFS 1
#define FS_TYPE_INTERNALFS 2
#define FS_TYPE_FLASHFS 3

#if MCU_VARIANT == MCU_ESP32
	#if defined(USE_FLASHFS)
		#define FS_TYPE FS_TYPE_FLASHFS
	#else
		//#define FS_TYPE FS_TYPE_SPIFFS
		#define FS_TYPE FS_TYPE_LITTLEFS
	#endif
#elif MCU_VARIANT == MCU_NRF52
	#if defined(USE_FLASHFS)
		#define FS_TYPE FS_TYPE_FLASHFS
	#else
		#define FS_TYPE FS_TYPE_INTERNALFS
	#endif
#else
	#define FS_TYPE FS_TYPE_SPIFFS
#endif

#if FS_TYPE == FS_TYPE_SPIFFS
#include <SPIFFS.h>
#define FS SPIFFS
#elif FS_TYPE == FS_TYPE_LITTLEFS
#include <LittleFS.h>
#define FS LittleFS
#elif FS_TYPE == FS_TYPE_INTERNALFS
#include <InternalFileSystem.h>
#define FS InternalFS
using namespace Adafruit_LittleFS_Namespace;
#elif FS_TYPE == FS_TYPE_FLASHFS
#include <Cached_SPIFlash.h>
#include <FlashFileSystem.h>
#define FS FlashFS
using namespace Adafruit_LittleFS_Namespace;
#else
#error "FileSystem type not specified"
#endif

#endif
