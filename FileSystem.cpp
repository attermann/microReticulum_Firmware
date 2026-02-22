#include "FileSystem.h"
#include "FileStream.h"
#include "FileSystemType.h"

#ifdef HAS_RNS

#include <Log.h>

#if FS_TYPE == FS_TYPE_INTERNALFS

inline int _countLfsBlock(void *p, lfs_block_t block) {
	lfs_size_t *size = (lfs_size_t*) p;
	*size += 1;
	return 0;
}

lfs_ssize_t usedBlocks() {
    lfs_size_t size = 0;
    lfs_traverse(FS._getFS(), _countLfsBlock, &size);
    return size;
}

size_t usedBytes() {
	const lfs_config* config = FS._getFS()->cfg;
	const size_t usedBlockCount = usedBlocks();
	return config->block_size * usedBlockCount;
}

size_t totalBytes() {
	const lfs_config* config = FS._getFS()->cfg;
	return config->block_size * config->block_count;
}

#elif FS_TYPE == FS_TYPE_FLASHFS

Adafruit_FlashTransport_SPI g_flashTransport(SS, SPI);

//Flash definition structure for GD25Q16C Flash (RAK15001)
Cached_SPIFlash g_flash(&g_flashTransport);
SPIFlash_Device_t g_RAK15001 {
	.total_size = (1UL << 21),
	.start_up_time_us = 5000,
	.manufacturer_id = 0xc8,
	.memory_type = 0x40,
	.capacity = 0x15,
	.max_clock_speed_mhz = 15,
	.quad_enable_bit_mask = 0x00,
	.has_sector_protection = false,
	.supports_fast_read = true,
	.supports_qspi = false,
	.supports_qspi_writes = false,
	.write_status_register_split = false,
	.single_status_byte = true,
};

#endif


bool FileSystem::init() {
	TRACE("Initializing filesystem...");
	try {
#if FS_TYPE == FS_TYPE_SPIFFS
		// Initialize SPIFFS
		INFO("SPIFFS mounting filesystem");
		if (!SPIFFS.begin(true, "/spiffs")) {
			ERROR("SPIFFS filesystem mount failed");
			return false;
		}
		INFO("SPIFFS filesystem is ready");
#elif FS_TYPE == FS_TYPE_LITTLEFS
		// Initialize LittleFS
		INFO("LittleFS mounting filesystem");
		if (!LittleFS.begin(true, "/littlefs")) {
			ERROR("LittleFS filesystem mount failed");
			return false;
		}
		DEBUG("LittleFS filesystem is ready");
#elif FS_TYPE == FS_TYPE_INTERNALFS
		// Initialize InternalFileSystem
		INFO("InternalFS mounting filesystem");
		if (!InternalFS.begin()) {
			ERROR("InternalFS filesystem mount failed");
			return false;
		}
		INFO("InternalFS filesystem is ready");
#elif FS_TYPE == FS_TYPE_FLASHFS
		// Initialize FlashFileSystem
		INFO("FlashFS mounting filesystem");
		if (!g_flash.begin(&g_RAK15001)) {
			ERROR("FlashFS failed to initialize");
			return false;
		}
		if (!FlashFS.begin(&g_flash)) {
			ERROR("FlashFS filesystem mount failed");
			return false;
		}
#endif
		// Ensure filesystem is writable and reformat if not
		RNS::Bytes test("test");
		if (write_file("/test", test) < 4) {
			HEAD("Failed to write test file, filesystem is being reformatted...", RNS::LOG_CRITICAL);
			//FS.format();
			reformat();
		}
		else {
			remove_file("/test");
		}
	}
	catch (std::exception& e) {
		//ERROR("FileSystem init Exception: " + std::string(e.what()));
		return false;
	}
	TRACE("Finished initializing");
	return true;
}

bool FileSystem::format() {
	INFO("Formatting filesystem...");
	try {
		if (!FS.format()) {
			ERROR("Format failed!");
			return false;
		}
		return true;
	}
	catch (std::exception& e) {
		ERROR("FileSystem reformat Exception: " + std::string(e.what()));
	}
	return false;
}

bool FileSystem::reformat() {
	INFO("Reformatting filesystem...");
	try {
		RNS::Bytes eeprom;
		read_file("/eeprom", eeprom);
		RNS::Bytes transport_identity;
		read_file("/transport_identity", transport_identity);
		//RNS::Bytes time_offset;
		//read_file("/time_offset", time_offset);
		if (!FS.format()) {
			ERROR("Format failed!");
			return false;
		}
		if (eeprom) {
			write_file("/eeprom", eeprom);
		}
		if (transport_identity) {
			write_file("/transport_identity", transport_identity);
		}
		//if (time_offset) {
		//	write_file("/time_offset", time_offset);
		//}
		return true;
	}
	catch (std::exception& e) {
		ERROR("FileSystem reformat Exception: " + std::string(e.what()));
	}
	return false;
}

#ifndef NDEBUG

void FileSystem::listDir(const char* dir, const char* prefix /*= ""*/) {
	Serial.print(prefix);
	std::string full_dir(dir);
	if (full_dir.compare("/") != 0) {
		full_dir += "/";
	}
	Serial.println(full_dir.c_str());
	std::string pre(prefix);
	pre.append("  ");
	try {
		File root = FS.open(dir);
		if (!root) {
			Serial.print(pre.c_str());
			Serial.println("(failed to open directory)");
			return;
		}
		File file = root.openNextFile();
		while (file) {
			char* name = (char*)file.name();
			std::string recurse_dir(full_dir);
			if (file.isDirectory()) {
				recurse_dir += name;
				listDir(recurse_dir.c_str(), pre.c_str());
			}
			else {
				Serial.print(pre.c_str());
				//Serial.print("FILE: ");
				Serial.print(name);
				Serial.print(" (");
				Serial.print(file.size());
				Serial.println(" bytes)");
			}
			file.close();
			file = root.openNextFile();
		}
		root.close();
	}
	catch (std::exception& e) {
		Serial.print("listDir Exception: ");
		Serial.println(e.what());
	}
}

void FileSystem::dumpDir(const char* dir) {
	Serial.print("DIR: ");
	std::string full_dir(dir);
	if (full_dir.compare("/") != 0) {
		full_dir += "/";
	}
	Serial.println(full_dir.c_str());
	try {
		File root = FS.open(dir);
		if (!root) {
			Serial.println("(failed to open directory)");
			return;
		}
		File file = root.openNextFile();
		while (file) {
			char* name = (char*)file.name();
			if (file.isDirectory()) {
				std::string recurse_dir(full_dir);
				recurse_dir += name;
				dumpDir(recurse_dir.c_str());
			}
			else {
				Serial.print("\nFILE: ");
				Serial.print(name);
				Serial.print(" (");
				Serial.print(file.size());
				Serial.println(" bytes)");
				char data[4096];
				size_t size = file.size();
				size_t read = file.readBytes(data, (size < sizeof(data)) ? size : sizeof(data));
				Serial.write(data, read);
				Serial.println("");
			}
			file.close();
			file = root.openNextFile();
		}
		root.close();
	}
	catch (std::exception& e) {
		Serial.print("dumpDir Exception: ");
		Serial.println(e.what());
	}
}

#endif


/*virtua*/ bool FileSystem::file_exists(const char* file_path) {
	TRACEF("file_exists: checking for existence of file %s", file_path);
/*
#if FS_TYPE == FS_TYPE_INTERNALFS || FS_TYPE == FS_TYPE_FLASHFS
	File file(FS);
	if (file.open(file_path, FILE_O_READ)) {
#else
	File file = FS.open(file_path, FILE_READ);
	if (file) {
#endif
		bool is_directory = file.isDirectory();
		file.close();
		return !is_directory;
	}
	return false;
*/
	return FS.exists(file_path);
}

/*virtua*/ size_t FileSystem::read_file(const char* file_path, RNS::Bytes& data) {
	TRACEF("read_file: reading from file %s", file_path);
	size_t read = 0;
#if FS_TYPE == FS_TYPE_INTERNALFS || FS_TYPE == FS_TYPE_FLASHFS
	File file(FS);
	if (file.open(file_path, FILE_O_READ)) {
#else
	File file = FS.open(file_path, FILE_READ);
	if (file) {
#endif
		size_t size = file.size();
		read = file.readBytes((char*)data.writable(size), size);
		TRACEF("read_file: read %u bytes from file %s", read, file_path);
		if (read != size) {
			ERRORF("read_file: failed to read file %s", file_path);
            data.resize(read);
		}
		//TRACE("read_file: closing input file");
		file.close();
	}
	else {
		ERRORF("read_file: failed to open input file %s", file_path);
	}
    return read;
}

/*virtua*/ size_t FileSystem::write_file(const char* file_path, const RNS::Bytes& data) {
	TRACEF("write_file: writing to file %s", file_path);
	// CBA TODO Replace remove with working truncation
	if (FS.exists(file_path)) {
		FS.remove(file_path);
	}
	size_t wrote = 0;
#if FS_TYPE == FS_TYPE_INTERNALFS || FS_TYPE == FS_TYPE_FLASHFS
	File file(FS);
	if (file.open(file_path, FILE_O_WRITE)) {
#else
	File file = FS.open(file_path, FILE_WRITE);
	if (file) {
#endif
		// Seek to beginning to overwrite
		//file.seek(0);
		//file.truncate(0);
		wrote = file.write(data.data(), data.size());
        TRACEF("write_file: wrote %u bytes to file %s", wrote, file_path);
        if (wrote < data.size()) {
			WARNINGF("write_file: not all data was written to file %s", file_path);
		}
		//TRACE("write_file: closing output file");
		file.close();
	}
	else {
		ERRORF("write_file: failed to open output file %s", file_path);
	}
    return wrote;
}

/*virtual*/ RNS::FileStream FileSystem::open_file(const char* file_path, RNS::FileStream::MODE file_mode) {
	TRACEF("open_file: opening file %s", file_path);
#if FS_TYPE == FS_TYPE_INTERNALFS || FS_TYPE == FS_TYPE_FLASHFS
	int mode;
	if (file_mode == RNS::FileStream::MODE_READ) {
		mode = FILE_O_READ;
	}
	else if (file_mode == RNS::FileStream::MODE_WRITE) {
		mode = FILE_O_WRITE;
		// CBA TODO Replace remove with working truncation
		if (FS.exists(file_path)) {
			FS.remove(file_path);
		}
	}
	else if (file_mode == RNS::FileStream::MODE_APPEND) {
		// CBA This is the default write mode for nrf52 littlefs
		mode = FILE_O_WRITE;
	}
	else {
		ERRORF("open_file: unsupported mode %d", file_mode);
		return {RNS::Type::NONE};
	}
	File* file = new File(FS);
	RNS::FileStream stream(new FileStream(file));
	if (!file->open(file_path, mode)) {
		ERRORF("open_file: failed to open output file %s", file_path);
		return {RNS::Type::NONE};
	}
	// Seek to beginning to overwrite (this is failing on nrf52)
	//if (file_mode == RNS::FileStream::MODE_WRITE) {
	//	file->seek(0);
	//	file->truncate(0);
	//}
	TRACEF("open_file: successfully opened file %s", file_path);
	return stream;
#else
	const char* mode;
	if (file_mode == RNS::FileStream::MODE_READ) {
		mode = FILE_READ;
	}
	else if (file_mode == RNS::FileStream::MODE_WRITE) {
		mode = FILE_WRITE;
	}
	else if (file_mode == RNS::FileStream::MODE_APPEND) {
		mode = FILE_APPEND;
	}
	else {
		ERRORF("open_file: unsupported mode %d", file_mode);
		return {RNS::Type::NONE};
	}
	TRACEF("open_file: opening file %s in mode %s", file_path, mode);
	// CBA Using copy constructor to obtain File*
	File* file = new File(FS.open(file_path, mode));
	RNS::FileStream stream(new FileStream(file));
	if (file == nullptr || !(*file)) {
		ERRORF("open_file: failed to open output file %s", file_path);
		return {RNS::Type::NONE};
	}
	TRACEF("open_file: successfully opened file %s", file_path);
	return RNS::FileStream(new FileStream(file));
#endif
}

/*virtua*/ bool FileSystem::remove_file(const char* file_path) {
	TRACEF("remove_file: removing file %s", file_path);
	return FS.remove(file_path);
}

/*virtua*/ bool FileSystem::rename_file(const char* from_file_path, const char* to_file_path) {
	TRACEF("rename_file: renaming file %s to %s", from_file_path, to_file_path);
	return FS.rename(from_file_path, to_file_path);
}

/*virtua*/ bool FileSystem::directory_exists(const char* directory_path) {
	TRACEF("directory_exists: checking for existence of directory %s", directory_path);
#if FS_TYPE == FS_TYPE_INTERNALFS || FS_TYPE == FS_TYPE_FLASHFS
	File file(FS);
	if (file.open(directory_path, FILE_O_READ)) {
#else
	File file = FS.open(directory_path, FILE_READ);
	if (file) {
#endif
		bool is_directory = file.isDirectory();
		file.close();
		return is_directory;
	}
	return false;
}

/*virtua*/ bool FileSystem::create_directory(const char* directory_path) {
	TRACEF("create_directory: creating directory %s", directory_path);
	if (!FS.mkdir(directory_path)) {
		ERROR("create_directory: failed to create directory " + std::string(directory_path));
		return false;
	}
	return true;
}

/*virtua*/ bool FileSystem::remove_directory(const char* directory_path) {
	TRACEF("remove_directory: removing directory %s", directory_path);
#if FS_TYPE == FS_TYPE_INTERNALFS || FS_TYPE == FS_TYPE_FLASHFS
	if (!FS.rmdir_r(directory_path)) {
#else
	if (!FS.rmdir(directory_path)) {
#endif
		ERROR("remove_directory: failed to remove directory " + std::string(directory_path));
		return false;
	}
	return true;
}

/*virtua*/ std::list<std::string> FileSystem::list_directory(const char* directory_path) {
	TRACEF("list_directory: listing directory %s", directory_path);
	std::list<std::string> files;
	File root = FS.open(directory_path);
	if (!root) {
		ERROR("list_directory: failed to open directory " + std::string(directory_path));
		return files;
	}
	File file = root.openNextFile();
	while (file) {
		if (!file.isDirectory()) {
			char* name = (char*)file.name();
			files.push_back(name);
		}
		// CBA Following close required to avoid leaking memory
		file.close();
		file = root.openNextFile();
	}
	root.close();
	TRACE("list_directory: returning directory listing");
	return files;
}

/*virtual*/ size_t FileSystem::storage_size() {
#if FS_TYPE == FS_TYPE_INTERNALFS
	return totalBytes();
#else
	return FS.totalBytes();
#endif
}

/*virtual*/ size_t FileSystem::storage_available() {
#if FS_TYPE == FS_TYPE_INTERNALFS
	return (totalBytes() - usedBytes());
#else
	return (FS.totalBytes() - FS.usedBytes());
#endif
}

#endif
