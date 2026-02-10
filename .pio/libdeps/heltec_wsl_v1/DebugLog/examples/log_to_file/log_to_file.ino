// Uncommenting DEBUGLOG_DISABLE_LOG disables ASSERT and all log (Release Mode)
// PRINT and PRINTLN are always valid even in Release Mode
// #define DEBUGLOG_DISABLE_LOG

// You can also set default log level by defining macro (default: INFO)
// #define DEBUGLOG_DEFAULT_LOG_LEVEL_TRACE

// You can also set default file level by defining macro (default: ERROR)
// #define DEBUGLOG_DEFAULT_FILE_LEVEL_WARN

// if you want to use standard SD library
#include <SD.h>
#define fs SD

// If you want to use SdFat
// #include <SdFat.h>
// SdFat fs;
// SdFatSdio fs;

// If you want use SD_MMC (ESP) or other FileSystems
// #include <SD_MMC.h>
// #define fs SD_MMC

// If you want use SPIFFS (ESP32) or other FileSystems
// #include <SPIFFS.h>
// #define fs SPIFFS

// #include <LittleFS.h>
// TODO:

// #include <FatFs.h>
// TODO:

// define DEBUGLOG_ENABLE_FILE_LOGGER to enable file logger
#define DEBUGLOG_ENABLE_FILE_LOGGER
#include <DebugLog.h>

void shorten(String& s) {
    for (size_t i = 0; i < s.length(); ++i) {
        if (s[i] == ':')
            s.setCharAt(i, '_');
    }
}

void print_all_files() {
    PRINTLN("========== Read all files START ==========");
    File root = fs.open("/");
    while (true) {
        File entry = root.openNextFile();
        if (!entry) {
            PRINTLN("========== Read all files DONE ==========\n");
            break;
        }
        PRINTLN("=====", entry.name(), "=====");
        if (entry.isDirectory()) {
            ;  // ignore directory
        } else {
            PRINTLN("File size =", entry.available(), "bytes");
            while (entry.available() > 0) {
                PRINT((char)entry.read());
            }
            PRINTLN();
        }
        entry.close();
    }
}

void setup() {
    Serial.begin(115200);
    delay(2000);

    if (fs.begin()) {
        PRINTLN("FileSystem initialization success");

        print_all_files();

        String filename = "/" + String(__TIME__) + ".txt";
        shorten(filename);

        // Set file system to save every log automatically
        LOG_ATTACH_FS_AUTO(fs, filename, FILE_WRITE);  // overwrite file
        // LOG_ATTACH_FS_AUTO(fs, filename, FILE_APPEND);  // append to file

    } else {
        ASSERTM(false, "FileSystem initialization failed!");
    }

    // PRINT_FILE and PRINTLN_FILE is not affected by file_level (always visible)
    // PRINT_FILE and PRINTLN_FILE is not displayed to Serial
    PRINT_FILE("DebugLog", "can print variable args: ");
    PRINTLN_FILE(1, 2.2, "three", "=> like this");

    // Apart from the log level to be displayed,
    // you can set the log level to be saved to a file (Default is DebugLogLevel::LVL_ERROR)
    // LOG_FILE_SET_LEVEL(DebugLogLevel::LVL_INFO);

    // The default log_leval is DebugLogLevel::LVL_INFO
    // 0: NONE, 1: ERROR, 2: WARN, 3: INFO, 4: DEBUG, 5: TRACE
    PRINTLN_FILE("current log level is", (int)LOG_FILE_GET_LEVEL());

    // LOG_XXXX outpus both Serial and File based on log_level and file_level
    // The default log_leval is DebugLogLevel::LVL_INFO
    // The default file_leval is DebugLogLevel::LVL_ERROR
    LOG_ERROR("this is error log");  // printed to both Serial and File
    LOG_WARN("this is warn log");    // won't be saved but printed
    LOG_INFO("this is info log");    // won't be saved but printed
    LOG_DEBUG("this is debug log");  // won't be printed
    LOG_TRACE("this is trace log");  // won't be printed

    // Log array
    float arr[3] {1.1, 2.2, 3.3};
    PRINTLN_FILE("Array can be also printed like this", LOG_AS_ARR(arr, 3));

#if ARX_HAVE_LIBSTDCPLUSPLUS >= 201103L  // Have libstdc++11
    // Log containers
    std::vector<int> vs {1, 2, 3};
    std::deque<float> ds {1.1, 2.2, 3.3};
    std::map<String, int> ms {{"one", 1}, {"two", 2}, {"three", 3}};
    PRINTLN_FILE("Containers can also be printed like", vs, ds, ms);
#endif

    delay(1000);

    // You can also use assert
    // If assertion failed, suspend program after prints message and close files
    int x = 1;
    // ASSERT(x != 1);
    // You can also use assert with messages by ASSERTM macro
    ASSERTM(x != 1, "This always fails");
}

void loop() {
    if (LOG_FILE_IS_OPEN()) {
        LOG_FILE_CLOSE();
    }
    PRINTLN("If DEBUGLOG_DISABLE_LOG is commented out (assert is enabled), does not come here");
    delay(1000);
}
