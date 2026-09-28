// Host-side companion tool, which opens the rover's Bluetooth SPP link (/dev/rfcomm0 on Linux) or a USB-serial bridge and logs every TELEMETRY_DATA packet plus any NACK/CRC fault into a SQLite database for offline validation.

#include "Protocol.h"

#include <sqlite3.h>

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>

using namespace rover_proto;

namespace {

// opens and configures a POSIX serial device for raw 115200 8N1
int openSerialPort(const char* path) {
    int fd = open(path, O_RDWR | O_NOCTTY);
    if (fd < 0) {
        perror("open serial device");
        return -1;
    }

    termios tty{};
    if (tcgetattr(fd, &tty) != 0) {
        perror("tcgetattr");
        close(fd);
        return -1;
    }

    cfsetospeed(&tty, B115200);
    cfsetispeed(&tty, B115200);

    tty.c_cflag &= ~PARENB;   // no parity
    tty.c_cflag &= ~CSTOPB;   // 1 stop bit
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;       // 8 data bits
    tty.c_cflag &= ~CRTSCTS;  // no hardware flow control
    tty.c_cflag |= CREAD | CLOCAL;

    cfmakeraw(&tty); // raw byte-for-byte, no line-discipline munging

    tty.c_cc[VMIN]  = 0; // non-blocking-ish reads
    tty.c_cc[VTIME] = 5; // 0.5s read timeout

    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        perror("tcsetattr");
        close(fd);
        return -1;
    }
    return fd;
}

// small RAII-ish wrapper so every call site doesn't repeat SQLite error handling
struct Db {
    sqlite3* handle = nullptr;

    bool open(const char* path) {
        return sqlite3_open(path, &handle) == SQLITE_OK;
    }

    bool exec(const char* sql) {
        char* errMsg = nullptr;
        const int rc = sqlite3_exec(handle, sql, nullptr, nullptr, &errMsg);
        if (rc != SQLITE_OK) {
            fprintf(stderr, "SQL error: %s\n", errMsg ? errMsg : "(unknown)");
            sqlite3_free(errMsg);
            return false;
        }
        return true;
    }

    ~Db() { if (handle) sqlite3_close(handle); }
};

// applies schema.sql; every statement in it is CREATE ... IF NOT EXISTS, so re-running this against an existing database is always safe
bool applySchema(Db& db, const char* schemaPath) {
    FILE* f = fopen(schemaPath, "rb");
    if (!f) {
        fprintf(stderr, "warning: could not open %s, assuming schema already applied\n", schemaPath);
        return true; // not fatal -> DB may already have the schema from a prior run
    }
    fseek(f, 0, SEEK_END);
    const long size = ftell(f);
    fseek(f, 0, SEEK_SET);
    std::string sql(static_cast<size_t>(size), '\0');
    if (fread(sql.data(), 1, static_cast<size_t>(size), f) != static_cast<size_t>(size)) {
        fclose(f);
        fprintf(stderr, "error reading %s\n", schemaPath);
        return false;
    }
    fclose(f);
    return db.exec(sql.c_str());
}

long long insertSession(Db& db, const char* notes) {
    sqlite3_stmt* stmt = nullptr;
    sqlite3_prepare_v2(db.handle, "INSERT INTO sessions (notes) VALUES (?);", -1, &stmt, nullptr);
    sqlite3_bind_text(stmt, 1, notes, -1, SQLITE_TRANSIENT);
    sqlite3_step(stmt);
    sqlite3_finalize(stmt);
    return sqlite3_last_insert_rowid(db.handle);
}

// unpacks TELEMETRY_DATA payload (layout defined in CommandProcessor::sendTelemetry) and inserts one row
void logTelemetryPacket(Db& db, long long sessionId, const Packet& pkt) {
    if (pkt.length != 60) {
        fprintf(stderr, "unexpected telemetry payload length %u, skipping\n", pkt.length);
        return;
    }

    size_t off = 0;
    auto readU32 = [&]() { uint32_t v; memcpy(&v, &pkt.payload[off], 4); off += 4; return v; };
    auto readF32 = [&]() { float v; memcpy(&v, &pkt.payload[off], 4); off += 4; return v; };

    const uint32_t roverMillis = readU32();
    const float voltage = readF32();
    const float comp = readF32();

    float wheelVals[12]; // FL(target,actual,cmd), RL(...), FR(...), RR(...)
    for (float& v : wheelVals) v = readF32();

    static const char* sql =
        "INSERT INTO telemetry_samples ("
        " session_id, rover_millis, battery_voltage, comp_factor,"
        " fl_target_radps, fl_actual_radps, fl_cmd,"
        " rl_target_radps, rl_actual_radps, rl_cmd,"
        " fr_target_radps, fr_actual_radps, fr_cmd,"
        " rr_target_radps, rr_actual_radps, rr_cmd"
        ") VALUES (?,?,?,?, ?,?,?, ?,?,?, ?,?,?, ?,?,?);";

    sqlite3_stmt* stmt = nullptr;
    sqlite3_prepare_v2(db.handle, sql, -1, &stmt, nullptr);
    sqlite3_bind_int64(stmt, 1, sessionId);
    sqlite3_bind_int64(stmt, 2, roverMillis);
    sqlite3_bind_double(stmt, 3, voltage);
    sqlite3_bind_double(stmt, 4, comp);
    for (int i = 0; i < 12; ++i) {
        sqlite3_bind_double(stmt, 5 + i, wheelVals[i]);
    }
    if (sqlite3_step(stmt) != SQLITE_DONE) {
        fprintf(stderr, "insert failed: %s\n", sqlite3_errmsg(db.handle));
    }
    sqlite3_finalize(stmt);
}

void logFaultEvent(Db& db, long long sessionId, const char* type, const char* detail) {
    sqlite3_stmt* stmt = nullptr;
    sqlite3_prepare_v2(db.handle,
        "INSERT INTO fault_events (session_id, event_type, detail) VALUES (?,?,?);",
        -1, &stmt, nullptr);
    sqlite3_bind_int64(stmt, 1, sessionId);
    sqlite3_bind_text(stmt, 2, type, -1, SQLITE_TRANSIENT);
    sqlite3_bind_text(stmt, 3, detail, -1, SQLITE_TRANSIENT);
    sqlite3_step(stmt);
    sqlite3_finalize(stmt);
}

}

int main(int argc, char** argv) {
    if (argc < 3) {
        fprintf(stderr, "usage: %s <serial_device> <sqlite_db_path> [\"session notes\"]\n", argv[0]);
        return 1;
    }
    const char* serialPath = argv[1];
    const char* dbPath = argv[2];
    const char* notes = argc >= 4 ? argv[3] : "";

    Db db;
    if (!db.open(dbPath)) {
        fprintf(stderr, "failed to open database %s\n", dbPath);
        return 1;
    }
    if (!applySchema(db, "schema.sql")) return 1;

    const long long sessionId = insertSession(db, notes);
    printf("Logging session %lld to %s (Ctrl+C to stop)\n", sessionId, dbPath);

    const int fd = openSerialPort(serialPath);
    if (fd < 0) return 1;

    FrameParser parser;
    Packet pkt;
    uint8_t byte;
    uint32_t crcErrorsSeen = 0;

    while (true) {
        const ssize_t n = read(fd, &byte, 1);
        if (n <= 0) continue; // read timeout, loop again (also where Ctrl+C naturally interrupts)

        if (parser.feed(byte, pkt)) {
            switch (static_cast<CommandId>(pkt.cmd)) {
                case CommandId::TELEMETRY_DATA:
                    logTelemetryPacket(db, sessionId, pkt);
                    break;
                case CommandId::NACK: {
                    char detail[64];
                    snprintf(detail, sizeof(detail), "cmd=0x%02X reason=0x%02X", pkt.payload[0], pkt.payload[1]);
                    logFaultEvent(db, sessionId, "NACK", detail);
                    break;
                }
                default:
                    break; // ACK/heartbeat etc. aren't individually logged
            }
        }

        // track CRC errors as they accumulate on the parser
        if (parser.crcErrorCount() != crcErrorsSeen) {
            crcErrorsSeen = parser.crcErrorCount();
            logFaultEvent(db, sessionId, "CRC_ERROR", "frame dropped");
        }
    }

    close(fd);
    return 0;
} // namespace
