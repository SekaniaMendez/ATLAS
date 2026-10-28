#include <atlas/storage/PagedVoxelMap.hpp>
#include <atlas/storage/VoxelMapArchive.hpp>
#include "BinaryIO.hpp"
#include <sqlite3.h>
#include <list>
#include <sstream>
#include <unordered_map>

namespace atlas::storage {
namespace {
using namespace atlas::slam;
void check(int code, sqlite3* db) {
  if (code != SQLITE_OK && code != SQLITE_DONE && code != SQLITE_ROW)
    throw std::runtime_error(std::string("Map database: ") + sqlite3_errmsg(db));
}
struct Statement {
  sqlite3* db;
  sqlite3_stmt* stmt = nullptr;
  Statement(sqlite3* database, const char* sql) : db(database) {
    check(sqlite3_prepare_v2(db, sql, -1, &stmt, nullptr), db);
  }
  ~Statement() { sqlite3_finalize(stmt); }
  void key(const BlockKey& key) {
    check(sqlite3_bind_int(stmt, 1, key.x), db); check(sqlite3_bind_int(stmt, 2, key.y), db);
    check(sqlite3_bind_int(stmt, 3, key.z), db);
  }
  void blob(int index, const std::string& bytes) {
    if (bytes.size() > binary::kMaxRecordBytes) throw std::runtime_error("Map block too large");
    check(sqlite3_bind_blob(stmt, index, bytes.data(), static_cast<int>(bytes.size()), SQLITE_TRANSIENT), db);
  }
  int step() { const auto result = sqlite3_step(stmt); check(result, db); return result; }
  std::string bytes() {
    const auto size = sqlite3_column_bytes(stmt, 0);
    if (size <= 0 || static_cast<size_t>(size) > binary::kMaxRecordBytes) throw std::runtime_error("Invalid map database record");
    return {static_cast<const char*>(sqlite3_column_blob(stmt, 0)), static_cast<size_t>(size)};
  }
};
std::string serialize(const SparseVoxelMap& map) {
  std::ostringstream output(std::ios::binary | std::ios::out);
  VoxelMapArchive::write(output, map); return output.str();
}
SparseVoxelMap deserialize(const std::string& bytes, size_t budget) {
  std::istringstream input(bytes, std::ios::binary | std::ios::in);
  return VoxelMapArchive::read(input, budget);
}
struct WriterLock {
  std::filesystem::path path;
  explicit WriterLock(std::filesystem::path value) : path(std::move(value)) {
    binary::create_new_directory(path);
  }
  ~WriterLock() { std::error_code ignored; std::filesystem::remove(path, ignored); }
};
}  // namespace

size_t PagedVoxelMap::block_payload_reservation(size_t samples) {
  // vector capacity growth may round upward. Charge twice the configured
  // capacity and fixed container/index allowance, even for sparsely hit blocks.
  constexpr size_t base = sizeof(VoxelBlock) + 4096;
  if (samples == 0 || samples > (binary::kMaxRecordBytes - base) / (512 * 2 * sizeof(MapPoint)))
    throw std::invalid_argument("Paged map sample capacity exceeds block limit");
  return base + 512 * 2 * samples * sizeof(MapPoint);
}

class PagedVoxelMap::Impl {
 public:
  struct Entry { BlockKey key; SparseVoxelMap map; bool dirty = false; };
  using Entries = std::list<Entry>;
  Impl(const std::filesystem::path& directory, Mode mode, SparseVoxelMapConfig config, size_t budget)
      : coordinates_(config), db_(nullptr, sqlite3_close) {
    if (mode == Mode::Create) binary::create_new_directory(directory);
    else if (!std::filesystem::is_directory(directory)) throw std::runtime_error("Map directory does not exist");
    lock_ = std::make_unique<WriterLock>(directory / ".writer-lock");
    sqlite3* raw = nullptr;
    const auto code = sqlite3_open_v2((directory / "blocks.sqlite").string().c_str(), &raw,
        SQLITE_OPEN_READWRITE | (mode == Mode::Create ? SQLITE_OPEN_CREATE : 0), nullptr);
    db_.reset(raw); check(code, db_.get());
    sqlite3_limit(db_.get(), SQLITE_LIMIT_LENGTH, static_cast<int>(binary::kMaxRecordBytes));
    exec("PRAGMA cache_size=-1024; PRAGMA mmap_size=0; PRAGMA synchronous=FULL;");
    if (mode == Mode::Create) {
      exec("CREATE TABLE metadata(id INTEGER PRIMARY KEY CHECK(id=1), data BLOB NOT NULL);"
           "CREATE TABLE blocks(x INTEGER,y INTEGER,z INTEGER,data BLOB NOT NULL,PRIMARY KEY(x,y,z)) WITHOUT ROWID;");
      save_metadata(coordinates_);
    } else {
      Statement query(db_.get(), "SELECT data FROM metadata WHERE id=1");
      if (query.step() != SQLITE_ROW) throw std::runtime_error("Missing map metadata");
      coordinates_ = deserialize(query.bytes(), binary::kMaxRecordBytes);
      if (coordinates_.block_count()) throw std::runtime_error("Invalid map metadata geometry");
    }
    const auto reservation = block_payload_reservation(coordinates_.config().max_samples_per_voxel);
    if (budget < reservation) throw std::invalid_argument("Map payload budget cannot hold one full block");
    stats_.payload_budget_bytes = budget; stats_.resident_block_limit = budget / reservation;
    reservation_ = reservation;
  }
  ~Impl() { try { flush(); } catch (...) {} }
  void exec(const char* sql) { check(sqlite3_exec(db_.get(), sql, nullptr, nullptr, nullptr), db_.get()); }
  void save_metadata(const SparseVoxelMap& map) {
    Statement q(db_.get(), "INSERT OR REPLACE INTO metadata(id,data) VALUES(1,?)");
    q.blob(1, serialize(map)); q.step();
  }
  void save(const Entry& entry) {
    Statement q(db_.get(), "INSERT OR REPLACE INTO blocks(x,y,z,data) VALUES(?,?,?,?)");
    q.key(entry.key); q.blob(4, serialize(entry.map)); q.step();
  }
  Entry* access(const BlockKey& key, bool create) {
    auto cached = index_.find(key);
    if (cached != index_.end()) { entries_.splice(entries_.begin(), entries_, cached->second); return &entries_.front(); }
    std::optional<std::string> bytes;
    { Statement query(db_.get(), "SELECT data FROM blocks WHERE x=? AND y=? AND z=?");
      query.key(key); if (query.step() == SQLITE_ROW) bytes = query.bytes(); }
    if (!bytes && !create) return nullptr;
    if (entries_.size() == stats_.resident_block_limit) {
      auto& victim = entries_.back();
      if (victim.dirty) { save(victim); ++stats_.writes; }
      index_.erase(victim.key); entries_.pop_back(); ++stats_.evictions;
    }
    SparseVoxelMap map = bytes ? deserialize(*bytes, reservation_) : SparseVoxelMap(coordinates_.config());
    if (bytes) {
      if (map.block_count() != 1 || !map.find_block(key) ||
          map.config().voxel_size_m != coordinates_.config().voxel_size_m ||
          map.config().max_samples_per_voxel != coordinates_.config().max_samples_per_voxel)
        throw std::runtime_error("Map block/config mismatch");
      map.set_bounds(coordinates_.config().bounds); ++stats_.loads;
    }
    entries_.push_front({key, std::move(map), false});
    try { index_.emplace(key, entries_.begin()); } catch (...) { entries_.pop_front(); throw; }
    return &entries_.front();
  }
  void flush() {
    if (entries_.empty()) return;
    exec("BEGIN IMMEDIATE");
    try { for (const auto& e : entries_) if (e.dirty) save(e); exec("COMMIT"); }
    catch (...) { sqlite3_exec(db_.get(), "ROLLBACK", nullptr, nullptr, nullptr); throw; }
    for (auto& e : entries_) if (e.dirty) { e.dirty = false; ++stats_.writes; }
  }
  const SparseVoxelMapConfig& config() const { return coordinates_.config(); }
  VoxelInsertResult insert(const MapPoint& point) {
    const auto key = coordinates_.voxel_key_for(point);
    // Use the same rejection precedence and validation as the in-memory map.
    if (!key || (config().bounds && !config().bounds->contains(point))) return coordinates_.insert(point);
    auto* entry = access(SparseVoxelMap::address_for(*key).block, true);
    entry->dirty = true;
    return entry->map.insert(point);
  }
  std::optional<Voxel> get(const VoxelKey& key) {
    auto* entry = access(SparseVoxelMap::address_for(key).block, false);
    if (!entry) return std::nullopt;
    const auto* voxel = entry->map.find(key);
    return voxel ? std::optional<Voxel>(*voxel) : std::nullopt;
  }
  void set_bounds(std::optional<MapBounds> bounds) {
    SparseVoxelMap next(coordinates_.config()); next.set_bounds(bounds);
    flush(); save_metadata(next); coordinates_ = std::move(next);
    for (auto& entry : entries_) entry.map.set_bounds(bounds);
  }
  PagedMapStats stats() const {
    auto result = stats_; result.resident_blocks = entries_.size();
    result.reserved_payload_bytes = entries_.size() * reservation_; return result;
  }
 private:
  SparseVoxelMap coordinates_;
  std::unique_ptr<WriterLock> lock_;
  std::unique_ptr<sqlite3, decltype(&sqlite3_close)> db_;
  Entries entries_;
  std::unordered_map<BlockKey, Entries::iterator, BlockKeyHash> index_;
  PagedMapStats stats_;
  size_t reservation_ = 0;
};

PagedVoxelMap::PagedVoxelMap(const std::filesystem::path& path, Mode mode, SparseVoxelMapConfig config, size_t budget)
    : impl_(std::make_unique<Impl>(path, mode, config, budget)) {}
PagedVoxelMap::~PagedVoxelMap() = default;
const SparseVoxelMapConfig& PagedVoxelMap::config() const { return impl_->config(); }
slam::VoxelInsertResult PagedVoxelMap::insert(const slam::MapPoint& point) { return impl_->insert(point); }
std::optional<slam::Voxel> PagedVoxelMap::get_voxel(const slam::VoxelKey& key) { return impl_->get(key); }
void PagedVoxelMap::set_bounds(std::optional<slam::MapBounds> bounds) { impl_->set_bounds(bounds); }
void PagedVoxelMap::flush() { impl_->flush(); }
PagedMapStats PagedVoxelMap::stats() const { return impl_->stats(); }
}  // namespace atlas::storage
