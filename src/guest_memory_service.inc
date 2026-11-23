// SPDX-License-Identifier: GPL-3.0-or-later
// ---- The guest's memory as a service: shared until written (included by memory_pages.cpp) ----
//
// After Gregory Price's "A Compressed RAM Service" (Linux Plumbers 2026, mm/cram.c): memory that
// is read where it lies and promoted when it is written. There the device compresses; here the
// emulator is the device, and the two forms a processor can read without a fault are a slot that
// shares the block of zeroes and a slot that shares a block with another slot of the same bytes.
//
//   Zero    the slot shows the shared block of zeroes, read-only
//   Own     the slot has a block of its own, read-write
//   Shared  the slot shows a block it shares with other slots of the same bytes, read-only
//   Fine    the slot is 128 pages of 16 KiB, read-only: its pages of zeroes show the zero block, the
//           others a compact block that holds only them (a written slot with many pages of zeroes);
//           a write makes it whole again, and it waits longer each time before it may be Fine again
//
//   Packed  the slot has no block: its data is LZ4-compressed in the packed store, or in the swap file
//           on the console's storage ("Swap" below); any access unpacks it
//   Idle    the slot keeps its block but allows no access: the first access of any kind wakes it. How
//           the scanner learns which slots nothing reads or writes; only those are packed or swapped
//   Clean   the slot has its block back from the swap file, read-only, and the file still holds the
//           same bytes: giving the block up again costs no write to the storage. A write makes it Own
//
// A write to a Zero or Shared slot faults; HandleGuestWriteFault gives the slot a block of its
// own (zeroed, or a copy) and the write repeats. So memory is no longer taken when the guest's
// kernel hands pages out, only when they are written; a slot the guest's kernel clears or frees
// whole goes back to the zero block; and a scanner gives back slots that hold only zeroes and
// lets slots of the same bytes share one block. A slot is locked (bit 7 of its state) while it
// changes; a writer that faults on a locked slot waits for it.
//
// 64-bit games only: a 32-bit game's fastmem window aliases the backing, which takes it whole.

namespace {
enum : std::uint8_t { SlotZero = 0, SlotOwn = 1, SlotShared = 2, SlotFine = 3, SlotPacked = 4, SlotIdle = 5, SlotClean = 6, SlotLock = 0x80, SlotMask = 0x7f };
constexpr std::size_t FinePage = 16384, FinePages = SparseSlot / FinePage;
// A written slot becomes Fine with at least this many pages of zeroes (512 KiB given back).
constexpr std::size_t FineMinZeroPages = 32;
// A slot made whole again by a write waits 1, 2, 4 ... 1024 scan passes before it may be Fine again.
constexpr unsigned FineBackoffMax = 10;

// Duplicate sharing is off unless asked for (dev-settings cram_dedup=on): on the console it found a
// dozen 2 MiB pairs that were written apart within a minute, and with it on a large game held 20 FPS
// instead of 30 (the GPU worker waited for every fence, once a frame) where it held 30 without.
std::atomic<bool> service_requested{true}, scan_requested{true}, dedup_requested{false}, fine_requested{true};
std::atomic<SparseRange*> service_range{nullptr}; // the guest range the service runs for
std::atomic<SparseRange*> probe_range{nullptr};   // the self-test's range
std::atomic<int> service_state{0};                // 0 not tested, 1 works here, 2 does not
const char* service_refused = "";

struct Counters {
    std::atomic<std::uint64_t> faults{0}, promotions{0}, copies{0}, waits{0};
    std::atomic<std::uint64_t> returns_clear{0}, returns_free{0}, returns_scan{0}, dedup{0};
    std::atomic<std::uint64_t> reserve_used{0}, host_reserve_used{0}, failures{0}, passes{0}, scan_ns{0};
    std::atomic<std::uint64_t> fined{0}, unfined{0}, fine_failures{0};
};
Counters counters;

// ---- The 64-bit fastmem window's aliases (src/host_memory.cpp, Window64; backported from ProsperoEden) ----
// A window region (2 MiB) shows a slot's block from a JIT fault until the slot's block changes. Every
// change of a slot's block or protection (a promotion, a return, a scan's fence) first drops its
// aliases, under the slot's lock: a stale alias would reach memory the slot gave up. An entry is the
// window address, plus AliasReadOnly when it shows the block read-only, or AliasSmall when it shows
// 16 KiB pieces of it rather than the whole block.
constexpr unsigned AliasesPerSlot = 4;
constexpr std::uintptr_t AliasReadOnly = 1, AliasSmall = 2, AliasFlags = AliasReadOnly | AliasSmall;
std::atomic<std::uint64_t> aliases_made{0}, aliases_dropped{0}, aliases_refused{0};
// The window region at `at` shows nothing again: a reservation straight over the mapping.
void VoidWindowRegion([[maybe_unused]] std::uintptr_t at) noexcept {
#ifdef PS5_NATIVE
    void* address = reinterpret_cast<void*>(at);
    if (sceKernelReserveVirtualRange(&address, SparseSlot, MAP_FIXED, SparseSlot) == 0 && address == reinterpret_cast<void*>(at))
        return;
    address = reinterpret_cast<void*>(at);
    if (munmap(address, SparseSlot) != 0 || sceKernelReserveVirtualRange(&address, SparseSlot, MAP_FIXED, SparseSlot) != 0 ||
        address != reinterpret_cast<void*>(at))
        std::abort(); // a window region left mapped would expose memory the slot gives up
#else
    if (mmap(reinterpret_cast<void*>(at), SparseSlot, PROT_NONE, MAP_PRIVATE | MAP_ANONYMOUS | MAP_FIXED | MAP_NORESERVE, -1, 0) ==
        MAP_FAILED)
        std::abort();
#endif
}
// Slot locked.
void DropSlotAliases(SparseRange& range, std::size_t slot) noexcept {
    if (!range.guest || !range.aliases) return;
    for (unsigned i = 0; i < AliasesPerSlot; ++i)
        if (auto& alias = range.aliases[slot * AliasesPerSlot + i]) {
            VoidWindowRegion(alias & ~AliasFlags);
            alias = 0;
            aliases_dropped.fetch_add(1, std::memory_order_relaxed);
        }
}
void ResetCounters() noexcept {
    for (auto* counter : {&counters.faults, &counters.promotions, &counters.copies, &counters.waits,
                          &counters.returns_clear, &counters.returns_free, &counters.returns_scan, &counters.dedup,
                          &counters.reserve_used, &counters.host_reserve_used, &counters.failures, &counters.passes, &counters.scan_ns,
                          &counters.fined, &counters.unfined, &counters.fine_failures})
        counter->store(0, std::memory_order_relaxed);
}

// Freed pages cleared (GuestMemoryFreedPart, below).
std::atomic<std::uint64_t> freed_part_calls{0}, freed_part_bytes{0};
// Idle blocks RecoverMemory gave back (GiveBackIdleBlocks).
std::atomic<std::uint64_t> idle_given_back{0}, idle_give_back_calls{0};
// What the keeper had given back before an allocation failed (KeepFree).
std::atomic<std::uint64_t> keeper_calls{0}, keeper_bytes{0};

bool LockSlot(SparseRange& range, std::size_t slot, bool count_wait = false) noexcept {
    auto& state = range.state[slot];
    bool waited = false;
    for (;;) {
        std::uint8_t value = state.load(std::memory_order_relaxed);
        if (!(value & SlotLock) &&
            state.compare_exchange_weak(value, value | SlotLock, std::memory_order_acquire, std::memory_order_relaxed))
            break;
        waited = true;
        sched_yield();
    }
    if (waited && count_wait) counters.waits.fetch_add(1, std::memory_order_relaxed);
    return waited;
}
// The lock, if the slot is not locked now. For whoever holds pack_mutex: a fault handler keeps its
// slot locked while it waits for a block, and asks for pack_mutex meanwhile to have memory given
// back (WaitForBlock). Waiting for that slot here would hold both up until the handler gave up,
// 50 s later, with its access refused. A locked slot is in use: it is left for another time.
bool TryLockSlot(SparseRange& range, std::size_t slot) noexcept {
    auto& state = range.state[slot];
    std::uint8_t value = state.load(std::memory_order_relaxed);
    return !(value & SlotLock) &&
           state.compare_exchange_strong(value, value | SlotLock, std::memory_order_acquire, std::memory_order_relaxed);
}
std::size_t FineKept(SparseRange& range, std::size_t slot) noexcept {
    return range.fine[slot] & 0xff;
}
std::uint8_t SlotState(SparseRange& range, std::size_t slot) noexcept {
    return range.state[slot].load(std::memory_order_acquire) & SlotMask;
}
// Sets the state and lets the slot go.
void UnlockSlot(SparseRange& range, std::size_t slot, std::uint8_t to) noexcept {
    // Packed: no block. Idle and Clean: a block that may not be written yet (CommitSparsePage must
    // come through ServiceCommit).
    range.owned[slot].store(to != SlotZero && to != SlotPacked && to != SlotIdle && to != SlotClean, std::memory_order_release);
    range.state[slot].store(to, std::memory_order_release);
}
std::uintptr_t SlotAddress(SparseRange& range, std::size_t slot) noexcept {
    return range.begin.load(std::memory_order_relaxed) + slot * SparseSlot;
}

// ---- Blocks: the pool, a reserve for when the pool says no, and shared blocks' references ----
//
// The reserve is the guest's headroom in the pool, held as blocks of its own: 1 GiB that a game's
// writes take when the pool has no block left. It used to be 32 MiB here, plus 1 GiB that the
// graphics caches only counted as taken (headless/performance.cpp): that 1 GiB stayed in the pool,
// where Eden's heap, the driver and the caches themselves could take it, and where 2 MiB blocks
// could no longer be cut from what was left. Tears of the Kingdom's writes found no block four
// times with the pool's largest free block at 1 or 2 MiB. Held here, nothing else can take it or
// cut it up; the caches see the pool as it is, less what is not held yet, and evict as soon as a
// burst of writes has used half of it (performance.h ViewOfPool). It is topped up every 100 ms
// (ReserveKeeper) and given back with the guest's memory when the session ends.
constexpr std::size_t ReserveBlocks = 256;      // 512 MiB (diagnostic62 shared-pool test)
constexpr std::size_t ReserveAtStart = 16;      // taken as the service starts; the keeper takes the rest
std::int64_t reserve[ReserveBlocks];
bool PackEnabled() noexcept; // packing (below)
// The reserve kept, in blocks: EDEN_CRAM_RESERVE_MIB (up to 512), else 128 MiB while packing runs
// (packing gives blocks back when the pool is short; MKW used none of the 512 MiB), else 512 MiB.
std::size_t ReserveTarget() noexcept {
    static const std::size_t blocks = [] {
        const char* value = std::getenv("EDEN_CRAM_RESERVE_MIB");
        const std::size_t mib = value != nullptr ? std::strtoull(value, nullptr, 10) : PackEnabled() ? 128 : 512;
        return std::max<std::size_t>(1, std::min(ReserveBlocks, mib / (SparseSlot >> 20)));
    }();
    return blocks;
}
std::atomic<std::size_t> reserve_count{0};
std::atomic_flag reserve_lock = ATOMIC_FLAG_INIT;
constexpr std::size_t SharedBlocks = 4096;
std::int64_t shared_block[SharedBlocks];
int shared_refs[SharedBlocks];
std::atomic_flag shared_lock = ATOMIC_FLAG_INIT;

void Spin(std::atomic_flag& flag) noexcept {
    while (flag.test_and_set(std::memory_order_acquire)) sched_yield();
}

// ---- Packed slots: an Own slot unwritten for a while, LZ4-compressed while the pool is short ----
// The slot maps the zero block with no access and its block goes back; the next access of any
// kind faults, and the fault handler gives it a block and unpacks it there (UnpackSlot). Packed
// data lives in the heap. A buffer the fault handler is done with is freed by the scanner (no
// free in a signal handler): it goes on a list through its first word.
struct PackedSlot {
    void* data = nullptr;       // in the packed store (memory); null when the data is in the swap file
    std::uint32_t size = 0;     // bytes stored (in the swap file, SparseSlot: stored as it was)
    std::uint32_t swap = 0;     // the swap file unit it starts at, + 1; 0: not in the file
    std::uint64_t hash = 0;     // swap file only: FullHash of the slot's 2 MiB, checked when read back
};
std::atomic<PackedSlot*> packed_slots{nullptr}; // per slot of the guest's range, made by the scanner
// Diagnostic (EDEN_CRAM_PACK_KEEP=1): a packed slot keeps its block, and the unpack compares what it
// unpacked with it. A difference means the slot's data changed while it was packed (written through a
// mapping other than the slot's own). Saves no memory: for finding what packing breaks.
bool PackKeep() noexcept {
    static const bool on = [] {
        const char* value = std::getenv("EDEN_CRAM_PACK_KEEP");
        return value != nullptr && value[0] == '1';
    }();
    return on;
}
std::int64_t* pack_kept = nullptr; // per slot: the block a packed slot kept, -1 none
// Pages of each slot the GPU's caches hold (eden_guest_gpu_cached, from the core's device memory
// manager): such a slot is not packed. With packing, Mario Kart World's emulated GPU read impossible
// draw arguments (1.8-2.9 billion indices) and the GPU hung, only in sessions where the GPU thread
// itself touched packed slots; the slots' data came back intact, so what the GPU reads through its
// caches and their ordering must not meet a packed slot. That left MKW ~86 MiB to pack (it maps
// nearly all its memory for the GPU), and it still ran out; the impossible draws come without
// packing too. So such slots are packed by default; EDEN_CRAM_PACK_GPU=0 keeps them out.
std::atomic<std::atomic<std::int32_t>*> gpu_cached_table{nullptr};
std::size_t gpu_cached_slots = 0;
std::atomic<std::uint64_t> pack_gpu_skips{0}, pack_gpu_marked{0};
// Slots a GPU thread (the command reader "GPU", or "GPUBack") ever had to unpack: never packed again.
// The caches' count misses what the command reader reads (command lists, draw arguments), and one
// such unpack was enough for impossible draws.
std::atomic<std::uint8_t>* gpu_touched = nullptr;
// Diagnostic: which threads touched packed slots (unpacks by thread name, 16 names at most).
struct Unpacker {
    char name[24];
    std::atomic<std::uint32_t> count;
};
Unpacker unpackers[16];
std::atomic_flag unpackers_lock = ATOMIC_FLAG_INIT;
void CountUnpacker(std::size_t slot) noexcept;
std::atomic<std::uint64_t> pack_keep_compared{0}, pack_keep_differed{0};
std::atomic<std::uint64_t> packed_count{0}, packed_bytes{0}, packs{0}, unpacks{0}, pack_skips{0};

// The packed store: packed data lives in 2 MiB blocks of direct memory of its own, filled one after
// another, outside Eden's heap. Packing then takes no heap memory, so it can run where the heap
// itself found no memory (eden_memory_recover_try), and leaves no slack in the heap. A block whose
// data all went back is given back by the scanner (StoreReclaim). When the pool has no block for
// it, the slot being packed gives its own. Taken and filled under pack_mutex; StoreRelease (the
// fault handler) only counts.
constexpr std::size_t StoreBlock = SparseSlot, StoreBlocks = 2048; // up to 4 GiB of packed data
std::uintptr_t store_base = 0;
std::int64_t store_physical[StoreBlocks];
std::uint32_t store_used[StoreBlocks];
std::atomic<std::uint32_t> store_live[StoreBlocks];
std::size_t store_current = StoreBlocks;
std::atomic<std::size_t> store_held{0};

#ifdef PS5_NATIVE
bool StoreStart() noexcept {
    if (store_base != 0) return true;
    void* address = reinterpret_cast<void*>(cpu_mapping_hint);
    if (sceKernelReserveVirtualRange(&address, StoreBlock * StoreBlocks, 0, StoreBlock) != 0) return false;
    store_base = reinterpret_cast<std::uintptr_t>(address);
    for (std::size_t i = 0; i < StoreBlocks; ++i) store_physical[i] = -1;
    return true;
}
// Room for `size` bytes (pack_mutex held). *donate: a block the caller gives up if the pool has
// none (-1 afterwards when it was taken).
void* StoreAlloc(std::size_t size, std::int64_t* donate) noexcept {
    if (store_current < StoreBlocks && store_used[store_current] + size <= StoreBlock) {
        void* at = reinterpret_cast<void*>(store_base + store_current * StoreBlock + store_used[store_current]);
        store_used[store_current] += static_cast<std::uint32_t>(size);
        store_live[store_current].fetch_add(static_cast<std::uint32_t>(size), std::memory_order_relaxed);
        return at;
    }
    std::size_t index = 0;
    while (index < StoreBlocks && store_physical[index] >= 0) ++index;
    if (index == StoreBlocks) return nullptr;
    std::int64_t physical = -1;
    if (eden_direct_low(StoreBlock, StoreBlock, 12, &physical) != 0) {
        if (donate == nullptr || *donate < 0) return nullptr;
        physical = std::exchange(*donate, -1);
    }
    if (!MapSlot(store_base + index * StoreBlock, PROT_READ | PROT_WRITE, physical)) {
        if (donate != nullptr && *donate < 0) *donate = physical;
        else (void)sceKernelReleaseDirectMemory(physical, StoreBlock);
        return nullptr;
    }
    store_physical[index] = physical;
    store_used[index] = static_cast<std::uint32_t>(size);
    store_live[index].store(static_cast<std::uint32_t>(size), std::memory_order_relaxed);
    store_current = index;
    store_held.fetch_add(StoreBlock, std::memory_order_relaxed);
    return reinterpret_cast<void*>(store_base + index * StoreBlock);
}
// Blocks whose data all went back (pack_mutex held).
void StoreReclaim() noexcept {
    // Nothing before the store has started: its tables are not set up (zero is not "no block"),
    // and mapping at store_base 0 would land on the app's own code (the self-test's range goes
    // back through here at start).
    if (store_base == 0) return;
    for (std::size_t i = 0; i < StoreBlocks; ++i) {
        if (store_physical[i] < 0 || i == store_current || store_live[i].load(std::memory_order_acquire) != 0) continue;
        if (!MapSlot(store_base + i * StoreBlock, PROT_NONE, zero_block)) continue;
        if (sceKernelReleaseDirectMemory(store_physical[i], StoreBlock) != 0) std::abort();
        store_physical[i] = -1;
        store_held.fetch_sub(StoreBlock, std::memory_order_relaxed);
    }
}
#endif
std::uint64_t StoreMoved() noexcept; // what StoreCompact moved so far (below)
void StoreRelease(void* data, std::uint32_t size) noexcept {
    const std::size_t index = (reinterpret_cast<std::uintptr_t>(data) - store_base) / StoreBlock;
    store_live[index].fetch_sub(size, std::memory_order_release);
}
// ---- Swap: packed slots on the console's storage ----
// The packed store gives back what LZ4 saves, and nothing for a slot that does not compress: the
// textures, models and sound a game keeps in its own memory once the GPU has its copy. At a Mario
// Kart World race load thousands of such slots were looked at and left (skips=3456) while the pool
// ran out. With a file behind the store such a slot's block goes back too.
//
// The file is on the console's internal storage, whose flash wears with every byte written, so
// writes are what this part is built around:
//   - Nothing is written unless an allocation has failed and asked for memory back: a game that
//     fits writes the file's test page (16 KiB) at most. And then only what the pool lacks.
//   - Memory first: a slot that compresses well goes to the packed store, as before. Only the others
//     go to the file, LZ4-compressed when that saves a tenth or more.
//   - Only slots nothing touched for a while go (Idle slots, below): a slot read back comes back as
//     Clean, and the file keeps its bytes. Giving a Clean slot's block up again writes nothing, so
//     data the game only reads is written once a session however often it comes and goes.
//   - Memory the game reuses (written again after a while unwritten: swap_rewritten) goes last, and
//     only when a request for memory would otherwise get nothing.
//   - A budget: at most EDEN_SWAP_BURST_MIB at once, refilled at EDEN_SWAP_RATE_MIB_MIN a minute, at
//     most EDEN_SWAP_SESSION_MIB a session and EDEN_SWAP_DAY_MIB a day. A small ledger beside the
//     file carries the day's and the lifetime's count over sessions; it is charged before the writes
//     it counts, so a session that ends without warning is counted in full. With the budget used
//     up nothing more is written; what the file holds stays readable.
//   - The file is at most EDEN_SWAP_MIB, never synced, gone when the session ends (it is unlinked
//     as soon as it is open, so a crash leaves nothing), and not made at all when the storage has
//     less than EDEN_SWAP_KEEP_FREE_MIB free.
// Safe: a slot's bytes are hashed when written and checked when read back, the file is tested once
// before it is trusted, a failed or refused write leaves the slot as it was (and ends swapping), and
// data that does not come back as it was ends the session rather than reaching the game.
// EDEN_SWAP=0 turns it off; it is off without storage access (no EDEN_SWAP_PATH).
constexpr std::size_t SwapUnit = 64 * 1024;
constexpr std::size_t SwapMaxUnits = (std::size_t{8} << 30) / SwapUnit; // the file: 8 GiB at most
constexpr std::size_t SwapPackMax = SparseSlot / 10 * 9;                // above this it is stored as it is
std::atomic<std::uint64_t> swap_map[SwapMaxUnits / 64];                 // a bit per unit in use
std::size_t swap_units = 0, swap_hint = 0;                              // the file's limit; where to look next
int swap_fd = -1;
std::atomic<int> swap_state{0};                                         // 0 not started, 1 on, 2 off
char swap_ledger_path[512];
// The budget and the ledger (pack_mutex held).
std::uint64_t swap_tokens = 0, swap_burst = 0, swap_rate = 0 /* bytes a second */, swap_token_ns = 0;
std::uint64_t swap_session_limit = 0, swap_day_limit = 0;
std::uint64_t swap_day = 0, swap_day_before = 0, swap_total_before = 0, swap_ledger_done = 0;
std::uint64_t swap_day_start = 0; // what the session had written when its day began (SwapDayRoll)
// What counts against the day's limit, of `written` in this session.
std::uint64_t SwapDayBytes(std::uint64_t written) noexcept {
    return swap_day_before + (written - std::min(written, swap_day_start));
}
// A session that runs past midnight (UTC) starts a new day there: what it wrote before stays with
// the old one (pack_mutex held).
void SwapDayRoll(std::uint64_t written) noexcept {
    if (const std::uint64_t today = static_cast<std::uint64_t>(time(nullptr)) / 86400; today > swap_day) {
        swap_day = today;
        swap_day_before = 0;
        swap_day_start = written;
    }
}
std::atomic<std::uint64_t> swap_written{0}, swap_writes{0}, swap_write_ns{0}, swap_raw{0};
std::atomic<std::uint64_t> swap_reads{0}, swap_read_ns{0}, swap_read_max_ns{0};
std::atomic<std::uint64_t> swap_used_units{0}, swap_slots{0}, swap_reused{0}, swap_dirtied{0};
std::atomic<std::uint64_t> swap_denied_rate{0}, swap_denied_cap{0}, swap_full{0};
// Buffers for compressed data read back in a fault handler (no allocation there): taken by a flag each.
constexpr std::size_t SwapScratches = 2;
char* swap_scratch[SwapScratches];
std::atomic_flag swap_scratch_taken[SwapScratches];

std::uint64_t FullHash(std::uintptr_t at, std::size_t bytes) noexcept;
std::size_t EnvOr(const char* name, std::size_t fallback) noexcept;
bool PackKeep() noexcept;

std::uint64_t SwapNow() noexcept {
    timespec now{};
    clock_gettime(CLOCK_MONOTONIC, &now);
    return static_cast<std::uint64_t>(now.tv_sec) * 1000000000ull + static_cast<std::uint64_t>(now.tv_nsec);
}
bool SwapOn() noexcept { return swap_state.load(std::memory_order_acquire) == 1; }
// Why it is off, for the report (SwapOff may run in a fault handler: no stdio there).
std::atomic<const char*> swap_off_why{"not needed yet"};
std::atomic<int> swap_off_errno{0};
void SwapOff(const char* why) noexcept {
    const int error = errno;
    if (swap_state.exchange(2, std::memory_order_acq_rel) == 2) return;
    swap_off_errno.store(error, std::memory_order_relaxed);
    swap_off_why.store(why, std::memory_order_release);
    Note("EDEN_SWAP off\n");
}
bool SwapLedgerCharge(std::size_t bytes) noexcept;
// pack_mutex held. False: nothing more may be written (the caller ends swapping).
bool SwapWrite(const void* from, std::size_t size, std::uint64_t offset) noexcept {
    if (!SwapLedgerCharge(size)) return false;
    const std::uint64_t start = SwapNow();
    const auto* bytes = static_cast<const char*>(from);
    std::size_t done = 0;
    while (done < size) {
        const ssize_t wrote = pwrite(swap_fd, bytes + done, size - done, static_cast<off_t>(offset + done));
        if (wrote < 0 && errno == EINTR) continue;
        if (wrote <= 0) break;
        done += static_cast<std::size_t>(wrote);
    }
    swap_writes.fetch_add(1, std::memory_order_relaxed);
    swap_write_ns.fetch_add(SwapNow() - start, std::memory_order_relaxed);
    // What reached the file counts against the budget even when the write then failed.
    swap_written.fetch_add(done, std::memory_order_relaxed);
    swap_tokens -= std::min<std::uint64_t>(swap_tokens, done);
    return done == size;
}
// In a fault handler too: read whole, or not at all.
bool SwapReadAt(void* to, std::size_t size, std::uint64_t offset) noexcept {
    auto* bytes = static_cast<char*>(to);
    std::size_t done = 0;
    while (done < size) {
        const ssize_t got = pread(swap_fd, bytes + done, size - done, static_cast<off_t>(offset + done));
        if (got < 0 && errno == EINTR) continue;
        if (got <= 0) break;
        done += static_cast<std::size_t>(got);
    }
    return done == size;
}
// The ledger: the day (days since 1970), what was written that day, and ever. Written to a second
// file that then takes the ledger's name: the ledger is whole or as it was, whatever happens meanwhile.
// `session`: what this session is to be counted with. No stdio: a fault handler's request for memory
// may write to the file, and so charge the ledger.
char* SwapLedgerText(char* at, const char* text) noexcept {
    while (*text != 0) *at++ = *text++;
    return at;
}
char* SwapLedgerNumber(char* at, std::uint64_t value) noexcept {
    char digits[20];
    int count = 0;
    do digits[count++] = static_cast<char>('0' + value % 10); while ((value /= 10) != 0);
    while (count > 0) *at++ = digits[--count];
    return at;
}
bool SwapLedgerWrite(std::uint64_t session) noexcept {
    if (swap_ledger_path[0] == 0) return false;
    char record[96];
    char* end = SwapLedgerText(record, "EGSWAP1 day=");
    end = SwapLedgerNumber(end, swap_day);
    end = SwapLedgerText(end, " day_bytes=");
    end = SwapLedgerNumber(end, SwapDayBytes(session));
    end = SwapLedgerText(end, " total_bytes=");
    end = SwapLedgerNumber(end, swap_total_before + session);
    *end++ = '\n';
    const auto length = static_cast<std::size_t>(end - record);
    char fresh[sizeof(swap_ledger_path) + 8];
    *SwapLedgerText(SwapLedgerText(fresh, swap_ledger_path), ".new") = 0;
    const int fd = open(fresh, O_WRONLY | O_CREAT | O_TRUNC, 0600);
    if (fd < 0) return false;
    // Synced before it takes the ledger's name: after a power cut the ledger is then the old one
    // or the new one, never an empty file that would forget the day. (As far as the storage
    // goes along: a sync that is refused does not end swapping.)
    const bool whole = write(fd, record, length) == static_cast<ssize_t>(length);
    if (whole) (void)fsync(fd);
    close(fd);
    if (!whole || rename(fresh, swap_ledger_path) != 0) return false;
    swap_ledger_done = session;
    return true;
}
// Before `bytes` more go to the swap file (pack_mutex held): the ledger is charged first, and ahead,
// SwapLedgerStep at a time. A session that ends without warning (out of memory, the power button)
// has then never written more than its ledger says, and the next session's limit for the day
// starts from no less than what the storage took. False: the ledger could not be written, and
// then nothing else is.
constexpr std::uint64_t SwapLedgerStep = std::uint64_t{32} << 20;
bool SwapLedgerCharge(std::size_t bytes) noexcept {
    const std::uint64_t after = swap_written.load(std::memory_order_relaxed) + bytes;
    if (after <= swap_ledger_done || SwapLedgerWrite((after / SwapLedgerStep + 1) * SwapLedgerStep)) return true;
    SwapOff("the ledger could not be written");
    return false;
}
// The exact count instead of the charge, when a session ends in order.
void SwapLedgerFlush() noexcept {
    const std::uint64_t written = swap_written.load(std::memory_order_relaxed);
    if (written != swap_ledger_done) (void)SwapLedgerWrite(written);
}
void SwapLedgerRead() noexcept {
    swap_day = static_cast<std::uint64_t>(time(nullptr)) / 86400;
    swap_day_before = swap_total_before = swap_day_start = 0;
    const int fd = open(swap_ledger_path, O_RDONLY);
    if (fd < 0) return;
    char record[96] = {};
    const ssize_t got = read(fd, record, sizeof(record) - 1);
    close(fd);
    unsigned long long day = 0, day_bytes = 0, total = 0;
    if (got <= 0 || std::sscanf(record, "EGSWAP1 day=%llu day_bytes=%llu total_bytes=%llu", &day, &day_bytes, &total) != 3)
        return;
    swap_total_before = total;
    // A clock that went back keeps the day's count (it never forgives what was written).
    if (day >= swap_day) swap_day_before = day_bytes, swap_day = day;
}
// May `bytes` more be written now (pack_mutex held)? `urgent`: memory was refused, not just short.
bool SwapBudget(std::size_t bytes, bool urgent) noexcept {
    const std::uint64_t elapsed_ms = (SwapNow() - swap_token_ns) / 1000000ull;
    swap_tokens = std::min<std::uint64_t>(swap_burst, swap_tokens + elapsed_ms * swap_rate / 1000ull);
    swap_token_ns += elapsed_ms * 1000000ull;
    // (A limit is reached when less than a whole slot is left of it, whatever is asked for: a slot is
    // asked for twice, before it is compressed and with its size after, and the two must agree, or
    // every slot is compressed for nothing from then on.)
    const std::uint64_t written = swap_written.load(std::memory_order_relaxed);
    const std::uint64_t most = std::max<std::uint64_t>(bytes, SparseSlot);
    SwapDayRoll(written);
    if (written + most > swap_session_limit || SwapDayBytes(written) + most > swap_day_limit) {
        swap_denied_cap.fetch_add(1, std::memory_order_relaxed);
        return false;
    }
    // A quarter of the burst is kept for when memory is refused.
    if (swap_tokens < bytes + (urgent ? 0 : swap_burst / 4)) {
        swap_denied_rate.fetch_add(1, std::memory_order_relaxed);
        return false;
    }
    return true;
}
// A run of `units` free units (pack_mutex held): its first unit, or SwapMaxUnits. Frees come from
// fault handlers at any time and only clear bits.
std::size_t SwapAlloc(std::size_t units) noexcept {
    for (int pass = 0; pass < 2; ++pass) {
        const std::size_t begin = pass == 0 ? swap_hint : 0;
        const std::size_t end = pass == 0 ? swap_units : std::min(swap_units, swap_hint + units);
        std::size_t run = 0;
        for (std::size_t unit = begin; unit < end; ++unit) {
            const std::uint64_t word = swap_map[unit / 64].load(std::memory_order_acquire);
            if (run == 0 && unit % 64 == 0 && word == ~std::uint64_t{0}) {
                unit += 63;
                continue;
            }
            run = (word >> (unit % 64)) & 1 ? 0 : run + 1;
            if (run < units) continue;
            const std::size_t first = unit + 1 - units;
            for (std::size_t i = first; i <= unit; ++i) swap_map[i / 64].fetch_or(std::uint64_t{1} << (i % 64), std::memory_order_acq_rel);
            swap_hint = unit + 1 < swap_units ? unit + 1 : 0;
            swap_used_units.fetch_add(units, std::memory_order_relaxed);
            return first;
        }
    }
    return SwapMaxUnits;
}
void SwapFree(std::size_t first, std::size_t units) noexcept {
    for (std::size_t i = first; i < first + units; ++i) swap_map[i / 64].fetch_and(~(std::uint64_t{1} << (i % 64)), std::memory_order_acq_rel);
    swap_used_units.fetch_sub(units, std::memory_order_relaxed);
}
std::size_t SwapUnitsOf(std::size_t bytes) noexcept { return (bytes + SwapUnit - 1) / SwapUnit; }

// The file, made when the pool is first short (the scanner), or when memory is first asked back by
// the driver (pack_mutex held; never in a fault handler or under the heap's lock: it allocates).
bool SwapStart() noexcept {
    if (const int state = swap_state.load(std::memory_order_acquire); state != 0) return state == 1;
    errno = 0;
    const char* const off = std::getenv("EDEN_SWAP");
    const char* const path = std::getenv("EDEN_SWAP_PATH");
    if (off != nullptr && off[0] == '0') return SwapOff("switched off"), false;
    if (path == nullptr || path[0] == 0 || std::strlen(path) + 8 > sizeof(swap_ledger_path)) return SwapOff("no path (no storage access)"), false;
    if (PackKeep()) return SwapOff("EDEN_CRAM_PACK_KEEP"), false;
    std::size_t limit = std::min<std::size_t>(EnvOr("EDEN_SWAP_MIB", 2048), SwapMaxUnits * SwapUnit >> 20) << 20;
    const std::size_t keep_free = EnvOr("EDEN_SWAP_KEEP_FREE_MIB", 8192) << 20;
    // The storage's free space, from the file's folder.
    char folder[sizeof(swap_ledger_path)];
    std::snprintf(folder, sizeof(folder), "%s", path);
    if (char* const slash = std::strrchr(folder, '/'); slash != nullptr && slash != folder) *slash = 0;
    struct statvfs space {};
    std::uint64_t free_bytes = ~std::uint64_t{0};
    if (statvfs(folder, &space) == 0 && space.f_frsize != 0)
        free_bytes = static_cast<std::uint64_t>(space.f_bavail) * static_cast<std::uint64_t>(space.f_frsize);
    if (free_bytes < keep_free + (std::uint64_t{256} << 20)) return SwapOff("the storage is nearly full"), false;
    limit = static_cast<std::size_t>(std::min<std::uint64_t>(limit, free_bytes - keep_free));
    if (limit < (std::size_t{64} << 20)) return SwapOff("EDEN_SWAP_MIB under 64"), false;
    for (std::size_t i = 0; i < SwapScratches; ++i)
        if (swap_scratch[i] == nullptr && (swap_scratch[i] = new (std::nothrow) char[SwapPackMax]) == nullptr)
            return SwapOff("no memory for its buffers"), false;
    const int fd = open(path, O_RDWR | O_CREAT | O_TRUNC, 0600);
    if (fd < 0) return SwapOff("the file could not be made"), false;
    swap_fd = fd;
    // Trusted only after a page written to it came back as written (16 KiB: all a session that
    // never runs out of memory writes).
    bool tested = false;
    {
        constexpr std::size_t TestBytes = 16384;
        char* const pattern = swap_scratch[0];
        for (std::size_t i = 0; i < TestBytes; ++i) pattern[i] = static_cast<char>((i * 2654435761u) >> 13);
        tested = pwrite(fd, pattern, TestBytes, 0) == static_cast<ssize_t>(TestBytes) && SwapReadAt(pattern + TestBytes, TestBytes, 0) &&
                 std::memcmp(pattern, pattern + TestBytes, TestBytes) == 0 && ftruncate(fd, 0) == 0;
    }
    // Unlinked while open: the storage takes the file back when the app ends, however it ends.
    const bool unlinked = unlink(path) == 0;
    if (!tested) {
        close(fd);
        swap_fd = -1;
        return SwapOff("the file did not read back as written"), false;
    }
    std::snprintf(swap_ledger_path, sizeof(swap_ledger_path), "%s.ledger", path);
    SwapLedgerRead();

    swap_units = limit / SwapUnit;
    swap_hint = 0;
    for (auto& word : swap_map) word.store(0, std::memory_order_relaxed);
    swap_used_units.store(0, std::memory_order_relaxed);
    swap_burst = std::uint64_t{EnvOr("EDEN_SWAP_BURST_MIB", 1024)} << 20;
    swap_rate = (std::uint64_t{EnvOr("EDEN_SWAP_RATE_MIB_MIN", 128)} << 20) / 60;
    swap_session_limit = std::uint64_t{EnvOr("EDEN_SWAP_SESSION_MIB", 4096)} << 20;
    swap_day_limit = std::uint64_t{EnvOr("EDEN_SWAP_DAY_MIB", 8192)} << 20;
    swap_tokens = swap_burst;
    swap_token_ns = SwapNow();
    swap_state.store(1, std::memory_order_release);
    char line[400];
    const int n = std::snprintf(line, sizeof(line),
        "EDEN_SWAP on: %s limit=%zuMiB storage_free=%lluMiB unlinked=%d burst=%lluMiB rate=%lluMiB/min session=%lluMiB "
        "day=%lluMiB written_today=%lluMiB written_ever=%lluMiB\n", path, limit >> 20,
        (unsigned long long)(free_bytes == ~std::uint64_t{0} ? 0 : free_bytes >> 20), unlinked ? 1 : 0,
        (unsigned long long)(swap_burst >> 20), (unsigned long long)(swap_rate * 60 >> 20),
        (unsigned long long)(swap_session_limit >> 20), (unsigned long long)(swap_day_limit >> 20),
        (unsigned long long)(swap_day_before >> 20), (unsigned long long)(swap_total_before >> 20));
    if (n > 0) Report(line);
    return true;
}
// A slot's 2 MiB from the file into `view` (a fault handler): false when it did not come back as written.
bool SwapReadSlot(void* view, const PackedSlot& entry) noexcept {
    const std::uint64_t start = SwapNow();
    const std::uint64_t offset = std::uint64_t{entry.swap - 1} * SwapUnit;
    bool ok = false;
    if (entry.size == SparseSlot) {
        ok = SwapReadAt(view, SparseSlot, offset);
    } else if (entry.size <= SwapPackMax) {
        std::size_t scratch = 0;
        while (swap_scratch_taken[scratch].test_and_set(std::memory_order_acquire))
            if (++scratch == SwapScratches) scratch = 0, sched_yield();
        ok = SwapReadAt(swap_scratch[scratch], entry.size, offset) &&
             LZ4_decompress_safe(swap_scratch[scratch], static_cast<char*>(view), static_cast<int>(entry.size),
                                 static_cast<int>(SparseSlot)) == static_cast<int>(SparseSlot);
        swap_scratch_taken[scratch].clear(std::memory_order_release);
    }
    ok = ok && FullHash(reinterpret_cast<std::uintptr_t>(view), SparseSlot) == entry.hash;
    const std::uint64_t took = SwapNow() - start;
    swap_reads.fetch_add(1, std::memory_order_relaxed);
    swap_read_ns.fetch_add(took, std::memory_order_relaxed);
    std::uint64_t most = swap_read_max_ns.load(std::memory_order_relaxed);
    while (took > most && !swap_read_max_ns.compare_exchange_weak(most, took, std::memory_order_relaxed)) {}
    return ok;
}

// Forgets a slot's packed data, in the store or in the swap file (the slot is locked).
void ForgetPacked(std::size_t slot) noexcept {
    PackedSlot* const table = packed_slots.load(std::memory_order_acquire);
    if (table == nullptr) return;
    if (table[slot].swap != 0) {
        SwapFree(table[slot].swap - 1, SwapUnitsOf(table[slot].size));
        swap_slots.fetch_sub(1, std::memory_order_relaxed);
    } else if (table[slot].data != nullptr) {
        packed_count.fetch_sub(1, std::memory_order_relaxed);
        packed_bytes.fetch_sub(table[slot].size, std::memory_order_relaxed);
        StoreRelease(table[slot].data, table[slot].size);
    }
    table[slot] = {};
}
// The swap file holds the slot's bytes (a Packed slot's only copy, or a Clean or Idle slot's second).
bool InSwap(std::size_t slot) noexcept {
    PackedSlot* const table = packed_slots.load(std::memory_order_acquire);
    return table != nullptr && table[slot].swap != 0;
}

#ifndef PS5_NATIVE
extern std::mutex pack_mutex;
// The host's check (below): the swap file as a session that has not started finds it.
void SwapCheckReset() noexcept {
    if (swap_fd >= 0) close(swap_fd);
    swap_fd = -1;
    swap_state.store(0, std::memory_order_release);
    swap_off_why.store("not needed yet", std::memory_order_release);
    swap_written.store(0, std::memory_order_relaxed);
    swap_ledger_done = 0;
    swap_ledger_path[0] = 0;
}
// What the ledger says was written on its day: -1 without a ledger.
long long SwapCheckLedgerDay(const char* ledger) noexcept {
    char record[96] = {};
    const int fd = open(ledger, O_RDONLY);
    if (fd < 0) return -1;
    const ssize_t got = read(fd, record, sizeof(record) - 1);
    close(fd);
    unsigned long long day = 0, day_bytes = 0, total = 0;
    if (got <= 0 || std::sscanf(record, "EGSWAP1 day=%llu day_bytes=%llu total_bytes=%llu", &day, &day_bytes, &total) != 3) return -1;
    return static_cast<long long>(day_bytes);
}
// A slot's bytes to the file as PackSlot writes them: its entry, or one with swap 0 when the budget
// or the file said no.
PackedSlot SwapCheckWrite(const char* slot, const char* stored, std::size_t size, bool urgent) noexcept {
    if (!SwapBudget(size, urgent)) return {};
    const std::size_t unit = SwapAlloc(SwapUnitsOf(size));
    if (unit == SwapMaxUnits) return {};
    const std::uint64_t hash = FullHash(reinterpret_cast<std::uintptr_t>(slot), SparseSlot);
    if (!SwapWrite(stored, size, std::uint64_t{unit} * SwapUnit)) {
        SwapFree(unit, SwapUnitsOf(size));
        return {};
    }
    return {nullptr, static_cast<std::uint32_t>(size), static_cast<std::uint32_t>(unit + 1), hash};
}
} // namespace

// The host's check of the swap file's own parts (headless/memory_check.cpp): the file's test, a
// slot's way there and back (as it is, and compressed), the hash that catches a changed byte, the
// budget's limits, and the ledger over sessions that end in order, without warning, and after
// midnight. `folder`: where the file may be made. Returns 0, or the number of the step that failed.
extern "C" int eden_swap_check(const char* folder) noexcept {
    char path[400], ledger[420];
    // (The port's LZ4 symbols are weak: a program that checks this links LZ4 itself.)
    if (&LZ4_compress_default == nullptr || &LZ4_decompress_safe == nullptr || &LZ4_compressBound == nullptr) return 100;
    if (std::snprintf(path, sizeof(path), "%s/guest-memory.swap", folder) >= static_cast<int>(sizeof(path))) return 1;
    std::snprintf(ledger, sizeof(ledger), "%s.ledger", path);
    unlink(ledger);
    const auto session = [&](const char* burst, const char* per_session, const char* per_day) noexcept {
        SwapCheckReset();
        setenv("EDEN_SWAP_PATH", path, 1);
        setenv("EDEN_SWAP_MIB", "64", 1);
        setenv("EDEN_SWAP_KEEP_FREE_MIB", "0", 1);
        setenv("EDEN_SWAP_RATE_MIB_MIN", "0", 1);
        setenv("EDEN_SWAP_BURST_MIB", burst, 1);
        setenv("EDEN_SWAP_SESSION_MIB", per_session, 1);
        setenv("EDEN_SWAP_DAY_MIB", per_day, 1);
        return SwapStart();
    };
    const std::lock_guard lock{pack_mutex};
    int failed = 0;
    auto expect = [&failed, step = 1](bool value) mutable noexcept {
        ++step;
        if (!value && failed == 0) failed = step;
        return value;
    };
    std::vector<char> slot(SparseSlot), back(SparseSlot), packed(static_cast<std::size_t>(LZ4_compressBound(static_cast<int>(SparseSlot))));
    std::uint64_t seed = 0x9e3779b97f4a7c15ull;
    for (char& byte : slot) byte = static_cast<char>((seed = seed * 6364136223846793005ull + 1442695040888963407ull) >> 56);
    const std::uint64_t mib = std::uint64_t{1} << 20;

    // Session 1: 8 MiB at once, no refill. The file is unlinked while open; no ledger before a write.
    if (!expect(session("8", "64", "64"))) return failed;                                       // 2
    expect(access(path, F_OK) != 0 && access(ledger, F_OK) != 0);                               // 3
    // A slot that does not compress, there and back; the ledger was charged first, a step ahead.
    const PackedSlot raw = SwapCheckWrite(slot.data(), slot.data(), SparseSlot, true);
    expect(raw.swap != 0 && SwapReadSlot(back.data(), raw) && std::memcmp(slot.data(), back.data(), SparseSlot) == 0); // 4
    expect(SwapCheckLedgerDay(ledger) == static_cast<long long>(SwapLedgerStep));                // 5
    // A slot LZ4 makes smaller, there and back; a byte changed in the file is caught.
    std::memset(slot.data(), 0, SparseSlot / 2);
    const int size = LZ4_compress_default(slot.data(), packed.data(), static_cast<int>(SparseSlot), static_cast<int>(packed.size()));
    expect(size > 0 && static_cast<std::size_t>(size) <= SwapPackMax);                           // 6
    const PackedSlot small = SwapCheckWrite(slot.data(), packed.data(), static_cast<std::size_t>(size), true);
    std::memset(back.data(), 0x55, SparseSlot);
    expect(small.swap != 0 && SwapReadSlot(back.data(), small) && std::memcmp(slot.data(), back.data(), SparseSlot) == 0); // 7
    const off_t spot = static_cast<off_t>(std::uint64_t{small.swap - 1} * SwapUnit + 4096);
    const char changed = static_cast<char>(~packed[4096]);
    expect(pwrite(swap_fd, &changed, 1, spot) == 1 && !SwapReadSlot(back.data(), small));         // 8
    expect(pwrite(swap_fd, &packed[4096], 1, spot) == 1 && SwapReadSlot(back.data(), small));     // 9
    // Units given back are found again.
    const std::uint64_t used = swap_used_units.load(std::memory_order_relaxed);
    SwapFree(raw.swap - 1, SwapUnitsOf(SparseSlot));
    expect(swap_used_units.load(std::memory_order_relaxed) == used - SwapUnitsOf(SparseSlot));    // 10
    // The burst: about 3 MiB of 8 are gone. One more slot may go either way; after it a quarter of the
    // burst is left for when memory was refused; after one more nothing is.
    expect(SwapBudget(SparseSlot, false) && SwapCheckWrite(slot.data(), slot.data(), SparseSlot, false).swap != 0); // 11
    expect(!SwapBudget(SparseSlot, false) && SwapBudget(SparseSlot, true));                       // 12
    expect(SwapCheckWrite(slot.data(), slot.data(), SparseSlot, true).swap != 0 && !SwapBudget(SparseSlot, true)); // 13
    // The session ends in order: the ledger has the exact count.
    const std::uint64_t first = swap_written.load(std::memory_order_relaxed);
    SwapLedgerFlush();
    expect(first == 3 * SparseSlot + static_cast<std::uint64_t>(size) && SwapCheckLedgerDay(ledger) == static_cast<long long>(first)); // 14

    // Session 2 starts from the ledger: of 16 MiB a day, what session 1 wrote is gone.
    if (!expect(session("64", "64", "16"))) return failed;                                       // 15
    expect(swap_day_before == first && swap_total_before == first);                              // 16
    std::size_t wrote = 0;
    while (wrote < 16 && SwapCheckWrite(slot.data(), slot.data(), SparseSlot, true).swap != 0) ++wrote;
    expect(wrote == (16 * mib - first) / SparseSlot && swap_denied_cap.load(std::memory_order_relaxed) != 0); // 17
    // It ends without warning: the ledger keeps the charge, no less than what was written.
    const std::uint64_t second = swap_written.load(std::memory_order_relaxed);
    expect(SwapCheckLedgerDay(ledger) >= static_cast<long long>(first + second));                 // 18

    // Session 3: 4 MiB a session.
    unlink(ledger);
    if (!expect(session("64", "4", "64"))) return failed;                                        // 19
    expect(swap_day_before == 0);                                                                // 20
    wrote = 0;
    while (wrote < 16 && SwapCheckWrite(slot.data(), slot.data(), SparseSlot, true).swap != 0) ++wrote;
    expect(wrote == 2);                                                                          // 21
    // Session 4 runs past midnight: 4 MiB a day are used up, and the new day starts from nothing.
    unlink(ledger);
    if (!expect(session("64", "64", "4"))) return failed;                                        // 22
    wrote = 0;
    while (wrote < 16 && SwapCheckWrite(slot.data(), slot.data(), SparseSlot, true).swap != 0) ++wrote;
    expect(wrote == 2);                                                                          // 23
    --swap_day; // (as if the session had begun the day before)
    expect(SwapCheckWrite(slot.data(), slot.data(), SparseSlot, true).swap != 0 &&
           SwapDayBytes(swap_written.load(std::memory_order_relaxed)) == SparseSlot);            // 24
    // A ledger that cannot be written: nothing goes to the file, and swapping ends.
    if (!expect(session("64", "64", "64"))) return failed;                                       // 25
    std::snprintf(swap_ledger_path, sizeof(swap_ledger_path), "%s/no-such-folder/ledger", folder);
    const std::uint64_t before = swap_written.load(std::memory_order_relaxed);
    expect(SwapCheckWrite(slot.data(), slot.data(), SparseSlot, true).swap == 0 &&
           swap_written.load(std::memory_order_relaxed) == before && !SwapOn());                 // 26
    SwapCheckReset();
    unlink(ledger);
    for (const char* name : {"EDEN_SWAP_PATH", "EDEN_SWAP_MIB", "EDEN_SWAP_KEEP_FREE_MIB", "EDEN_SWAP_RATE_MIB_MIN",
                             "EDEN_SWAP_BURST_MIB", "EDEN_SWAP_SESSION_MIB", "EDEN_SWAP_DAY_MIB"})
        unsetenv(name);
    return failed;
}
namespace {
#endif

// ---- Idle slots: which slots nothing touches ----
// Packing used to choose slots by when they were last written (a sampled hash each pass). Reads are
// not seen that way: more than half of the slots packed in a Mario Kart World session were read and
// unpacked again (packs=1252 unpacks=724), a block and 2 MiB of LZ4 each time. For the swap file
// that would be a write each time. So while the pool is short the scanner takes all access away
// from slots for a while (Idle: the block stays, mprotect only). A slot the game or the emulator
// touches faults once and has its access back (a signal and an mprotect); it is then left alone
// for IdleBackoff seconds, twice as long each time it was woken or unpacked. A slot still Idle
// after IdleSeconds is one nothing reads or writes: those are packed, the longest idle first, and
// only those go to the swap file. EDEN_CRAM_IDLE=0 goes back to choosing by writes (and no swap).
bool IdleEnabled() noexcept {
    static const bool on = [] {
        const char* value = std::getenv("EDEN_CRAM_IDLE");
        return value == nullptr || value[0] != '0';
    }();
    return on;
}
std::uint32_t* idle_since = nullptr;                    // per slot: when it was made Idle (s)
// Per slot: how often it was written again after it had stayed unwritten for RecycleQuietSeconds or
// more. That is memory the game reuses (a pool it loads the next track into): untouched for
// minutes, it looks like the best there is to take, and what is written to the file for it is
// thrown away at the game's next write. In a 14-minute Mario Kart World session 684 of 979 slots
// written to the file ended that way, whether they had been unwritten for one minute (79%) or for
// five to ten (61%): how long a slot has been unwritten does not tell the two kinds apart, its
// history does. Counted for free by the scanner, which sees a slot's sampled hash change
// (ColdPass), and by the write that drops a slot's copy in the file (MakeDirty, under the slot's
// lock; rewrite_noted keeps the scanner from counting that write again). Slots never counted go
// to the file first. The others go only when a request for memory would otherwise get nothing
// (RecoverMemory): they used to go once they had stayed unwritten for 2, 4, 8, then 16 minutes,
// and of the slots written that way after one and two such counts 74% and 83% were thrown away
// by the game's next write.
constexpr std::uint32_t RecycleQuietSeconds = 30;
std::atomic<std::uint8_t>* swap_rewritten = nullptr;
std::atomic<std::uint8_t>* rewrite_noted = nullptr;
void NoteRewritten(std::size_t slot) noexcept {
    if (swap_rewritten == nullptr) return;
    if (const auto times = swap_rewritten[slot].load(std::memory_order_relaxed); times < 255)
        swap_rewritten[slot].store(static_cast<std::uint8_t>(times + 1), std::memory_order_relaxed);
}
unsigned Rewritten(std::size_t slot) noexcept {
    return swap_rewritten != nullptr ? swap_rewritten[slot].load(std::memory_order_relaxed) : 0;
}
// What a slot was when it went to the file (per slot: SwapAgeClass in bits 0-2, how often it had
// been written again in bits 4-5), and how many of each kind were written and then had their copy
// dropped by a write (the report's EDEN_SWAP_AGES line).
std::uint8_t* swap_age_class = nullptr;
std::atomic<std::uint32_t> swap_age_written[6], swap_age_dropped[6], swap_kind_written[4], swap_kind_dropped[4];
unsigned SwapAgeClass(std::uint32_t unwritten_s) noexcept {
    return unwritten_s < 30 ? 0 : unwritten_s < 60 ? 1 : unwritten_s < 120 ? 2 : unwritten_s < 300 ? 3 : unwritten_s < 600 ? 4 : 5;
}
// A slot goes to the file (pack_mutex held): what it is then.
void NoteSwapWrite(std::size_t slot, std::uint32_t unwritten_s) noexcept {
    if (swap_age_class == nullptr) return;
    const unsigned age = SwapAgeClass(unwritten_s), kind = std::min<unsigned>(Rewritten(slot), 3);
    swap_age_class[slot] = static_cast<std::uint8_t>(age | kind << 4);
    swap_age_written[age].fetch_add(1, std::memory_order_relaxed);
    swap_kind_written[kind].fetch_add(1, std::memory_order_relaxed);
}
std::atomic<std::uint32_t>* touch_since = nullptr;      // per slot: when it was last woken or unpacked (s)
std::atomic<std::uint8_t>* touch_times = nullptr;       // per slot: how often
std::atomic<std::uint64_t> idled{0}, idle_woken{0};
// Read faults that found their slot Clean already (HandleGuestWriteFault), and those that then
// faulted again at once: the fault's write bit was wrong if these are many.
std::atomic<std::uint64_t> clean_late_reads{0}, clean_refaults{0};
std::uint32_t ColdNow() noexcept;
void NoteTouched(std::size_t slot) noexcept {
    if (touch_since == nullptr) return;
    touch_since[slot].store(ColdNow(), std::memory_order_relaxed);
    if (const auto times = touch_times[slot].load(std::memory_order_relaxed); times < 255)
        touch_times[slot].store(static_cast<std::uint8_t>(times + 1), std::memory_order_relaxed);
}
// Own or Clean -> Idle (the scanner; the slot is locked).
bool MakeIdle(SparseRange& range, std::size_t slot, std::uint32_t now) noexcept {
    DropSlotAliases(range, slot);
    if (mprotect(reinterpret_cast<void*>(SlotAddress(range, slot)), SparseSlot, PROT_NONE) != 0) return false;
    if (idle_since != nullptr) idle_since[slot] = now;
    idled.fetch_add(1, std::memory_order_relaxed);
    return true;
}
// Idle -> Clean (the swap file has its bytes: read-only) or Own (a fault handler; the slot is locked).
std::uint8_t WakeIdle(SparseRange& range, std::size_t slot) noexcept {
    const bool clean = InSwap(slot);
    if (mprotect(reinterpret_cast<void*>(SlotAddress(range, slot)), SparseSlot, clean ? PROT_READ : PROT_READ | PROT_WRITE) != 0)
        std::abort();
    NoteTouched(slot);
    idle_woken.fetch_add(1, std::memory_order_relaxed);
    return clean ? SlotClean : SlotOwn;
}
// Clean or Idle -> Own for a write: read-write, and the swap file's copy (now stale) is forgotten.
bool MakeDirty(SparseRange& range, std::size_t slot) noexcept {
    DropSlotAliases(range, slot); // read-only aliases of a Clean slot
    if (mprotect(reinterpret_cast<void*>(SlotAddress(range, slot)), SparseSlot, PROT_READ | PROT_WRITE) != 0) std::abort();
    if (InSwap(slot)) {
        ForgetPacked(slot);
        swap_dirtied.fetch_add(1, std::memory_order_relaxed);
        NoteRewritten(slot);
        if (rewrite_noted != nullptr) rewrite_noted[slot].store(1, std::memory_order_relaxed);
        if (swap_age_class != nullptr) {
            const std::uint8_t was = swap_age_class[slot];
            swap_age_dropped[(was & 7) % 6].fetch_add(1, std::memory_order_relaxed);
            swap_kind_dropped[(was >> 4) & 3].fetch_add(1, std::memory_order_relaxed);
        }
    }
    return true;
}

// Blocks packing freed, kept for unpacking (KeepForUnpack below).
constexpr std::size_t UnpackKeepBlocks = 256;
std::int64_t unpack_keep[UnpackKeepBlocks];
std::atomic<std::size_t> unpack_keep_count{0};
std::atomic_flag unpack_keep_lock = ATOMIC_FLAG_INIT;
#ifdef PS5_NATIVE
// A zeroed block from the reserve (the pool refused one).
bool ReserveBlock(std::int64_t* physical) noexcept {
    Spin(reserve_lock);
    const std::size_t count = reserve_count.load(std::memory_order_relaxed);
    const bool ok = count > 0;
    if (ok) {
        *physical = reserve[count - 1];
        reserve_count.store(count - 1, std::memory_order_relaxed);
    }
    reserve_lock.clear(std::memory_order_release);
    if (ok) counters.reserve_used.fetch_add(1, std::memory_order_relaxed);
    return ok;
}
// Zeroed blocks into the reserve until it holds `blocks` (at most ReserveBlocks) or the pool says no.
std::atomic<std::uint64_t> reserve_hold_ns{0}; // no refill before this (CLOCK_MONOTONIC): GiveBackIdleBlocks
void FillReserve(std::size_t blocks = ReserveTarget()) noexcept {
    // What a request for memory was just given from here is left in the pool for its taker: refilled
    // 100 ms later, the same 94 MiB went back and forth thirty times while the heap's allocation,
    // which had asked for them, failed.
    if (SwapNow() < reserve_hold_ns.load(std::memory_order_relaxed)) return;
    blocks = std::min(blocks, ReserveTarget());
    for (;;) {
        Spin(reserve_lock);
        const bool full = reserve_count.load(std::memory_order_relaxed) >= blocks;
        reserve_lock.clear(std::memory_order_release);
        if (full) return;
        std::int64_t physical = -1;
        if (!ZeroedBlock(&physical)) return;
        Spin(reserve_lock);
        const std::size_t count = reserve_count.load(std::memory_order_relaxed);
        if (count < blocks) {
            reserve[count] = physical;
            reserve_count.store(count + 1, std::memory_order_relaxed);
            physical = -1;
        }
        reserve_lock.clear(std::memory_order_release);
        if (physical >= 0 && sceKernelReleaseDirectMemory(physical, SparseSlot) != 0) std::abort();
    }
}
// Blocks packing freed, kept for unpacking only (an unpack overwrites the whole block, so they are
// not zeroed). A burst of packing under a short pool is followed by reads of some of those slots: a
// read whose unpack found no block waited in its fault handler for up to 2 s, and with dozens of them
// the threads the GPU's work waits on stalled until the driver declared the device lost (Mario Kart
// World: every GPU hang came seconds after ~1.3 GB was packed). EDEN_CRAM_UNPACK_KEEP_MIB (default
// 256, at most 512) sets how many are kept.
std::size_t UnpackKeepTarget() noexcept {
    static const std::size_t blocks = [] {
        const char* value = std::getenv("EDEN_CRAM_UNPACK_KEEP_MIB");
        const std::size_t mib = value != nullptr ? std::strtoull(value, nullptr, 10) : 256;
        return std::min(UnpackKeepBlocks, mib / (SparseSlot >> 20));
    }();
    return blocks;
}
bool KeepForUnpack(std::int64_t physical) noexcept {
    Spin(unpack_keep_lock);
    const std::size_t count = unpack_keep_count.load(std::memory_order_relaxed);
    const bool kept = count < UnpackKeepTarget();
    if (kept) {
        unpack_keep[count] = physical;
        unpack_keep_count.store(count + 1, std::memory_order_relaxed);
    }
    unpack_keep_lock.clear(std::memory_order_release);
    return kept;
}
bool UnpackBlock(std::int64_t* physical) noexcept {
    Spin(unpack_keep_lock);
    const std::size_t count = unpack_keep_count.load(std::memory_order_relaxed);
    const bool ok = count > 0;
    if (ok) {
        *physical = unpack_keep[count - 1];
        unpack_keep_count.store(count - 1, std::memory_order_relaxed);
    }
    unpack_keep_lock.clear(std::memory_order_release);
    return ok;
}
void EmptyUnpackKeep() noexcept {
    Spin(unpack_keep_lock);
    for (std::size_t i = 0; i < unpack_keep_count.load(std::memory_order_relaxed); ++i)
        if (sceKernelReleaseDirectMemory(unpack_keep[i], SparseSlot) != 0) std::abort();
    unpack_keep_count.store(0, std::memory_order_relaxed);
    unpack_keep_lock.clear(std::memory_order_release);
}
void EmptyReserve() noexcept {
    EmptyUnpackKeep();
    Spin(reserve_lock);
    for (std::size_t i = 0; i < reserve_count.load(std::memory_order_relaxed); ++i)
        if (sceKernelReleaseDirectMemory(reserve[i], SparseSlot) != 0) std::abort();
    reserve_count.store(0, std::memory_order_relaxed);
    reserve_lock.clear(std::memory_order_release);
}
#endif

// One more slot shows `physical`; the first one makes it a shared block. False when the table is full.
bool ShareBlock(std::int64_t physical) noexcept {
    Spin(shared_lock);
    std::size_t free_entry = SharedBlocks;
    bool done = false;
    for (std::size_t i = 0; i < SharedBlocks && !done; ++i) {
        if (shared_refs[i] > 0 && shared_block[i] == physical) {
            ++shared_refs[i];
            done = true;
        } else if (shared_refs[i] == 0 && free_entry == SharedBlocks) {
            free_entry = i;
        }
    }
    if (!done && free_entry < SharedBlocks) {
        shared_block[free_entry] = physical;
        shared_refs[free_entry] = 1;
        done = true;
    }
    shared_lock.clear(std::memory_order_release);
    return done;
}
// One slot less shows `physical`: the references left (0: the block is no longer shown).
int UnshareBlock(std::int64_t physical) noexcept {
    Spin(shared_lock);
    int left = 0;
    for (std::size_t i = 0; i < SharedBlocks; ++i)
        if (shared_refs[i] > 0 && shared_block[i] == physical) {
            left = --shared_refs[i];
            break;
        }
    shared_lock.clear(std::memory_order_release);
    return left;
}

// ---- The changes of a slot (the slot is locked) ----

// Zero -> Own: a zeroed block of its own, read-write.
bool PromoteZero(SparseRange& range, std::size_t slot) noexcept {
    DropSlotAliases(range, slot);
    const std::uintptr_t at = SlotAddress(range, slot);
#ifdef PS5_NATIVE
    std::int64_t physical = -1;
    if (!ZeroedBlock(&physical) && !ReserveBlock(&physical)) return false;
    if (!MapSlot(at, PROT_READ | PROT_WRITE, physical)) {
        (void)sceKernelReleaseDirectMemory(physical, SparseSlot);
        return false;
    }
    range.physical[slot] = physical;
#else
    if (mprotect(reinterpret_cast<void*>(at), SparseSlot, PROT_READ | PROT_WRITE) != 0) return false;
    range.physical[slot] = 0;
#endif
    guest_committed.fetch_add(SparseSlot, std::memory_order_relaxed);
    return true;
}

// Shared -> Own: the last slot to show a block takes it back read-write; the others copy it.
bool PromoteShared(SparseRange& range, std::size_t slot) noexcept {
    DropSlotAliases(range, slot);
    const std::uintptr_t at = SlotAddress(range, slot);
#ifdef PS5_NATIVE
    const std::int64_t shared = range.physical[slot];
    Spin(shared_lock);
    bool last = false;
    for (std::size_t i = 0; i < SharedBlocks; ++i)
        if (shared_refs[i] > 0 && shared_block[i] == shared) {
            last = shared_refs[i] == 1;
            if (last) shared_refs[i] = 0;
            break;
        }
    shared_lock.clear(std::memory_order_release);
    if (last) return mprotect(reinterpret_cast<void*>(at), SparseSlot, PROT_READ | PROT_WRITE) == 0;
    std::int64_t physical = -1;
    if (eden_direct_low(SparseSlot, SparseSlot, 12, &physical) != 0 && !ReserveBlock(&physical)) return false;
    void* const view = MapView(physical, SparseSlot, PROT_READ | PROT_WRITE, SparseSlot);
    if (view == nullptr) {
        (void)sceKernelReleaseDirectMemory(physical, SparseSlot);
        return false;
    }
    std::memcpy(view, reinterpret_cast<const void*>(at), SparseSlot);
    UnmapView(view, SparseSlot);
    if (!MapSlot(at, PROT_READ | PROT_WRITE, physical)) {
        (void)sceKernelReleaseDirectMemory(physical, SparseSlot);
        return false;
    }
    range.physical[slot] = physical;
    if (UnshareBlock(shared) == 0 && sceKernelReleaseDirectMemory(shared, SparseSlot) != 0) std::abort();
    guest_committed.fetch_add(SparseSlot, std::memory_order_relaxed);
    counters.copies.fetch_add(1, std::memory_order_relaxed);
    return true;
#else
    (void)range;
    (void)slot;
    (void)at;
    return false; // the host never shares
#endif
}

// Own, Shared or Fine -> Zero: the slot shows the zero block again and its memory goes back.
void ReturnSlot(SparseRange& range, std::size_t slot) noexcept {
    DropSlotAliases(range, slot);
    const std::uintptr_t at = SlotAddress(range, slot);
    const std::uint8_t state = SlotState(range, slot);
    if (state == SlotPacked) {
        // Its block went back when it was packed: it shows the zero block (readable) again.
#ifdef PS5_NATIVE
        if (!MapSlot(at, PROT_READ, zero_block)) std::abort();
#endif
        ForgetPacked(slot);
        range.physical[slot] = -1;
        return;
    }
    // Idle or Clean: a block of its own, as an Own slot's (below), and maybe a copy in the swap file.
    if (state == SlotIdle || state == SlotClean) ForgetPacked(slot);
    if (state == SlotFine) {
        const std::size_t kept = FineKept(range, slot);
#ifdef PS5_NATIVE
        if (!MapSlot(at, PROT_READ, zero_block)) std::abort();
        if (kept != 0 && sceKernelReleaseDirectMemory(range.physical[slot], kept * FinePage) != 0) std::abort();
#else
        if (madvise(reinterpret_cast<void*>(at), SparseSlot, MADV_DONTNEED) != 0) std::abort();
#endif
        guest_committed.fetch_sub(kept * FinePage, std::memory_order_relaxed);
        range.fine[slot] &= 0xf00;
        range.physical[slot] = -1;
        return;
    }
#ifdef PS5_NATIVE
    if (!MapSlot(at, PROT_READ, zero_block)) std::abort();
    if (state == SlotShared) {
        if (UnshareBlock(range.physical[slot]) == 0) {
            if (sceKernelReleaseDirectMemory(range.physical[slot], SparseSlot) != 0) std::abort();
            guest_committed.fetch_sub(SparseSlot, std::memory_order_relaxed);
        }
    } else {
        if (sceKernelReleaseDirectMemory(range.physical[slot], SparseSlot) != 0) std::abort();
        guest_committed.fetch_sub(SparseSlot, std::memory_order_relaxed);
    }
#else
    (void)state;
    if (madvise(reinterpret_cast<void*>(at), SparseSlot, MADV_DONTNEED) != 0 ||
        mprotect(reinterpret_cast<void*>(at), SparseSlot, PROT_READ) != 0)
        std::abort();
    guest_committed.fetch_sub(SparseSlot, std::memory_order_relaxed);
#endif
    range.physical[slot] = -1;
}

// Packed -> Own: a block of its own, read-write, with the slot's data unpacked into it. From the swap
// file -> Clean instead (read-only, the file keeps its copy), unless the block is wanted for writing.
// *to: the slot's state afterwards.
bool UnpackSlot(SparseRange& range, std::size_t slot, std::uint8_t* to, bool for_write = false) noexcept {
#ifdef PS5_NATIVE
    PackedSlot* const table = packed_slots.load(std::memory_order_acquire);
    if (table == nullptr || (table[slot].data == nullptr && table[slot].swap == 0)) std::abort();
    const bool from_swap = table[slot].swap != 0;
    const std::uintptr_t at = SlotAddress(range, slot);
    std::int64_t physical = -1;
    if (eden_direct_low(SparseSlot, SparseSlot, 12, &physical) != 0 && !UnpackBlock(&physical) &&
        !ReserveBlock(&physical))
        return false;
    // Unpacked in a view of its own and only then mapped at the slot: mapped first, a reader that
    // had not faulted yet would see the block half unpacked.
    void* const view = MapView(physical, SparseSlot, PROT_READ | PROT_WRITE, SparseSlot);
    if (view == nullptr) {
        (void)sceKernelReleaseDirectMemory(physical, SparseSlot);
        return false;
    }
    if (from_swap) {
        if (!SwapReadSlot(view, table[slot])) {
            Note("EDEN_SWAP a slot's data did not come back from the swap file as it was written\n");
            std::abort(); // the guest's data is gone: never carry on
        }
    } else {
        const int out = LZ4_decompress_safe(static_cast<const char*>(table[slot].data), static_cast<char*>(view),
                                            static_cast<int>(table[slot].size), static_cast<int>(SparseSlot));
        if (out != static_cast<int>(SparseSlot)) std::abort(); // the guest's data is gone: never carry on
    }
    if (pack_kept != nullptr && pack_kept[slot] >= 0) {
        if (void* const kept_view = MapView(pack_kept[slot], SparseSlot, PROT_READ, SparseSlot); kept_view != nullptr) {
            pack_keep_compared.fetch_add(1, std::memory_order_relaxed);
            const auto* a = static_cast<const std::uint64_t*>(view);
            const auto* b = static_cast<const std::uint64_t*>(kept_view);
            for (std::size_t i = 0; i < SparseSlot / 8; ++i)
                if (a[i] != b[i]) {
                    if (pack_keep_differed.fetch_add(1, std::memory_order_relaxed) < 8) {
                        char line[200];
                        const int n = std::snprintf(line, sizeof(line),
                            "EDEN_CRAM_PACK_KEEP slot %zu changed while packed: word %zu unpacked %#llx kept %#llx\n",
                            slot, i, (unsigned long long)a[i], (unsigned long long)b[i]);
                        if (n > 0) Note(line);
                    }
                    break;
                }
            UnmapView(kept_view, SparseSlot);
        }
        (void)sceKernelReleaseDirectMemory(pack_kept[slot], SparseSlot);
        pack_kept[slot] = -1;
    }
    UnmapView(view, SparseSlot);
    const bool clean = from_swap && !for_write;
    if (!MapSlot(at, clean ? PROT_READ : PROT_READ | PROT_WRITE, physical)) {
        (void)sceKernelReleaseDirectMemory(physical, SparseSlot);
        return false;
    }
    range.physical[slot] = physical;
    if (!clean) ForgetPacked(slot); // a Clean slot's entry stays: the file keeps the copy
    NoteTouched(slot);
    guest_committed.fetch_add(SparseSlot, std::memory_order_relaxed);
    unpacks.fetch_add(1, std::memory_order_relaxed);
    *to = clean ? SlotClean : SlotOwn;
    return true;
#else
    (void)range;
    (void)slot;
    (void)to;
    (void)for_write;
    return false; // the host never packs
#endif
}

bool AllZero(std::uintptr_t at) noexcept {
    const auto* word = reinterpret_cast<const volatile std::uint64_t*>(at);
    for (std::size_t i = 0; i < SparseSlot / 8; i += 8) {
        if (word[i] | word[i + 1] | word[i + 2] | word[i + 3] | word[i + 4] | word[i + 5] | word[i + 6] | word[i + 7])
            return false;
    }
    return true;
}

bool PageZero(std::uintptr_t at) noexcept {
    const auto* word = reinterpret_cast<const volatile std::uint64_t*>(at);
    for (std::size_t i = 0; i < FinePage / 8; i += 8)
        if (word[i] | word[i + 1] | word[i + 2] | word[i + 3] | word[i + 4] | word[i + 5] | word[i + 6] | word[i + 7])
            return false;
    return true;
}

// Fine -> Own: a written Fine slot becomes one block of 2 MiB again (a copy of what it shows), and
// stays whole for the rest of the session.
bool PromoteFine(SparseRange& range, std::size_t slot) noexcept {
    DropSlotAliases(range, slot);
    const std::uintptr_t at = SlotAddress(range, slot);
    const std::size_t kept = FineKept(range, slot);
#ifdef PS5_NATIVE
    std::int64_t physical = -1;
    if (eden_direct_low(SparseSlot, SparseSlot, 12, &physical) != 0 && !ReserveBlock(&physical)) return false;
    void* const view = MapView(physical, SparseSlot, PROT_READ | PROT_WRITE, SparseSlot);
    if (view == nullptr) {
        (void)sceKernelReleaseDirectMemory(physical, SparseSlot);
        return false;
    }
    std::memcpy(view, reinterpret_cast<const void*>(at), SparseSlot);
    UnmapView(view, SparseSlot);
    if (!MapSlot(at, PROT_READ | PROT_WRITE, physical)) {
        (void)sceKernelReleaseDirectMemory(physical, SparseSlot);
        return false;
    }
    if (kept != 0 && sceKernelReleaseDirectMemory(range.physical[slot], kept * FinePage) != 0) std::abort();
    range.physical[slot] = physical;
#else
    if (mprotect(reinterpret_cast<void*>(at), SparseSlot, PROT_READ | PROT_WRITE) != 0) return false;
#endif
    guest_committed.fetch_add(SparseSlot - kept * FinePage, std::memory_order_relaxed);
    const unsigned times = std::min<unsigned>(((range.fine[slot] >> 8) & 0xf) + 1, FineBackoffMax);
    range.fine[slot] = static_cast<std::uint16_t>(times << 8);
    range.fine_after[slot] = static_cast<std::uint32_t>(counters.passes.load(std::memory_order_relaxed)) + (1u << times);
    counters.unfined.fetch_add(1, std::memory_order_relaxed);
    return true;
}

#ifdef PS5_NATIVE
bool MapRun(std::uintptr_t at, std::size_t bytes, std::int64_t physical) noexcept {
    void* address = reinterpret_cast<void*>(at);
    return sceKernelMapDirectMemory(&address, bytes, PROT_READ, MAP_FIXED, physical, FinePage) == 0 &&
           address == reinterpret_cast<void*>(at);
}
#endif

// Own -> Fine (the slot is locked and fenced, and has `zero` marked in a mask of its pages): its other
// pages go into a compact block, and the slot is mapped again page run by page run, read-only.
// The 2 MiB mapping is taken away whole first (a large mapping is never split); a thread that
// touches the slot meanwhile faults, waits for the lock and repeats its access.
bool MakeFine(SparseRange& range, std::size_t slot, const bool (&zero)[FinePages], std::size_t kept) noexcept {
    const std::uintptr_t at = SlotAddress(range, slot);
#ifdef PS5_NATIVE
    std::int64_t compact = -1;
    if (eden_direct_low(kept * FinePage, FinePage, 12, &compact) != 0) return false;
    void* const view = MapView(compact, kept * FinePage, PROT_READ | PROT_WRITE, FinePage);
    if (view == nullptr) {
        (void)sceKernelReleaseDirectMemory(compact, kept * FinePage);
        return false;
    }
    std::size_t next = 0;
    for (std::size_t page = 0; page < FinePages; ++page)
        if (!zero[page]) {
            std::memcpy(static_cast<std::uint8_t*>(view) + next * FinePage,
                        reinterpret_cast<const void*>(at + page * FinePage), FinePage);
            ++next;
        }
    UnmapView(view, kept * FinePage);
    const std::int64_t whole = range.physical[slot];
    if (munmap(reinterpret_cast<void*>(at), SparseSlot) != 0) std::abort();
    bool ok = true;
    next = 0;
    for (std::size_t page = 0; page < FinePages && ok;) {
        std::size_t end = page + 1;
        while (end < FinePages && zero[end] == zero[page]) ++end;
        const std::size_t bytes = (end - page) * FinePage;
        ok = MapRun(at + page * FinePage, bytes, zero[page] ? zero_block : compact + std::int64_t(next * FinePage));
        if (!zero[page]) next += end - page;
        page = end;
    }
    if (!ok) {
        // The slot shows its block again, as it was (fenced: read-only until the caller unfences).
        if (!MapSlot(at, PROT_READ, whole)) std::abort();
        (void)sceKernelReleaseDirectMemory(compact, kept * FinePage);
        return false;
    }
    if (sceKernelReleaseDirectMemory(whole, SparseSlot) != 0) std::abort();
    range.physical[slot] = compact;
#else
    for (std::size_t page = 0; page < FinePages; ++page)
        if (zero[page] && madvise(reinterpret_cast<void*>(at + page * FinePage), FinePage, MADV_DONTNEED) != 0)
            std::abort();
#endif
    guest_committed.fetch_sub(SparseSlot - kept * FinePage, std::memory_order_relaxed);
    range.fine[slot] = static_cast<std::uint16_t>((range.fine[slot] & 0xf00) | kept);
    counters.fined.fetch_add(1, std::memory_order_relaxed);
    return true;
}

// CommitSparsePage for a range with slot states.
bool ServiceCommit(SparseRange& range, std::size_t slot) noexcept {
    LockSlot(range, slot);
    const std::uint8_t state = SlotState(range, slot);
    // The block is wanted for writing: an Idle or Clean slot is Own again, without its copy in the swap file.
    std::uint8_t to = SlotOwn;
    const bool ok = state == SlotZero ? PromoteZero(range, slot)
                  : state == SlotFine ? PromoteFine(range, slot)
                  : state == SlotPacked ? UnpackSlot(range, slot, &to, true)
                  : state == SlotIdle || state == SlotClean ? MakeDirty(range, slot) : true;
    UnlockSlot(range, slot,
               ok && (state == SlotZero || state == SlotFine || state == SlotPacked || state == SlotIdle || state == SlotClean)
                   ? SlotOwn : state);
    return ok;
}

// ---- The fault ----
SparseRange* WriteRangeOf(std::uintptr_t address) noexcept {
    for (SparseRange* range : {service_range.load(std::memory_order_acquire), probe_range.load(std::memory_order_acquire)}) {
        if (range == nullptr || range->state == nullptr) continue;
        const auto begin = range->begin.load(std::memory_order_acquire);
        if (begin != 0 && address >= begin && address < range->end) return range;
    }
    return nullptr;
}
std::atomic<std::uintptr_t> probe_expected{0}; // the self-test's write, whatever si_addr says
std::atomic<bool> probe_mismatch{false};
// The last write the service refused, for the crash report: the address, then why (1: it faulted
// again and again with nothing to change, 2: no block was left for its slot) and the slot's state.
std::atomic<std::uintptr_t> refused_address{0};
std::atomic<unsigned> refused_detail{0}; // why | state << 8
void Refused(std::uintptr_t address, unsigned why, std::uint8_t state) noexcept {
    refused_detail.store(why | unsigned{state} << 8, std::memory_order_relaxed);
    refused_address.store(address, std::memory_order_release);
}
} // namespace

// The last write the service refused (headless/crash_report.cpp): its address, 0 for none.
extern "C" std::uintptr_t eden_guest_write_refused(unsigned* why, unsigned* state) noexcept {
    const std::uintptr_t address = refused_address.load(std::memory_order_acquire);
    const unsigned detail = refused_detail.load(std::memory_order_relaxed);
    *why = detail & 0xff;
    *state = detail >> 8;
    return address;
}

#ifdef PS5_NATIVE
extern "C" const char __eden_text_start[] __attribute__((weak));
extern "C" const char __eden_text_end[] __attribute__((weak));
#endif

namespace {
// Who writes the slots that get promoted. A fault handler (ServiceSignal) reads the interrupted
// instruction's address and the first return address of the app's code above the stack pointer:
// a write by the emulated CPU has the JIT's entry point there, a write by the emulator itself the
// function that called the copy. The promotions are counted by that place (ServiceReport).
constexpr std::size_t WriterPcs = 4;
struct WriterSite {
    std::atomic<std::uintptr_t> key{0};
    std::uintptr_t pcs[WriterPcs]{};
    std::atomic<std::uint64_t> count{0};
};
constexpr std::size_t WriterSites = 512;
WriterSite writer_sites[WriterSites];
std::atomic<std::uint64_t> promotions_in_app_code{0}, promotions_elsewhere{0};
thread_local std::uintptr_t fault_pcs[WriterPcs] = {};
thread_local bool fault_in_app_code = false;

void NoteWriter() noexcept {
    (fault_in_app_code ? promotions_in_app_code : promotions_elsewhere).fetch_add(1, std::memory_order_relaxed);
    std::uintptr_t key = 0xCBF29CE484222325ull;
    for (const std::uintptr_t pc : fault_pcs) key = (key ^ pc) * 0x100000001B3ull;
    key |= 1;
    std::size_t at = static_cast<std::size_t>((key >> 12) % WriterSites);
    for (std::size_t probe = 0; probe < 16; ++probe, at = (at + 1) % WriterSites) {
        std::uintptr_t expected = 0;
        if (writer_sites[at].key.compare_exchange_strong(expected, key, std::memory_order_acq_rel)) {
            for (std::size_t i = 0; i < WriterPcs; ++i) writer_sites[at].pcs[i] = fault_pcs[i];
            writer_sites[at].count.fetch_add(1, std::memory_order_relaxed);
            return;
        }
        if (expected == key) {
            writer_sites[at].count.fetch_add(1, std::memory_order_relaxed);
            return;
        }
    }
}
} // namespace

namespace {
// Writers waiting for a block (WaitForBlock), and how many waits ended with one.
std::atomic<unsigned> memory_waiters{0};
std::atomic<std::uint64_t> memory_waits_won{0}, memory_waits_lost{0};

// No block was left for a written slot. Most of what the pool gives out is held by the graphics
// caches, which free what is not drawn once they see memory short (headless/performance.cpp reads
// eden_guest_memory_wanted, and while a writer waits the caches see no free memory at all). The
// game's own memory comes first: a large open-world game wrote 1.8 GB of new memory within 10 s
// after 23 minutes of play, faster than the caches free at the end of each frame, and the write
// that found no block ended the session. So the writer waits here, in its fault handler, and tries
// again every 20 ms for up to 2 s while the caches give memory back. Only then is it refused.
std::size_t RecoverMemory(std::size_t bytes, bool wait, bool driver, int keeper) noexcept;
template <typename Promote>
bool WaitForBlock(Promote&& promote) noexcept {
    memory_waiters.fetch_add(1, std::memory_order_acq_rel);
    bool ok = false;
    for (unsigned tries = 0; tries < 100 && !ok; ++tries) {
        // Slots nothing touches give their blocks up first (packed, or to the swap file): without
        // waiting for a pass that is packing, and without the driver (this is a fault handler).
        if (RecoverMemory(2 * SparseSlot, false, false, 0) == 0) {
            const timespec pause{0, 20'000'000};
            nanosleep(&pause, nullptr);
        }
        ok = promote();
    }
    memory_waiters.fetch_sub(1, std::memory_order_acq_rel);
    (ok ? memory_waits_won : memory_waits_lost).fetch_add(1, std::memory_order_relaxed);
    Note(ok ? "EDEN_CRAM a written slot waited for the graphics caches to give memory back, and got a block\n"
            : "EDEN_CRAM a written slot waited 2 s for memory and found no block\n");
    return ok;
}
} // namespace

// How many writes of the guest's memory are waiting for a block now (headless/performance.cpp):
// while any is, the graphics caches are told no memory is free.
// A page of the guest's memory the GPU's caches now hold (+1) or no longer hold (-1), from the core's
// device memory manager. Counted per slot; packing skips slots with any (PackSlot).
extern "C" void eden_guest_gpu_cached(const void* host, int delta) noexcept {
    SparseRange* const range = guest_range.load(std::memory_order_acquire);
    if (range == nullptr) return;
    const std::uintptr_t begin = range->begin.load(std::memory_order_relaxed);
    const auto at = reinterpret_cast<std::uintptr_t>(host);
    if (at < begin || at >= range->end) return;
    auto* table = gpu_cached_table.load(std::memory_order_acquire);
    if (table == nullptr) {
        const std::size_t slots = (range->end - begin) / SparseSlot;
        auto* made = new (std::nothrow) std::atomic<std::int32_t>[slots]();
        if (made == nullptr) return;
        gpu_cached_slots = slots;
        if (gpu_cached_table.compare_exchange_strong(table, made, std::memory_order_acq_rel)) table = made;
        else delete[] made;
    }
    const std::size_t slot = (at - begin) / SparseSlot;
    if (slot < gpu_cached_slots) table[slot].fetch_add(delta, std::memory_order_acq_rel);
}

extern "C" unsigned eden_guest_memory_wanted(void) noexcept {
    return memory_waiters.load(std::memory_order_acquire);
}

// Last resort for a host heap piece, after the direct pool refused even 2 MiB.
// Transfer ownership under the reserve lock: never release then race the keeper
// to reacquire the same memory. The host's committed-block table owns/frees it.
// Leave at least half the configured reserve for guest write faults, and give
// waiting guest writers priority. This does not change the guest's own fallback.
bool TakeHostReserveBlock(std::int64_t* physical) noexcept {
#ifdef PS5_NATIVE
    const char* enabled = std::getenv("EDEN_HOST_RESERVE_TRANSFER");
    if (!physical || !enabled || std::strcmp(enabled, "1") != 0) return false;
    Spin(reserve_lock);
    const auto count = reserve_count.load(std::memory_order_relaxed);
    const bool allowed = service_range.load(std::memory_order_acquire) != nullptr &&
        memory_waiters.load(std::memory_order_acquire) == 0 && count > ReserveTarget() / 2;
    if (allowed) {
        *physical = reserve[count - 1];
        reserve_count.store(count - 1, std::memory_order_relaxed);
    }
    reserve_lock.clear(std::memory_order_release);
    if (allowed) counters.host_reserve_used.fetch_add(1, std::memory_order_relaxed);
    return allowed;
#else
    (void)physical;
    return false;
#endif
}

// The guest's headroom (the reserve): what it holds and what it is to hold, in bytes; both 0 while
// the service does not run or keeps no reserve (the host). headless/performance.cpp counts what is
// not held yet as taken, as all of it was before.
extern "C" void eden_guest_memory_reserve(std::size_t* held, std::size_t* target) noexcept {
#ifdef PS5_NATIVE
    const bool running = service_range.load(std::memory_order_acquire) != nullptr;
    *held = running ? reserve_count.load(std::memory_order_relaxed) * SparseSlot : 0;
    *target = running ? ReserveTarget() * SparseSlot : 0;
#else
    *held = 0;
    *target = 0;
#endif
}

// How the waits ended so far, and the writes refused for want of a block (headless/performance.cpp
// ReportMemory prints them beside what holds the pool).
extern "C" void eden_guest_memory_waits(unsigned long long* won, unsigned long long* lost,
                                        unsigned long long* refused) noexcept {
    *won = memory_waits_won.load(std::memory_order_relaxed);
    *lost = memory_waits_lost.load(std::memory_order_relaxed);
    *refused = counters.failures.load(std::memory_order_relaxed);
}

// A fault at `address`: true when it was an access to a slot of the guest's memory that can now
// succeed (the faulting instruction repeats), false for anything else. `write`: 1 a write, 0 a
// read, -1 not known (taken as a write).
bool HandleGuestWriteFault(void* fault, int write) noexcept {
    auto address = reinterpret_cast<std::uintptr_t>(fault);
    if (const auto expected = probe_expected.load(std::memory_order_acquire); expected != 0 && address != expected) {
        probe_mismatch.store(true, std::memory_order_relaxed);
        address = expected;
    }
    SparseRange* const range = WriteRangeOf(address);
    if (range == nullptr) return false;
    const std::size_t slot = (address - range->begin.load(std::memory_order_relaxed)) / SparseSlot;
    // The same address faulting again and again with nothing to change is not a slot to promote
    // but a real fault. A promotion starts the count again (below).
    static thread_local std::uintptr_t last = 0;
    static thread_local unsigned repeats = 0;
    repeats = address == last ? repeats + 1 : 1;
    last = address;
    if (repeats > 16) {
        repeats = 0;
        Refused(address, 1, SlotState(*range, slot));
        Note("EDEN_CRAM refused a write: 17 faults in a row at one address, its slot unchanged\n");
        return false;
    }
    counters.faults.fetch_add(1, std::memory_order_relaxed);
    const bool waited = LockSlot(*range, slot, true);
    const std::uint8_t state = SlotState(*range, slot);
    if (waited) {
        // Someone changed the slot meanwhile (or is mapping it again): the access may now succeed,
        // and a read must never make a Fine slot whole. It faults again if it was a write.
        UnlockSlot(*range, slot, state);
        return true;
    }
    std::uint8_t to = SlotOwn;
    bool kept_clean = false;
    const auto promote = [&]() noexcept {
        bool done = true;
        if (state == SlotFine) {
            done = PromoteFine(*range, slot);
        } else if (state == SlotZero) {
            done = PromoteZero(*range, slot);
            if (done) {
                counters.promotions.fetch_add(1, std::memory_order_relaxed);
                NoteWriter();
            }
        } else if (state == SlotShared) {
            done = PromoteShared(*range, slot);
        } else if (state == SlotPacked) {
            done = UnpackSlot(*range, slot, &to);
            if (done) CountUnpacker(slot);
        } else if (state == SlotIdle) {
            to = WakeIdle(*range, slot); // any access: Own again, or Clean (a write there faults once more)
        } else if (state == SlotClean) {
            // Only a write faults on a Clean slot. A read that got here faulted while the slot was
            // Idle, and another thread woke it first: it can be read as it is, and keeps its copy
            // in the swap file (dropped for a read, the slot would also count as written again).
            // Should the fault's write bit ever be wrong, the access faults here again at once,
            // and is then taken for the write it is.
            if (write == 0 && repeats > 1) clean_refaults.fetch_add(1, std::memory_order_relaxed);
            if (write != 0 || repeats > 1) done = MakeDirty(*range, slot);
            else to = SlotClean, kept_clean = true, clean_late_reads.fetch_add(1, std::memory_order_relaxed);
        }
        return done;
    };
    bool ok = promote();
    if (!ok) ok = WaitForBlock(promote);
    UnlockSlot(*range, slot, ok ? to : state);
    if (!ok) {
        counters.failures.fetch_add(1, std::memory_order_relaxed);
        Refused(address, 2, state);
        Note("EDEN_CRAM out of memory: a written slot of the guest's memory found no block\n");
    } else if (state != SlotOwn && !kept_clean) {
        // The slot was made writable. A game that writes one place each time its slot was given
        // back (cleared, freed or scanned) faults there again and again over a session, each time
        // with something to do: counted as repeats, the 17th such write ended the game after
        // 15 to 60 minutes (Swordpdf/ProsperoEden#246).
        repeats = 0;
    }
    return ok;
}

namespace {
struct sigaction service_previous_segv{}, service_previous_bus{};

void ServiceSignal(int signal, siginfo_t* info, void* context) {
#ifdef PS5_NATIVE
    fault_in_app_code = false;
    for (std::uintptr_t& pc : fault_pcs) pc = 0;
    if (context != nullptr) {
        const auto low = reinterpret_cast<std::uintptr_t>(__eden_text_start) + 8;
        const auto high = reinterpret_cast<std::uintptr_t>(__eden_text_end);
        const std::uintptr_t rip = Eden::Fastmem::ContextRip(context);
        fault_in_app_code = rip >= low && rip < high;
        const auto* stack = reinterpret_cast<const std::uintptr_t*>(Eden::Fastmem::ContextRsp(context));
        std::size_t found = 0;
        // Never past the page (16 KiB) that holds the stack pointer: a thread's stack can end a few
        // hundred bytes above it, and what lies after it is not mapped.
        const auto page_end = ((reinterpret_cast<std::uintptr_t>(stack) | 0x3FFFu) + 1);
        const auto room = static_cast<unsigned>((page_end - reinterpret_cast<std::uintptr_t>(stack)) / sizeof(std::uintptr_t));
        for (unsigned i = 0; i < 160 && i < room && found < WriterPcs; ++i) {
            if (stack[i] >= low && stack[i] < high && (found == 0 || fault_pcs[found - 1] != stack[i])) {
                fault_pcs[found++] = stack[i];
            }
        }
    }
#endif
    if (HandleGuestWriteFault(info ? info->si_addr : nullptr, context ? Eden::Fastmem::ContextFaultWrite(context) : -1)) return;
    const struct sigaction& previous = signal == SIGSEGV ? service_previous_segv : service_previous_bus;
    if (previous.sa_flags & SA_SIGINFO) {
        previous.sa_sigaction(signal, info, context);
    } else if (previous.sa_handler == SIG_DFL) {
        std::signal(signal, SIG_DFL); // the faulting instruction repeats and ends the process
    } else if (previous.sa_handler != SIG_IGN) {
        previous.sa_handler(signal);
    }
}

bool InstallServiceSignal() noexcept {
    static std::once_flag once;
    static bool installed = false;
    std::call_once(once, [] {
        struct sigaction action{};
        action.sa_sigaction = ServiceSignal;
        action.sa_flags = SA_SIGINFO | SA_RESTART;
        sigemptyset(&action.sa_mask);
        installed = sigaction(SIGSEGV, &action, &service_previous_segv) == 0 &&
                    sigaction(SIGBUS, &action, &service_previous_bus) == 0;
    });
    return installed;
}

// Gives a range slot states (a guest range, or the self-test's).
bool AddStates(SparseRange& range) noexcept {
    const std::size_t slots = (range.end - range.begin.load(std::memory_order_relaxed)) / SparseSlot;
    auto* state = new (std::nothrow) std::atomic<std::uint8_t>[slots];
    auto* fine = state ? new (std::nothrow) std::uint16_t[slots]() : nullptr;
    auto* fine_after = fine ? new (std::nothrow) std::uint32_t[slots]() : nullptr;
    auto* aliases = fine_after && range.guest ? new (std::nothrow) std::uintptr_t[slots * AliasesPerSlot]() : nullptr;
    if (fine_after == nullptr || (range.guest && aliases == nullptr)) {
        delete[] state;
        delete[] fine;
        delete[] fine_after;
        return false;
    }
    range.fine = fine;
    range.fine_after = fine_after;
    range.aliases = aliases;
    for (std::size_t slot = 0; slot < slots; ++slot)
        state[slot].store(range.owned[slot].load(std::memory_order_relaxed) ? SlotOwn : SlotZero, std::memory_order_relaxed);
    range.state = state;
    return true;
}

// ---- The scanner ----
std::thread scanner;
std::mutex scanner_mutex;
std::condition_variable scanner_wake;
bool scanner_stop = false;
#ifdef PS5_NATIVE
// The reserve kept full: every 100 ms, so a burst of writes that used it finds it whole again as
// soon as the caches have given memory back (the scanner only passes every 20 s).
std::thread keeper;

void KeepFree() noexcept; // below: slots nothing touches give their blocks up while the pool is nearly empty
void ReserveKeeper() {
    std::unique_lock lock{scanner_mutex};
    for (unsigned tick = 1; !scanner_stop; ++tick) {
        scanner_wake.wait_for(lock, std::chrono::milliseconds(100), [] { return scanner_stop; });
        if (scanner_stop) break;
        lock.unlock();
        FillReserve();
        if (tick % 2 == 0) KeepFree();
        lock.lock();
    }
}
#endif

std::uint64_t Mix(std::uint64_t hash, std::uint64_t word) noexcept {
    hash ^= word + 0x9e3779b97f4a7c15ull + (hash << 6) + (hash >> 2);
    return hash * 0xff51afd7ed558ccdull;
}
std::uint64_t SampleHash(std::uintptr_t at) noexcept {
    const auto* word = reinterpret_cast<const volatile std::uint64_t*>(at);
    std::uint64_t hash = 0;
    for (std::size_t i = 0; i < SparseSlot / 8; i += SparseSlot / 8 / 64) hash = Mix(hash, word[i] ^ word[i + 1]);
    return hash;
}
std::uint64_t FullHash(std::uintptr_t at, std::size_t bytes) noexcept {
    const auto* word = reinterpret_cast<const volatile std::uint64_t*>(at);
    std::uint64_t hash = 0;
    for (std::size_t i = 0; i < bytes / 8; ++i) hash = Mix(hash, word[i]);
    return hash;
}

void Fence(std::uintptr_t at) noexcept {
    // A window alias would write past the fence.
    if (SparseRange* range = guest_range.load(std::memory_order_acquire); range && range->aliases &&
        at >= range->begin.load(std::memory_order_relaxed) && at < range->end)
        DropSlotAliases(*range, (at - range->begin.load(std::memory_order_relaxed)) / SparseSlot);
    if (mprotect(reinterpret_cast<void*>(at), SparseSlot, PROT_READ) != 0) std::abort();
}
void Unfence(std::uintptr_t at) noexcept {
    if (mprotect(reinterpret_cast<void*>(at), SparseSlot, PROT_READ | PROT_WRITE) != 0) std::abort();
}

// An Own slot that holds only zeroes goes back. Fenced while it is looked at again.
bool ScanZero(SparseRange& range, std::size_t slot) noexcept {
    LockSlot(range, slot);
    if (SlotState(range, slot) != SlotOwn) {
        UnlockSlot(range, slot, SlotState(range, slot));
        return false;
    }
    const std::uintptr_t at = SlotAddress(range, slot);
    Fence(at);
    const bool zero = AllZero(at);
    if (zero) ReturnSlot(range, slot);
    else Unfence(at);
    UnlockSlot(range, slot, zero ? SlotZero : SlotOwn);
    return zero;
}

// An Own slot with many pages of zeroes becomes Fine. A racy count first, then the count again fenced.
bool ScanFine(SparseRange& range, std::size_t slot) noexcept {
    const std::uintptr_t at = SlotAddress(range, slot);
    if (counters.passes.load(std::memory_order_relaxed) < range.fine_after[slot]) return false;
    std::size_t zeros = 0;
    for (std::size_t page = 0; page < FinePages; ++page) zeros += PageZero(at + page * FinePage);
    if (zeros < FineMinZeroPages) return false;
    LockSlot(range, slot);
    if (SlotState(range, slot) != SlotOwn) {
        UnlockSlot(range, slot, SlotState(range, slot));
        return false;
    }
    Fence(at);
    bool zero[FinePages];
    zeros = 0;
    for (std::size_t page = 0; page < FinePages; ++page) zeros += zero[page] = PageZero(at + page * FinePage);
    bool fined = false;
    if (zeros == FinePages) {
        ReturnSlot(range, slot);
        UnlockSlot(range, slot, SlotZero);
        counters.returns_scan.fetch_add(1, std::memory_order_relaxed);
        return true;
    }
    if (zeros >= FineMinZeroPages) {
        fined = MakeFine(range, slot, zero, FinePages - zeros);
        if (!fined) counters.fine_failures.fetch_add(1, std::memory_order_relaxed);
    }
    if (!fined) Unfence(at);
    UnlockSlot(range, slot, fined ? SlotFine : SlotOwn);
    return fined;
}

// Two slots of the same bytes show one block (the console; the host only counts them).
bool ScanShare(SparseRange& range, std::size_t keep, std::size_t other) noexcept {
    const std::size_t first = std::min(keep, other), second = std::max(keep, other);
    LockSlot(range, first);
    LockSlot(range, second);
    const std::uint8_t keep_state = SlotState(range, keep), other_state = SlotState(range, other);
    bool shared = false;
    if ((keep_state == SlotOwn || keep_state == SlotShared) && other_state == SlotOwn) {
        const std::uintptr_t keep_at = SlotAddress(range, keep), other_at = SlotAddress(range, other);
        if (keep_state == SlotOwn) Fence(keep_at);
        Fence(other_at);
        const bool same = std::memcmp(reinterpret_cast<const void*>(keep_at), reinterpret_cast<const void*>(other_at),
                                      SparseSlot) == 0;
#ifdef PS5_NATIVE
        // A block becoming shared counts both slots (the first ShareBlock makes it 1, the second 2); one
        // already shared counts one more. A table that is full shares nothing.
        bool counted = false;
        if (same && ShareBlock(range.physical[keep])) {
            counted = keep_state == SlotShared || ShareBlock(range.physical[keep]);
            if (!counted) (void)UnshareBlock(range.physical[keep]);
        }
        if (counted) {
            if (!MapSlot(other_at, PROT_READ, range.physical[keep])) std::abort();
            if (sceKernelReleaseDirectMemory(range.physical[other], SparseSlot) != 0) std::abort();
            range.physical[other] = range.physical[keep];
            guest_committed.fetch_sub(SparseSlot, std::memory_order_relaxed);
            shared = true;
        }
#else
        (void)same;
#endif
        if (!shared) {
            if (keep_state == SlotOwn) Unfence(keep_at);
            Unfence(other_at);
        }
        if (same) counters.dedup.fetch_add(1, std::memory_order_relaxed);
    }
    UnlockSlot(range, second, second == keep ? (shared ? SlotShared : keep_state) : (shared ? SlotShared : other_state));
    UnlockSlot(range, first, first == keep ? (shared ? SlotShared : keep_state) : (shared ? SlotShared : other_state));
    return shared;
}

// ---- Packing (see "Packed slots" above) ----
// EDEN_CRAM_PACK=0 turns it off. While the pool's largest free block is under PackBelowLargest,
// each scan pass packs up to PackPerPass slots unwritten for ColdSeconds or more, the
// coldest first. A slot packed before waits twice as many passes each time (it came back), and
// one that does not compress to PackMaxRatio is left as it is.
// On unless EDEN_CRAM_PACK=0 (2026-11-09). It only starts once the pool's largest free block is
// below EDEN_CRAM_PACK_BELOW_MIB, so a game that fits never packs. Without PS5 Pro memory (12,224
// MiB) Mario Kart World ran out at every race load without it; with it (GPU-reachable slots
// included, below) it ran a whole race and into the next (docs/MKW-MEMORY-LEAK.md). It was off from
// 2026-11-08: sessions that packed showed impossible draws and GPU hangs, but those draws turned out
// to come without packing too (the race-load event), and the hangs with GPU threads waiting for
// memory went with the unpack reserve (EDEN_CRAM_UNPACK_KEEP_MIB).
bool PackEnabled() noexcept {
    static const bool on = [] {
        const char* value = std::getenv("EDEN_CRAM_PACK");
        return (value == nullptr || value[0] != '0') && &LZ4_compress_default != nullptr &&
               &LZ4_decompress_safe != nullptr && &LZ4_compressBound != nullptr;
    }();
    return on;
}
constexpr std::size_t PackPerPass = 256;                       // 512 MiB of slots
// EDEN_CRAM_PACK_BELOW_MIB / EDEN_CRAM_PACK_COLD_S: when packing starts (largest free block) and how long
// a slot must stay unwritten (tuning; defaults 3072 MiB and 30 s: MKW without Pro memory needs packing
// to start well before the race load, which takes ~1.5 GB at once).
std::size_t EnvOr(const char* name, std::size_t fallback) noexcept {
    const char* value = std::getenv(name);
    return value != nullptr && *value ? static_cast<std::size_t>(std::strtoull(value, nullptr, 10)) : fallback;
}
std::size_t PackBelowLargest() noexcept {
    static const std::size_t bytes = EnvOr("EDEN_CRAM_PACK_BELOW_MIB", 3072) << 20;
    return bytes;
}
constexpr double PackMaxRatio = 0.6;
std::vector<std::uint8_t> pack_times; // per slot, the scanner's: how often it was packed
std::atomic<bool> pack_refused{false};
std::mutex pack_mutex; // the scanner's cold pass and packing, and eden_memory_recover
thread_local bool holding_pack = false; // this thread holds pack_mutex (a heap growth inside must not wait for it)
std::vector<std::size_t> recover_order; // eden_memory_recover's list, room made once (no allocation there)
std::vector<std::size_t> pack_ready;    // PackPass's, likewise
std::vector<char> pack_scratch;

#ifdef PS5_NATIVE
extern "C" [[gnu::weak]] std::int32_t sceKernelAvailableDirectMemorySize(std::int64_t, std::int64_t, std::size_t, std::int64_t*, std::size_t*);
#endif
bool PoolShortForPacking() noexcept {
#ifdef PS5_NATIVE
    const auto total = sceKernelGetDirectMemorySize();
    std::int64_t start = 0;
    std::size_t largest = 0;
    return total > 0 && &sceKernelAvailableDirectMemorySize != nullptr &&
           sceKernelAvailableDirectMemorySize(0, total, SparseSlot, &start, &largest) == 0 &&
           largest < PackBelowLargest();
#else
    return false;
#endif
}

// What the pool has free in aligned 2 MiB pieces (what a guest slot, the driver and the heap take).
// 0 when it cannot be read.
#ifdef PS5_NATIVE
struct PoolRegion { std::int64_t start; std::int64_t end; std::int32_t type; };
extern "C" [[gnu::weak]] std::int32_t sceKernelDirectMemoryQuery(std::int64_t, int, PoolRegion*, std::size_t);
#endif
std::size_t PoolFreePieces() noexcept {
#ifdef PS5_NATIVE
    const std::int64_t total = sceKernelGetDirectMemorySize();
    if (total <= 0 || &sceKernelDirectMemoryQuery == nullptr) return 0;
    const auto pieces = [](std::int64_t from, std::int64_t to) noexcept {
        const std::int64_t first = (from + std::int64_t(SparseSlot) - 1) / std::int64_t(SparseSlot);
        const std::int64_t last = to / std::int64_t(SparseSlot);
        return last > first ? static_cast<std::size_t>(last - first) * SparseSlot : std::size_t{0};
    };
    std::size_t free_bytes = 0;
    std::int64_t offset = 0;
    unsigned regions = 0;
    PoolRegion region{};
    while (offset < total && regions < 16384 && sceKernelDirectMemoryQuery(offset, 1, &region, sizeof(region)) == 0 &&
           region.end > offset) {
        free_bytes += pieces(offset, region.start);
        offset = region.end;
        ++regions;
    }
    if (regions < 16384) free_bytes += pieces(offset, total);
    return free_bytes;
#else
    return 0;
#endif
}
// Idle slots (above): how long a slot must stay untouched before it is packed, how long a touched
// slot is left alone (doubled each time it was touched, up to 8 times: ProbeSlot), and how many
// slots a pass may make Idle (2 GiB).
std::size_t IdleSeconds() noexcept {
    static const std::size_t seconds = EnvOr("EDEN_CRAM_IDLE_S", 10);
    return seconds;
}
std::size_t IdleBackoff() noexcept {
    static const std::size_t seconds = EnvOr("EDEN_CRAM_IDLE_BACKOFF_S", 5);
    return seconds;
}
constexpr std::size_t IdlePerPass = 1024;

// (ColdPass, below: a sampled hash of each Own slot, and when it last changed.)
std::vector<std::uint64_t> cold_hash;
std::vector<std::uint32_t> cold_since; // per Own slot: when its hash last changed (s, steady clock)

// Slots that did not compress when last looked at, until they are written again (ColdPass sees their
// sampled hash change): LZ4 is not tried on them again where they could not go to the swap file anyway.
std::uint8_t* pack_raw = nullptr;
// A recovery is packing (pack_mutex held): a packed slot's block goes straight back to the pool, not
// to the blocks kept for unpacking, and pack_released tells whether it did (the store may have taken
// it instead). The driver asked 32 MiB back, was told 32 MiB, and found none: the blocks had gone to
// refill what unpacks had used, and its allocation ended the session.
bool recovering = false, pack_released = false;
std::atomic<std::uint64_t> swap_last_resort{0}; // slots written to the file as a request's last resort (RecoverMemory)
enum class Swap { No, Reuse, Yes, Urgent }; // may the slot go to the swap file: never; only if the file has its bytes already; ...

// Idle -> Packed (Own, without Idle slots): the scanner, or a recovery (pack_mutex held). Readable
// and not writable while it is compressed, then the zero block with no access.
bool PackSlot(SparseRange& range, std::size_t slot, Swap swap = Swap::No) noexcept {
#ifdef PS5_NATIVE
    PackedSlot* const table = packed_slots.load(std::memory_order_acquire);
    if (table == nullptr) return false;
    static const bool pack_gpu = [] {
        const char* value = std::getenv("EDEN_CRAM_PACK_GPU");
        return value == nullptr || value[0] != '0';
    }();
    if (auto* gpu_cached = gpu_cached_table.load(std::memory_order_acquire);
        !pack_gpu && gpu_cached != nullptr && slot < gpu_cached_slots &&
        gpu_cached[slot].load(std::memory_order_acquire) > 0) {
        pack_gpu_skips.fetch_add(1, std::memory_order_relaxed);
        return false;
    }
    if (!pack_gpu && gpu_touched != nullptr && slot < gpu_cached_slots &&
        gpu_touched[slot].load(std::memory_order_acquire) != 0) {
        pack_gpu_skips.fetch_add(1, std::memory_order_relaxed);
        return false;
    }
    if (!TryLockSlot(range, slot)) return false;
    // Only a slot that is still what it was chosen for: Idle, where slots are made Idle (one touched
    // since its caller listed it is in use again: Own or Clean), else Own.
    const std::uint8_t before = SlotState(range, slot);
    if (before != (IdleEnabled() && idle_since != nullptr ? SlotIdle : SlotOwn)) {
        UnlockSlot(range, slot, before);
        return false;
    }
    const std::uintptr_t at = SlotAddress(range, slot);
    const int access_before = before == SlotOwn ? PROT_READ | PROT_WRITE : PROT_NONE;
    // The slot stays as it was.
    const auto leave = [&](bool skipped) noexcept {
        if (mprotect(reinterpret_cast<void*>(at), SparseSlot, access_before) != 0) std::abort();
        UnlockSlot(range, slot, before);
        if (skipped) pack_skips.fetch_add(1, std::memory_order_relaxed);
        return false;
    };
    const bool in_swap = table[slot].swap != 0; // Idle from Clean: the file has its bytes
    if (in_swap && swap == Swap::No) {
        UnlockSlot(range, slot, before);
        return false;
    }
    int size = 0;
    void* data = nullptr;
    bool to_swap = in_swap;
    std::size_t unit = SwapMaxUnits, stored = 0;
    std::uint64_t hash = 0;
    if (!in_swap) {
        // Known not to compress: nothing to do unless it may go to the swap file and the budget has
        // room for the least it could take (asked before LZ4 runs once more, 1 ms a slot).
        if (pack_raw != nullptr && pack_raw[slot] != 0 &&
            ((swap != Swap::Yes && swap != Swap::Urgent) || !SwapOn() || !SwapBudget(SparseSlot / 2, swap == Swap::Urgent))) {
            UnlockSlot(range, slot, before);
            pack_skips.fetch_add(1, std::memory_order_relaxed);
            return false;
        }
        // Readable, not writable, while it is compressed (and written to the file).
        if (before == SlotOwn) Fence(at);
        else if (mprotect(reinterpret_cast<void*>(at), SparseSlot, PROT_READ) != 0) std::abort();
        size = LZ4_compress_default(reinterpret_cast<const char*>(at), pack_scratch.data(),
                                    static_cast<int>(SparseSlot), static_cast<int>(pack_scratch.size()));
        const bool compresses = size > 0 && size <= static_cast<int>(SparseSlot * PackMaxRatio);
        if (pack_raw != nullptr) pack_raw[slot] = compresses ? 0 : 1;
        if (!compresses) {
            // Not for the packed store. To the swap file when it may, while the slot can still be read.
            if ((swap != Swap::Yes && swap != Swap::Urgent) || !SwapOn()) return leave(true);
            const bool packed = size > 0 && static_cast<std::size_t>(size) <= SwapPackMax;
            stored = packed ? static_cast<std::size_t>(size) : SparseSlot;
            if (!SwapBudget(stored, swap == Swap::Urgent)) return leave(true);
            unit = SwapAlloc(SwapUnitsOf(stored));
            if (unit == SwapMaxUnits) {
                swap_full.fetch_add(1, std::memory_order_relaxed);
                return leave(true);
            }
            hash = FullHash(at, SparseSlot);
            if (!SwapWrite(packed ? static_cast<const void*>(pack_scratch.data()) : reinterpret_cast<const void*>(at), stored,
                           std::uint64_t{unit} * SwapUnit)) {
                // The storage is full or failing: nothing more is written. The slot stays as it was.
                SwapFree(unit, SwapUnitsOf(stored));
                SwapOff("a write failed");
                return leave(true);
            }
            if (!packed) swap_raw.fetch_add(1, std::memory_order_relaxed);
            if (slot < cold_since.size()) NoteSwapWrite(slot, ColdNow() - cold_since[slot]);
            to_swap = true;
        } else if (!StoreStart()) {
            return leave(true);
        }
    }
    DropSlotAliases(range, slot);
    // No access first: from here every reader faults and waits for the slot. Mapped readable even
    // for a moment, the zero block would show zeroes where the guest's data was (a pointer read
    // there made a guest thread jump to address 0).
    if (mprotect(reinterpret_cast<void*>(at), SparseSlot, PROT_NONE) != 0) std::abort();
    if (!MapSlot(at, PROT_NONE, zero_block)) {
        // The system will not map with no access: the slot stays as it was, and packing stops.
        if (unit != SwapMaxUnits) SwapFree(unit, SwapUnitsOf(stored));
        pack_refused.store(true, std::memory_order_relaxed);
        Note("EDEN_CRAM_PACK off: the system refused a no-access mapping\n");
        return leave(false);
    }
    // The slot's block, mapped nowhere now: the store may take it when the pool has no block.
    std::int64_t donate = PackKeep() && pack_kept != nullptr ? -1 : range.physical[slot];
    if (!to_swap) {
        data = StoreAlloc(static_cast<std::size_t>(size), &donate);
        if (data == nullptr) {
            // No room anywhere (the store's address space is full): the block, untouched, comes back.
            if (!MapSlot(at, access_before, range.physical[slot])) std::abort();
            UnlockSlot(range, slot, before);
            pack_skips.fetch_add(1, std::memory_order_relaxed);
            return false;
        }
        std::memcpy(data, pack_scratch.data(), static_cast<std::size_t>(size));
    }
    pack_released = false;
    if (PackKeep() && pack_kept != nullptr) {
        pack_kept[slot] = range.physical[slot];
    } else if (donate >= 0 && (recovering || !KeepForUnpack(donate))) {
        if (sceKernelReleaseDirectMemory(donate, SparseSlot) != 0) std::abort();
        pack_released = true;
    }
    range.physical[slot] = -1;
    guest_committed.fetch_sub(SparseSlot, std::memory_order_relaxed);
    if (in_swap) {
        swap_reused.fetch_add(1, std::memory_order_relaxed); // its entry stays: nothing was written
    } else if (to_swap) {
        table[slot] = {nullptr, static_cast<std::uint32_t>(stored), static_cast<std::uint32_t>(unit + 1), hash};
        swap_slots.fetch_add(1, std::memory_order_relaxed);
    } else {
        table[slot] = {data, static_cast<std::uint32_t>(size), 0, 0};
        packed_count.fetch_add(1, std::memory_order_relaxed);
        packed_bytes.fetch_add(static_cast<std::uint64_t>(size), std::memory_order_relaxed);
    }
    packs.fetch_add(1, std::memory_order_relaxed);
    UnlockSlot(range, slot, SlotPacked);
    return true;
#else
    (void)range;
    (void)slot;
    (void)swap;
    return false;
#endif
}

// ---- Research: how much of the held memory is unwritten for a while, and how it compresses ----
// EDEN_CRAM_COLD_REPORT=1: every pass hashes every 64th word of each Own slot; a slot whose hash
// stays the same is counted as unwritten for that many passes (reads are not seen). Up to
// ColdSamples slots unwritten for ColdSeconds are compressed with LZ4 per pass, to estimate
// what compressing them would give back. Measurement only: nothing changes in the slots.
bool ColdReport() noexcept {
    static const bool on = [] {
        const char* value = std::getenv("EDEN_CRAM_COLD_REPORT");
        return value != nullptr && value[0] == '1';
    }();
    return on;
}
constexpr std::size_t ColdSamples = 24;
std::size_t ColdSecondsNow() noexcept {
    static const std::size_t seconds = EnvOr("EDEN_CRAM_PACK_COLD_S", 30);
    return seconds;
}
#define ColdSeconds ColdSecondsNow()
std::uint32_t ColdNow() noexcept {
    return static_cast<std::uint32_t>(std::chrono::duration_cast<std::chrono::seconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count());
}
std::size_t cold_rotor = 0;
std::atomic<std::uint64_t> cold_1{0}, cold_3{0}, cold_6{0}, cold_own{0}, cold_zero{0}, cold_zero_old{0};
// Diagnostic (MKW leak): every 64th word of the slot is zero.
bool ColdLooksZero(std::uintptr_t at) noexcept {
    const auto* word = reinterpret_cast<const volatile std::uint64_t*>(at);
    for (std::size_t i = 0; i < SparseSlot / 8; i += 64)
        if (word[i] != 0) return false;
    return true;
}
std::atomic<std::uint64_t> cold_sampled_bytes{0}, cold_compressed_bytes{0}, cold_compress_ns{0};

std::uint64_t ColdHash(std::uintptr_t at) noexcept {
    const auto* word = reinterpret_cast<const volatile std::uint64_t*>(at);
    std::uint64_t hash = 0;
    for (std::size_t i = 0; i < SparseSlot / 8; i += 64) hash = Mix(hash, word[i]);
    return hash;
}

void PackPass(SparseRange& range, std::size_t slots, std::vector<std::size_t>& candidates) noexcept;

// A slot is probed (made Idle) when nothing has written it for ProbeColdSeconds (its sampled hash
// stood still that long; a Clean slot cannot have been written) and the time it is left alone after
// a touch has run out. Slots being written are never probed: they would only fault, and a slot
// with no access is one the system cannot read or write on the app's behalf (a file write from a
// guest buffer would fail), which memory in use must not risk. pack_mutex held.
std::size_t ProbeColdSeconds() noexcept {
    static const std::size_t seconds = EnvOr("EDEN_CRAM_PROBE_COLD_S", 8);
    return seconds;
}
bool ProbeSlot(SparseRange& range, std::size_t slot, std::uint32_t now) noexcept {
    const std::uint8_t state = SlotState(range, slot);
    if (state != SlotOwn && state != SlotClean) return false;
    if (state == SlotOwn && now - cold_since[slot] < ProbeColdSeconds()) return false;
    // Left alone after a touch for 5 s, twice as long each time, 40 s at most. It was 320 s: eight
    // minutes into a Mario Kart World session most slots had been touched six times and were looked
    // at once in five minutes; requests that needed 400 MiB found 350 MiB of Idle slots, while
    // 1.5 GB of the game's memory had not been written for a minute. A probe that is touched costs
    // one fault (0.1 ms).
    if (now - touch_since[slot].load(std::memory_order_relaxed) <
        (IdleBackoff() << std::min<unsigned>(touch_times[slot].load(std::memory_order_relaxed), 3)))
        return false;
    if (!TryLockSlot(range, slot)) return false;
    const std::uint8_t locked = SlotState(range, slot);
    const bool made = (locked == SlotOwn || locked == SlotClean) && MakeIdle(range, slot, now);
    UnlockSlot(range, slot, made ? SlotIdle : locked);
    return made;
}
// When memory was refused: how long a slot must have been Idle before it may be taken. Shorter than
// for the scanner's own passes (IdleSeconds): the allocation that gets nothing ends the session.
std::size_t UrgentIdleSeconds() noexcept {
    static const std::size_t seconds = EnvOr("EDEN_CRAM_URGENT_IDLE_S", 2);
    return seconds;
}


// Diagnostic: the zero block (every untouched slot shows it, and the sparse tables' unused parts) must
// stay zero. Written through a mapping that should not allow it, every untouched slot of the game
// would read that garbage. Checked each pass through a read-only view of its own.
std::atomic<std::uint64_t> zero_block_dirty{0};
void CheckZeroBlock() noexcept {
#ifdef PS5_NATIVE
    static const volatile std::uint64_t* view = []() -> const volatile std::uint64_t* {
        void* at = reinterpret_cast<void*>(cpu_mapping_hint);
        if (zero_block < 0 || sceKernelMapDirectMemory(&at, SparseSlot, PROT_READ, 0, zero_block, SparseSlot) != 0)
            return nullptr;
        return static_cast<const volatile std::uint64_t*>(at);
    }();
    if (view == nullptr) return;
    for (std::size_t i = 0; i < SparseSlot / 8; i += 8)
        if (view[i] != 0) {
            if (zero_block_dirty.fetch_add(1, std::memory_order_relaxed) < 4) {
                char line[160];
                const int n = std::snprintf(line, sizeof(line), "EDEN_CRAM_ZERO_BLOCK written: word %zu is %#llx\n", i,
                                            (unsigned long long)view[i]);
                if (n > 0) Report(line);
            }
            return;
        }
#endif
}

#ifdef PS5_NATIVE
extern "C" int pthread_getname_np(pthread_t, char*, std::size_t);
#endif
void CountUnpacker(std::size_t slot) noexcept {
    char name[24] = "?";
#ifdef PS5_NATIVE
    (void)pthread_getname_np(pthread_self(), name, sizeof(name));
#endif
    name[sizeof(name) - 1] = 0;
    if ((std::strcmp(name, "GPU") == 0 || std::strcmp(name, "GPUBack") == 0) && gpu_touched != nullptr &&
        slot < gpu_cached_slots && gpu_touched[slot].exchange(1, std::memory_order_acq_rel) == 0)
        pack_gpu_marked.fetch_add(1, std::memory_order_relaxed);
    while (unpackers_lock.test_and_set(std::memory_order_acquire)) {}
    for (auto& entry : unpackers) {
        if (entry.name[0] == 0) std::memcpy(entry.name, name, sizeof(name));
        if (std::strcmp(entry.name, name) == 0) {
            entry.count.fetch_add(1, std::memory_order_relaxed);
            break;
        }
    }
    unpackers_lock.clear(std::memory_order_release);
}
void ReportUnpackers() noexcept {
    char line[512];
    int length = std::snprintf(line, sizeof(line), "EDEN_CRAM_UNPACKERS");
    for (auto& entry : unpackers) {
        if (entry.name[0] == 0 || length <= 0 || length >= static_cast<int>(sizeof(line)) - 40) break;
        length += std::snprintf(line + length, sizeof(line) - length, " %s=%u", entry.name,
                                entry.count.load(std::memory_order_relaxed));
    }
    if (length > 0 && length < static_cast<int>(sizeof(line)) - 1) {
        line[length] = '\n';
        line[length + 1] = 0;
        Report(line);
    }
}

void ColdPass(SparseRange& range, std::size_t slots) noexcept {
    CheckZeroBlock();
    const std::lock_guard pack_lock{pack_mutex};
    holding_pack = true;
    struct Held { ~Held() { holding_pack = false; } } held;
    // Room made once: a pass allocates nothing (the heap may be out of memory when it runs).
    static std::vector<std::size_t> candidates;
    if (cold_hash.size() != slots) {
        cold_hash.assign(slots, 0);
        cold_since.assign(slots, ColdNow());
        candidates.clear();
        candidates.reserve(slots);
        pack_ready.reserve(slots);
    }
    candidates.clear();
    std::uint64_t own = 0, c1 = 0, c3 = 0, c6 = 0, zero = 0, zero_old = 0;
    const std::uint32_t now = ColdNow();
    const bool idle_mode = IdleEnabled() && PackEnabled() && idle_since != nullptr;
    const bool pool_short = idle_mode && !pack_refused.load(std::memory_order_relaxed) && PoolShortForPacking();
    std::size_t made_idle = 0;
    for (std::size_t slot = 0; slot < slots; ++slot) {
        const std::uint8_t state = SlotState(range, slot);
        if (state == SlotIdle) {
            // Untouched since it was made Idle (no hash: reading it would wake it).
            ++own;
            if (idle_mode && now - idle_since[slot] >= IdleSeconds()) candidates.push_back(slot);
            continue;
        }
        if (state != SlotOwn && state != SlotClean) {
            cold_since[slot] = now;
            cold_hash[slot] = 0;
            continue;
        }
        ++own;
        std::uint32_t age = 0;
        if (state == SlotOwn) {
            const std::uint64_t hash = ColdHash(SlotAddress(range, slot));
            // (A write that dropped the slot's copy in the swap file was counted there.)
            const bool noted = rewrite_noted != nullptr && rewrite_noted[slot].exchange(0, std::memory_order_relaxed) != 0;
            if (hash != cold_hash[slot]) {
                // Written again after a while unwritten: memory the game reuses (swap_rewritten).
                if (!noted && cold_hash[slot] != 0 && now - cold_since[slot] >= RecycleQuietSeconds) NoteRewritten(slot);
                cold_since[slot] = now;
                cold_hash[slot] = hash;
                if (pack_raw != nullptr) pack_raw[slot] = 0; // written: it may compress now
            }
            age = now - cold_since[slot];
            c1 += age >= 20;
            c3 += age >= 60;
            c6 += age >= 120;
        }
        // While the pool is short: no access for a while, to learn whether anything touches it.
        if (pool_short && made_idle < IdlePerPass && ProbeSlot(range, slot, now)) {
            ++made_idle;
            continue;
        }
        if (state != SlotOwn) continue;
        if (!idle_mode && age >= ColdSeconds) candidates.push_back(slot);
        if (ColdReport() && ColdLooksZero(SlotAddress(range, slot))) {
            ++zero;
            zero_old += age >= 60;
        }
    }
    cold_zero.store(zero, std::memory_order_relaxed);
    cold_zero_old.store(zero_old, std::memory_order_relaxed);
    cold_own.store(own, std::memory_order_relaxed);
    cold_1.store(c1, std::memory_order_relaxed);
    cold_3.store(c3, std::memory_order_relaxed);
    cold_6.store(c6, std::memory_order_relaxed);
    if (PackEnabled()) PackPass(range, slots, candidates);
    if (!ColdReport()) return;
    const auto start = std::chrono::steady_clock::now();
    const bool can_compress = &::Common::Compression::CompressDataLZ4 != nullptr;
    for (std::size_t i = 0; can_compress && i < std::min(ColdSamples, candidates.size()); ++i) {
        const std::size_t slot = candidates[(cold_rotor + i * 7919) % candidates.size()];
        if (SlotState(range, slot) != SlotOwn) continue; // packed meanwhile: a read would unpack it
        std::vector<std::uint8_t> copy(SparseSlot);
        std::memcpy(copy.data(), reinterpret_cast<const void*>(SlotAddress(range, slot)), SparseSlot);
        const auto packed = ::Common::Compression::CompressDataLZ4(copy.data(), copy.size());
        cold_sampled_bytes.fetch_add(SparseSlot, std::memory_order_relaxed);
        cold_compressed_bytes.fetch_add(packed.empty() ? SparseSlot : packed.size(), std::memory_order_relaxed);
    }
    cold_rotor += ColdSamples;
    cold_compress_ns.fetch_add(static_cast<std::uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::steady_clock::now() - start).count()), std::memory_order_relaxed);
    const std::uint64_t sampled = cold_sampled_bytes.load(std::memory_order_relaxed);
    char line[400];
    const int length = std::snprintf(line, sizeof(line),
        "EDEN_CRAM_COLD own=%lluMiB unwritten_20s=%lluMiB unwritten_60s=%lluMiB unwritten_120s=%lluMiB "
        "lz4_sampled=%lluMiB lz4_ratio=%.3f lz4_ms=%llu zero_sampled=%lluMiB zero_unwritten_60s=%lluMiB\n",
        (unsigned long long)(own * 2), (unsigned long long)(c1 * 2), (unsigned long long)(c3 * 2),
        (unsigned long long)(c6 * 2), (unsigned long long)(sampled >> 20),
        sampled ? double(cold_compressed_bytes.load(std::memory_order_relaxed)) / double(sampled) : 0.0,
        (unsigned long long)(cold_compress_ns.load(std::memory_order_relaxed) / 1000000),
        (unsigned long long)(cold_zero.load(std::memory_order_relaxed) * 2),
        (unsigned long long)(cold_zero_old.load(std::memory_order_relaxed) * 2));
    if (length > 0) Report(line);
}

#ifdef PS5_NATIVE
// The packed store's blocks are filled one after another, and a block goes back only when all of
// its data did. Slots are unpacked in no order, so blocks stay held for the little that is left in
// them: ten minutes into a Mario Kart World session (3,900 slots packed, 2,700 unpacked again) the
// store held 544 MiB of blocks for 282 MiB of data. When 64 MiB or more of the store is such empty
// space, the data in blocks less than half full moves to the block being filled (a copy under the
// slot's lock; the slot stays Packed), and the emptied blocks go back. pack_mutex held.
std::atomic<std::uint64_t> store_compacted{0};
void StoreCompact(SparseRange& range, std::size_t slots) noexcept {
    PackedSlot* const table = packed_slots.load(std::memory_order_acquire);
    const std::size_t held = store_held.load(std::memory_order_relaxed);
    const std::size_t live = static_cast<std::size_t>(packed_bytes.load(std::memory_order_relaxed));
    static const bool on = EnvOr("EDEN_CRAM_STORE_COMPACT", 1) != 0;
    if (!on || table == nullptr || store_base == 0 || held < live + (std::size_t{64} << 20)) return;
    std::size_t moved = 0;
    for (std::size_t slot = 0; slot < slots && moved < (std::size_t{128} << 20); ++slot) {
        if (SlotState(range, slot) != SlotPacked) continue;
        void* const data = table[slot].data; // (looked at again under the slot's lock)
        if (data == nullptr) continue;        // in the swap file
        const std::size_t block = (reinterpret_cast<std::uintptr_t>(data) - store_base) / StoreBlock;
        if (block == store_current || store_live[block].load(std::memory_order_relaxed) * 2 > StoreBlock) continue;
        if (!TryLockSlot(range, slot)) continue; // (a fault handler has it: TryLockSlot)
        const std::uint8_t state = SlotState(range, slot);
        bool room = true;
        if (state == SlotPacked && table[slot].data == data) {
            const std::uint32_t size = table[slot].size;
            void* const fresh = StoreAlloc(size, nullptr);
            room = fresh != nullptr;
            if (room) {
                std::memcpy(fresh, data, size);
                table[slot].data = fresh;
                StoreRelease(data, size);
                moved += size;
            }
        }
        UnlockSlot(range, slot, state);
        if (!room) break; // the pool has no block for the store now
    }
    if (moved != 0) {
        store_compacted.fetch_add(moved, std::memory_order_relaxed);
        StoreReclaim();
    }
}
#endif

std::uint64_t StoreMoved() noexcept {
#ifdef PS5_NATIVE
    return store_compacted.load(std::memory_order_relaxed);
#else
    return 0;
#endif
}

void PackPass(SparseRange& range, std::size_t slots, std::vector<std::size_t>& candidates) noexcept {
#ifdef PS5_NATIVE
    StoreReclaim();
    StoreCompact(range, slots);
#endif
    if (packed_slots.load(std::memory_order_acquire) == nullptr) {
        auto* table = new (std::nothrow) PackedSlot[slots]();
        if (table == nullptr) return;
        if (gpu_touched == nullptr) {
            gpu_touched = new (std::nothrow) std::atomic<std::uint8_t>[slots]();
            if (gpu_cached_slots == 0) gpu_cached_slots = slots;
        }
        if (PackKeep()) {
            pack_kept = new (std::nothrow) std::int64_t[slots];
            if (pack_kept != nullptr)
                for (std::size_t i = 0; i < slots; ++i) pack_kept[i] = -1;
        }
        if (idle_since == nullptr && IdleEnabled()) {
            auto* since = new (std::nothrow) std::uint32_t[slots]();
            auto* touched = new (std::nothrow) std::atomic<std::uint32_t>[slots]();
            auto* times = new (std::nothrow) std::atomic<std::uint8_t>[slots]();
            if (since != nullptr && touched != nullptr && times != nullptr) {
                touch_times = times;
                touch_since = touched;
                idle_since = since;
            }
        }
        if (pack_raw == nullptr) pack_raw = new (std::nothrow) std::uint8_t[slots]();
        if (swap_rewritten == nullptr) swap_rewritten = new (std::nothrow) std::atomic<std::uint8_t>[slots]();
        if (rewrite_noted == nullptr) rewrite_noted = new (std::nothrow) std::atomic<std::uint8_t>[slots]();
        if (swap_age_class == nullptr) swap_age_class = new (std::nothrow) std::uint8_t[slots]();
        packed_slots.store(table, std::memory_order_release);
        pack_times.assign(slots, 0);
        recover_order.reserve(slots);
        pack_scratch.resize(static_cast<std::size_t>(LZ4_compressBound(static_cast<int>(SparseSlot))));
    }
    if (pack_refused.load(std::memory_order_relaxed) || !PoolShortForPacking()) return;
    // Device memory nothing is bound to goes back before any guest memory is packed.
    if (&eden_radv_private_radv_ps5_eden_trim != nullptr) (void)eden_radv_private_radv_ps5_eden_trim(std::uint64_t{512} << 20);
    // The pool is short: the swap file is made ready (nothing is written to it but its test).
    if (IdleEnabled() && idle_since != nullptr) (void)SwapStart();
    // The longest idle first; without Idle slots the coldest, each needing ColdSeconds << (times packed) unwritten.
    const bool idle_mode = IdleEnabled() && idle_since != nullptr;
    const std::uint32_t now = ColdNow();
    std::vector<std::size_t>& ready = pack_ready;
    ready.clear();
    for (const std::size_t slot : candidates)
        if (idle_mode || now - cold_since[slot] >= (ColdSeconds << std::min<unsigned>(pack_times[slot], 4))) ready.push_back(slot);
    if (idle_mode) std::sort(ready.begin(), ready.end(), [](std::size_t a, std::size_t b) { return idle_since[a] < idle_since[b]; });
    else std::sort(ready.begin(), ready.end(), [](std::size_t a, std::size_t b) { return cold_since[a] < cold_since[b]; });
    // Into the packed store only (slots that compress): the swap file takes slots only when memory
    // is asked back (RecoverMemory).
    std::size_t done = 0;
    for (const std::size_t slot : ready) {
        if (done >= PackPerPass) break;
        if (pack_times[slot] < 255) ++pack_times[slot];
        if (PackSlot(range, slot, Swap::No)) {
            ++done;
            cold_since[slot] = now;
        }
    }
    static const bool pass_log = EnvOr("EDEN_SWAP_LOG", 0) != 0;
    if (pass_log) {
        char line[240];
        const int length = std::snprintf(line, sizeof(line),
            "EDEN_SWAP_PASS free=%zuMiB packed=%zu file=%lluMiB written=%lluMiB tokens=%lluMiB\n",
            PoolFreePieces() >> 20, done,
            (unsigned long long)(swap_used_units.load(std::memory_order_relaxed) * SwapUnit >> 20),
            (unsigned long long)(swap_written.load(std::memory_order_relaxed) >> 20), (unsigned long long)(swap_tokens >> 20));
        if (length > 0) Report(line);
    }
}

void ScanPass(SparseRange& range) noexcept {
    const auto start = std::chrono::steady_clock::now();
    const std::size_t slots = (range.end - range.begin.load(std::memory_order_relaxed)) / SparseSlot;
    std::vector<std::pair<std::uint64_t, std::size_t>> samples;
    for (std::size_t slot = 0; slot < slots; ++slot) {
        if (SlotState(range, slot) != SlotOwn) continue;
        const std::uintptr_t at = SlotAddress(range, slot);
        if (AllZero(at)) { // a read without the lock: only a hint, checked again fenced
            if (ScanZero(range, slot)) counters.returns_scan.fetch_add(1, std::memory_order_relaxed);
        } else if (fine_requested.load(std::memory_order_relaxed) && ScanFine(range, slot)) {
            // now Fine
        } else if (dedup_requested.load(std::memory_order_relaxed)) {
            samples.emplace_back(SampleHash(at), slot);
        }
    }
    std::sort(samples.begin(), samples.end());
    for (std::size_t i = 0; i < samples.size();) {
        std::size_t j = i + 1;
        while (j < samples.size() && samples[j].first == samples[i].first) ++j;
        if (j - i > 1) {
            std::vector<std::pair<std::uint64_t, std::size_t>> full;
            for (std::size_t k = i; k < j; ++k)
                full.emplace_back(FullHash(SlotAddress(range, samples[k].second), SparseSlot), samples[k].second);
            std::sort(full.begin(), full.end());
            for (std::size_t a = 0; a < full.size();) {
                std::size_t b = a + 1;
                while (b < full.size() && full[b].first == full[a].first) {
                    (void)ScanShare(range, full[a].second, full[b].second);
                    ++b;
                }
                a = b;
            }
        }
        i = j;
    }
    if (ColdReport() || PackEnabled()) ColdPass(range, slots);
    counters.passes.fetch_add(1, std::memory_order_relaxed);
    counters.scan_ns.fetch_add(static_cast<std::uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::steady_clock::now() - start).count()), std::memory_order_relaxed);
}

void ServiceReport(SparseRange& range, const char* when) noexcept {
    const std::size_t slots = (range.end - range.begin.load(std::memory_order_relaxed)) / SparseSlot;
    std::size_t own = 0, shared = 0, fine = 0, fine_kept = 0, idle = 0, clean = 0, packed = 0, swapped_out = 0;
    std::size_t kinds[4] = {}; // slots that hold data, by how often they were written again (swap_rewritten)
    PackedSlot* const packed_table = packed_slots.load(std::memory_order_acquire);
    for (std::size_t slot = 0; slot < slots; ++slot) {
        const auto state = SlotState(range, slot);
        if (&range == guest_range.load(std::memory_order_acquire) &&
            (state == SlotOwn || state == SlotIdle || state == SlotClean || state == SlotPacked))
            ++kinds[std::min<unsigned>(Rewritten(slot), 3)];
        own += state == SlotOwn || state == SlotIdle || state == SlotClean; // a block of its own, as Own
        idle += state == SlotIdle;
        clean += state == SlotClean;
        packed += state == SlotPacked;
        swapped_out += state == SlotPacked && packed_table != nullptr && packed_table[slot].swap != 0;
        shared += state == SlotShared;
        if (state == SlotFine) {
            ++fine;
            fine_kept += FineKept(range, slot);
        }
    }
    char line[640];
    const int length = std::snprintf(line, sizeof(line),
        "EDEN_CRAM %s held=%zuMiB own=%zu shared=%zu zero=%zu faults=%llu promotions=%llu copies=%llu waits=%llu "
        "returns_clear=%llu returns_free=%llu returns_scan=%llu dedup=%llu reserve_used=%llu failures=%llu "
        "passes=%llu scan_ms=%llu fine=%zu fine_held=%zuMiB fined=%llu unfined=%llu fine_failures=%llu "
        "reserve=%zuMiB host_transferred=%lluMiB freed_parts=%llu freed_parts_cleared=%lluMiB\n",
        when, guest_committed.load(std::memory_order_relaxed) >> 20, own, shared, slots - own - shared - fine - packed,
        (unsigned long long)counters.faults.load(), (unsigned long long)counters.promotions.load(),
        (unsigned long long)counters.copies.load(), (unsigned long long)counters.waits.load(),
        (unsigned long long)counters.returns_clear.load(), (unsigned long long)counters.returns_free.load(),
        (unsigned long long)counters.returns_scan.load(), (unsigned long long)counters.dedup.load(),
        (unsigned long long)counters.reserve_used.load(), (unsigned long long)counters.failures.load(),
        (unsigned long long)counters.passes.load(), (unsigned long long)(counters.scan_ns.load() / 1000000),
        fine, fine_kept * FinePage >> 20, (unsigned long long)counters.fined.load(),
        (unsigned long long)counters.unfined.load(), (unsigned long long)counters.fine_failures.load(),
        reserve_count.load(std::memory_order_relaxed) * (SparseSlot >> 20),
        (unsigned long long)counters.host_reserve_used.load() * (SparseSlot >> 20),
        (unsigned long long)freed_part_calls.load(), (unsigned long long)(freed_part_bytes.load() >> 20));
    if (length > 0) Report(line);
    // Who wrote the slots that were promoted: the largest places (offsets in the app's image).
    struct Top { const WriterSite* site; std::uint64_t count; } top[10]{};
    for (const WriterSite& site : writer_sites) {
        const std::uint64_t count = site.count.load(std::memory_order_relaxed);
        if (site.key.load(std::memory_order_relaxed) == 0 || count <= top[9].count) continue;
        std::size_t place = 9;
        while (place > 0 && top[place - 1].count < count) {
            top[place] = top[place - 1];
            --place;
        }
        top[place] = {&site, count};
    }
    char writers[640];
    int used = std::snprintf(writers, sizeof(writers),
                             "EDEN_CRAM_WRITERS promotions by code of the app %llu, by other code %llu\n",
                             (unsigned long long)promotions_in_app_code.load(), (unsigned long long)promotions_elsewhere.load());
    if (used > 0) Report(writers);
    for (const Top& entry : top) {
        if (entry.count == 0) break;
        int length = std::snprintf(writers, sizeof(writers), "EDEN_CRAM_WRITER %llu:", (unsigned long long)entry.count);
        for (const std::uintptr_t pc : entry.site->pcs) {
            if (length > 0 && static_cast<std::size_t>(length) < sizeof(writers) - 24)
                length += std::snprintf(writers + length, sizeof(writers) - length, " %llx",
                                        (unsigned long long)(pc != 0 ? pc - 0x400000ull : 0));
        }
        if (length > 0 && static_cast<std::size_t>(length) < sizeof(writers) - 2) {
            writers[length++] = '\n';
            writers[length] = 0;
            Report(writers);
        }
    }
    if (PackEnabled()) {
        char pack_line[400];
        const int pack_length = std::snprintf(pack_line, sizeof(pack_line),
            "EDEN_CRAM_PACK %s packed=%lluMiB stored=%lluMiB store_held=%lluMiB store_moved=%lluMiB packs=%llu unpacks=%llu skips=%llu unpack_keep=%lluMiB"
            " kept_compared=%llu kept_differed=%llu zero_block_dirty=%llu gpu_skips=%llu gpu_marked=%llu\n", when,
            (unsigned long long)(packed_count.load(std::memory_order_relaxed) * (SparseSlot >> 20)),
            (unsigned long long)(packed_bytes.load(std::memory_order_relaxed) >> 20),
            (unsigned long long)(store_held.load(std::memory_order_relaxed) >> 20),
            (unsigned long long)(StoreMoved() >> 20),
            (unsigned long long)packs.load(std::memory_order_relaxed), (unsigned long long)unpacks.load(std::memory_order_relaxed),
            (unsigned long long)pack_skips.load(std::memory_order_relaxed), (unsigned long long)(unpack_keep_count.load(std::memory_order_relaxed) * 2),
            (unsigned long long)pack_keep_compared.load(std::memory_order_relaxed),
            (unsigned long long)pack_keep_differed.load(std::memory_order_relaxed),
            (unsigned long long)zero_block_dirty.load(std::memory_order_relaxed),
            (unsigned long long)pack_gpu_skips.load(std::memory_order_relaxed),
            (unsigned long long)pack_gpu_marked.load(std::memory_order_relaxed));
        if (pack_length > 0) Report(pack_line);
        ReportUnpackers();
        // The swap file and the Idle slots. written: this session, to the storage (what wears it).
        const std::uint64_t reads = swap_reads.load(std::memory_order_relaxed), writes = swap_writes.load(std::memory_order_relaxed);
        const std::uint64_t written = swap_written.load(std::memory_order_relaxed);
        char swap_line[640];
        const int swap_length = std::snprintf(swap_line, sizeof(swap_line),
            "EDEN_SWAP %s on=%d (%s) out=%zuMiB clean=%zuMiB file_used=%lluMiB written=%lluMiB written_today=%lluMiB "
            "written_ever=%lluMiB writes=%llu raw=%llu write_ms=%llu reused=%llu dirtied=%llu reads=%llu read_avg_us=%llu "
            "read_max_us=%llu denied_rate=%llu denied_cap=%llu full=%llu idle_now=%zuMiB idled=%llu woken=%llu kept_free=%lluMiB/%llu "
            "last_resort=%llu late_reads=%llu/%llu\n", when,
            swap_state.load(std::memory_order_relaxed) == 1 ? 1 : 0,
            swap_state.load(std::memory_order_relaxed) == 1 ? "ok" : swap_off_why.load(std::memory_order_acquire),
            swapped_out * (SparseSlot >> 20), clean * (SparseSlot >> 20),
            (unsigned long long)(swap_used_units.load(std::memory_order_relaxed) * SwapUnit >> 20),
            (unsigned long long)(written >> 20), (unsigned long long)(SwapDayBytes(written) >> 20),
            (unsigned long long)((swap_total_before + written) >> 20), (unsigned long long)writes,
            (unsigned long long)swap_raw.load(std::memory_order_relaxed),
            (unsigned long long)(swap_write_ns.load(std::memory_order_relaxed) / 1000000),
            (unsigned long long)swap_reused.load(std::memory_order_relaxed),
            (unsigned long long)swap_dirtied.load(std::memory_order_relaxed), (unsigned long long)reads,
            (unsigned long long)(reads ? swap_read_ns.load(std::memory_order_relaxed) / reads / 1000 : 0),
            (unsigned long long)(swap_read_max_ns.load(std::memory_order_relaxed) / 1000),
            (unsigned long long)swap_denied_rate.load(std::memory_order_relaxed),
            (unsigned long long)swap_denied_cap.load(std::memory_order_relaxed),
            (unsigned long long)swap_full.load(std::memory_order_relaxed), idle * (SparseSlot >> 20),
            (unsigned long long)idled.load(std::memory_order_relaxed), (unsigned long long)idle_woken.load(std::memory_order_relaxed),
            (unsigned long long)(keeper_bytes.load(std::memory_order_relaxed) >> 20),
            (unsigned long long)keeper_calls.load(std::memory_order_relaxed),
            (unsigned long long)swap_last_resort.load(std::memory_order_relaxed),
            (unsigned long long)clean_late_reads.load(std::memory_order_relaxed),
            (unsigned long long)clean_refaults.load(std::memory_order_relaxed));
        if (swap_length > 0) Report(swap_line);
        // Slots written by how long they had been unwritten (under 30 s, 60 s, 2, 5, 10 min, more)
        // and by how often they had been written again before (never, once, twice, more), with how
        // many of each a later write dropped; and the slots that hold data now, by the same count.
        const int ages_length = std::snprintf(swap_line, sizeof(swap_line),
            "EDEN_SWAP_AGES %s written=%u/%u/%u/%u/%u/%u dropped=%u/%u/%u/%u/%u/%u by_rewrites=%u/%u/%u/%u "
            "dropped=%u/%u/%u/%u slots=%zu/%zu/%zu/%zu\n", when,
            swap_age_written[0].load(), swap_age_written[1].load(), swap_age_written[2].load(), swap_age_written[3].load(),
            swap_age_written[4].load(), swap_age_written[5].load(), swap_age_dropped[0].load(), swap_age_dropped[1].load(),
            swap_age_dropped[2].load(), swap_age_dropped[3].load(), swap_age_dropped[4].load(), swap_age_dropped[5].load(),
            swap_kind_written[0].load(), swap_kind_written[1].load(), swap_kind_written[2].load(), swap_kind_written[3].load(),
            swap_kind_dropped[0].load(), swap_kind_dropped[1].load(), swap_kind_dropped[2].load(), swap_kind_dropped[3].load(),
            kinds[0], kinds[1], kinds[2], kinds[3]);
        if (ages_length > 0 && (writes != 0 || kinds[1] + kinds[2] + kinds[3] != 0)) Report(swap_line);
    }
}

// What finer slots would give: 16 KiB pages of the Own slots that hold only zeroes, and those that
// repeat another page (by a full hash). Read once, when the session ends.
void MeasureFinePages(SparseRange& range) noexcept {
    constexpr std::size_t Fine = 16384;
    const std::size_t slots = (range.end - range.begin.load(std::memory_order_relaxed)) / SparseSlot;
    std::vector<std::uint64_t> hashes;
    std::size_t pages = 0, zero = 0;
    for (std::size_t slot = 0; slot < slots; ++slot) {
        if (SlotState(range, slot) != SlotOwn) continue;
        for (std::size_t offset = 0; offset < SparseSlot; offset += Fine) {
            const std::uintptr_t at = SlotAddress(range, slot) + offset;
            ++pages;
            const auto* word = reinterpret_cast<const volatile std::uint64_t*>(at);
            bool is_zero = true;
            for (std::size_t i = 0; i < Fine / 8 && is_zero; ++i) is_zero = word[i] == 0;
            if (is_zero) ++zero;
            else hashes.push_back(FullHash(at, Fine));
        }
    }
    std::sort(hashes.begin(), hashes.end());
    std::size_t repeats = 0;
    for (std::size_t i = 1; i < hashes.size(); ++i) repeats += hashes[i] == hashes[i - 1];
    char line[192];
    const int length = std::snprintf(line, sizeof(line),
        "EDEN_CRAM_FINE pages16k=%zu zero=%zu (%zuMiB) repeats=%zu (%zuMiB)\n", pages, zero, zero * Fine >> 20, repeats,
        repeats * Fine >> 20);
    if (length > 0) Report(line);
}

void ScannerLoop(SparseRange* range) {
    auto next_report = std::chrono::steady_clock::now() + std::chrono::seconds(30);
    std::unique_lock lock{scanner_mutex};
    while (!scanner_stop) {
        // Every 20 s; every 5 s while packing has a short pool to answer (the game grows ~10 MiB/s).
        const bool hurry = PackEnabled() && !pack_refused.load(std::memory_order_relaxed) && PoolShortForPacking();
        // While the pool is not short it is looked at every 2 s (one system call), and the pass comes
        // at once when it runs short: a race load takes 1.5 GB within seconds, and the slots nothing
        // touches have to be found (Idle slots) before the memory is asked back.
        const int seconds = !hurry ? 20 : 5;
        for (int waited = 0; waited < seconds && !scanner_stop; waited += 2) {
            scanner_wake.wait_for(lock, std::chrono::seconds(std::min(2, seconds - waited)), [] { return scanner_stop; });
            if (!hurry && !scanner_stop && PackEnabled() && !pack_refused.load(std::memory_order_relaxed) && PoolShortForPacking())
                break;
        }
        if (scanner_stop) break;
        lock.unlock();
        if (scan_requested.load(std::memory_order_relaxed)) ScanPass(*range);
        if (std::chrono::steady_clock::now() >= next_report) {
            ServiceReport(*range, "now");
            next_report = std::chrono::steady_clock::now() + std::chrono::seconds(30);
        }
        lock.lock();
    }
}

// Once: does a write to a slot that shows the zero block come back here, with its address, and
// resume? A refusal keeps the guest's memory committed when it is handed out, as before.
bool ServiceAvailable() noexcept {
    int state = service_state.load(std::memory_order_acquire);
    if (state != 0) return state == 1;
    const char* step = "ok";
    long long fault_ns = -1;
    void* base = nullptr;
    if (!SparseAvailable()) step = "sparse";
    else if (!InstallServiceSignal()) step = "install";
    else if ((base = NewSparseRange(2 * SparseSlot, false)) == nullptr) step = "reserve";
    if (base != nullptr) {
        SparseRange* range = nullptr;
        {
            const std::lock_guard lock{sparse_mutex};
            range = SparseRangeOf(reinterpret_cast<std::uintptr_t>(base));
        }
        if (range == nullptr || !AddStates(*range)) {
            step = "states";
        } else {
            probe_range.store(range, std::memory_order_release);
            auto* word = reinterpret_cast<volatile std::uint64_t*>(static_cast<std::uint8_t*>(base) + SparseSlot + 64);
            const auto before = counters.faults.load(std::memory_order_relaxed);
            probe_expected.store(reinterpret_cast<std::uintptr_t>(word), std::memory_order_release);
            const auto start = std::chrono::steady_clock::now();
            *word = 0x5a17ed5a17ed5a17ull; // faults once: the slot shows the zero block
            fault_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now() - start).count();
            probe_expected.store(0, std::memory_order_release);
            if (*word != 0x5a17ed5a17ed5a17ull) step = "value";
            else if (counters.faults.load(std::memory_order_relaxed) != before + 1) step = "fault-count";
            else if (probe_mismatch.load(std::memory_order_relaxed)) step = "fault-address";
            else if (SlotState(*range, 1) != SlotOwn || SlotState(*range, 0) != SlotZero) step = "state";
            else {
                // Back to the zero block, then written again: a slot comes and goes.
                LockSlot(*range, 1);
                ReturnSlot(*range, 1);
                UnlockSlot(*range, 1, SlotZero);
                if (*word != 0) step = "return";
                else {
                    *word = 7;
                    if (*word != 7 || SlotState(*range, 1) != SlotOwn) step = "again";
                }
            }
            if (std::strcmp(step, "ok") == 0) {
                // Fenced and unfenced: read while fenced, written after.
                const std::uintptr_t at = SlotAddress(*range, 1);
                Fence(at);
                if (*word != 7) step = "fence-read";
                Unfence(at);
                *word = 9;
                if (std::strcmp(step, "ok") == 0 && *word != 9) step = "unfence";
            }
            if (std::strcmp(step, "ok") == 0) {
                // Idle: no access until a read wakes it, with its data as it was, writable again.
                LockSlot(*range, 1);
                const bool idle = MakeIdle(*range, 1, 0);
                UnlockSlot(*range, 1, idle ? SlotIdle : SlotOwn);
                const auto before_wake = counters.faults.load(std::memory_order_relaxed);
                if (!idle) step = "idle";
                else if (*word != 9) step = "idle-read";
                else if (SlotState(*range, 1) != SlotOwn || counters.faults.load(std::memory_order_relaxed) != before_wake + 1)
                    step = "idle-wake";
                else {
                    *word = 9; // written again without a fault
                    if (counters.faults.load(std::memory_order_relaxed) != before_wake + 1) step = "idle-write";
                }
            }
            if (std::strcmp(step, "ok") == 0) {
                // Two slots of the same bytes: one block, then copied apart by a write, then the last
                // one takes the block back.
                auto* other = reinterpret_cast<volatile std::uint64_t*>(static_cast<std::uint8_t*>(base) + 64);
                *other = 9;
                const bool shared = ScanShare(*range, 1, 0);
#ifdef PS5_NATIVE
                if (!shared || SlotState(*range, 0) != SlotShared || SlotState(*range, 1) != SlotShared) step = "share";
                else if (*other != 9 || *word != 9) step = "share-read";
                else {
                    *other = 10; // a copy of its own
                    if (*other != 10 || *word != 9 || SlotState(*range, 0) != SlotOwn) step = "share-copy";
                    else {
                        *word = 11; // the last one: the block is its own again
                        if (*word != 11 || *other != 10 || SlotState(*range, 1) != SlotOwn) step = "share-last";
                    }
                }
#else
                if (shared) step = "share-host";
#endif
            }
            if (std::strcmp(step, "ok") == 0) {
                // Slot 1 with one page of data at page 3: Fine (one 16 KiB page kept), read in place,
                // then a write makes it whole again with every byte where it was.
                LockSlot(*range, 1);
                if (SlotState(*range, 1) != SlotZero) ReturnSlot(*range, 1);
                UnlockSlot(*range, 1, SlotZero);
                auto* page3 = reinterpret_cast<volatile std::uint64_t*>(static_cast<std::uint8_t*>(base) + SparseSlot + 3 * FinePage + 8);
                *page3 = 0x0f1e2d3c4b5a6978ull;
                const auto held = guest_committed.load(std::memory_order_relaxed);
                if (!ScanFine(*range, 1) || SlotState(*range, 1) != SlotFine) step = "fine";
                else if (*page3 != 0x0f1e2d3c4b5a6978ull || *word != 0) step = "fine-read";
                else if (held - guest_committed.load(std::memory_order_relaxed) != SparseSlot - FinePage) step = "fine-held";
                else {
                    *word = 13; // page 0: the slot is whole again
                    if (*word != 13 || *page3 != 0x0f1e2d3c4b5a6978ull || SlotState(*range, 1) != SlotOwn ||
                        (range->fine[1] & 0xf00) == 0)
                        step = "fine-write";
                }
            }
            for (std::size_t slot = 0; slot < 2; ++slot)
                if (SlotState(*range, slot) != SlotZero) {
                    LockSlot(*range, slot);
                    ReturnSlot(*range, slot);
                    UnlockSlot(*range, slot, SlotZero);
                }
            probe_range.store(nullptr, std::memory_order_release);
        }
        FreeSparsePages(base, 2 * SparseSlot);
    }
    ResetCounters();
    const bool ok = std::strcmp(step, "ok") == 0;
    service_refused = step;
    char line[128];
    const int length = std::snprintf(line, sizeof(line), "EDEN_CRAM_CHECK ok=%d step=%s fault_ns=%lld\n", ok ? 1 : 0,
                                     step, fault_ns);
    if (length > 0) Report(line);
    service_state.store(ok ? 1 : 2, std::memory_order_release);
    return ok;
}

// The guest range starts being a service (when the platform passed the check).
void ServiceStart(SparseRange& range) noexcept {
    // A refusal belongs to the session it ended: a later crash must not report it again.
    refused_address.store(0, std::memory_order_relaxed);
    if (!service_requested.load(std::memory_order_relaxed) || !ServiceAvailable() || !AddStates(range)) {
        char line[96];
        const int length = std::snprintf(line, sizeof(line), "EDEN_CRAM off: %s\n",
                                         !service_requested.load() ? "switched-off" : service_refused);
        if (length > 0) Report(line);
        return;
    }
    ResetCounters();
    range.promote = true;
#ifdef PS5_NATIVE
    FillReserve(ReserveAtStart);
#endif
    service_range.store(&range, std::memory_order_release);
    Eden::Fastmem::guest_write_fault.store(&HandleGuestWriteFault, std::memory_order_release);
    {
        const std::lock_guard lock{scanner_mutex};
        scanner_stop = false;
    }
    scanner = std::thread(ScannerLoop, &range);
#ifdef PS5_NATIVE
    keeper = std::thread(ReserveKeeper);
#endif
    char line[128];
    const int length = std::snprintf(line, sizeof(line), "EDEN_CRAM on: scan=%d dedup=%d fine=%d reserve=%zuMiB\n",
                                     scan_requested.load() ? 1 : 0, dedup_requested.load() ? 1 : 0,
                                     fine_requested.load() ? 1 : 0,
                                     reserve_count.load() * (SparseSlot >> 20));
    if (length > 0) Report(line);
}

void ServiceStop(SparseRange& range) noexcept {
    if (service_range.load(std::memory_order_acquire) != &range) return;
    {
        const std::lock_guard lock{scanner_mutex};
        scanner_stop = true;
    }
    scanner_wake.notify_all();
    if (scanner.joinable()) scanner.join();
#ifdef PS5_NATIVE
    if (keeper.joinable()) keeper.join();
#endif
    ServiceReport(range, "end");
    MeasureFinePages(range);
    {
        const std::lock_guard pack_lock{pack_mutex};
        SwapLedgerFlush();
    }
    service_range.store(nullptr, std::memory_order_release);
    // Packed data goes with the range (ReleaseServiceBlocks drops what is still packed after this).
#ifdef PS5_NATIVE
    EmptyReserve();
#endif
}

// What a guest range keeps when it is given back: each block once, shared ones too.
void ReleaseServiceBlocks(SparseRange& range) noexcept {
#ifdef PS5_NATIVE
    const std::size_t slots = (range.end - range.begin.load(std::memory_order_relaxed)) / SparseSlot;
    for (std::size_t slot = 0; slot < slots; ++slot) {
        DropSlotAliases(range, slot);
        const auto state = SlotState(range, slot);
        if (state == SlotFine) {
            const std::size_t kept = FineKept(range, slot);
            if (kept != 0 && sceKernelReleaseDirectMemory(range.physical[slot], kept * FinePage) != 0) std::abort();
            guest_committed.fetch_sub(kept * FinePage, std::memory_order_relaxed);
        } else if (state == SlotOwn || state == SlotIdle || state == SlotClean) {
            if (state != SlotOwn && range.guest) ForgetPacked(slot); // (guest_range is cleared by now)
            if (sceKernelReleaseDirectMemory(range.physical[slot], SparseSlot) != 0) std::abort();
        } else if (state == SlotPacked) {
            ForgetPacked(slot);
        } else if (state == SlotShared && UnshareBlock(range.physical[slot]) == 0) {
            if (sceKernelReleaseDirectMemory(range.physical[slot], SparseSlot) != 0) std::abort();
            guest_committed.fetch_sub(SparseSlot, std::memory_order_relaxed);
        }
        if (state == SlotOwn || state == SlotIdle || state == SlotClean) guest_committed.fetch_sub(SparseSlot, std::memory_order_relaxed);
        range.owned[slot].store(false, std::memory_order_relaxed);
    }
    if (&range == guest_range.load(std::memory_order_acquire) || service_range.load(std::memory_order_acquire) == nullptr) {
        store_current = StoreBlocks; // every packed slot was forgotten above: all blocks go back
        StoreReclaim();
        delete[] packed_slots.exchange(nullptr, std::memory_order_acq_rel);
        cold_hash.clear();
        cold_since.clear();
        pack_times.clear();
        // The swap file is empty again and takes no storage; it stays open, with what was written
        // counted on, for the next game of this session. (Every slot's units went back above; the
        // map is cleared whole all the same: a unit left taken would be lost to every later game.)
        for (auto& word : swap_map) word.store(0, std::memory_order_relaxed);
        swap_used_units.store(0, std::memory_order_relaxed);
        swap_slots.store(0, std::memory_order_relaxed);
        swap_hint = 0;
        if (swap_fd >= 0) (void)ftruncate(swap_fd, 0);
        delete[] std::exchange(idle_since, nullptr);
        delete[] std::exchange(touch_since, nullptr);
        delete[] std::exchange(touch_times, nullptr);
        delete[] std::exchange(pack_raw, nullptr);
        delete[] std::exchange(swap_rewritten, nullptr);
        delete[] std::exchange(rewrite_noted, nullptr);
        delete[] std::exchange(swap_age_class, nullptr);
    }
#else
    const std::size_t slots = (range.end - range.begin.load(std::memory_order_relaxed)) / SparseSlot;
    for (std::size_t slot = 0; slot < slots; ++slot) {
        if (SlotState(range, slot) == SlotOwn) guest_committed.fetch_sub(SparseSlot, std::memory_order_relaxed);
        if (SlotState(range, slot) == SlotFine) guest_committed.fetch_sub(FineKept(range, slot) * FinePage, std::memory_order_relaxed);
        range.owned[slot].store(false, std::memory_order_relaxed);
    }
#endif
}
} // namespace

// The driver ran out of device memory (vulkan_memory_allocator.cpp): packs the coldest slots now,
// those unwritten for 10 s or more, until `bytes` went back. Returns what went back.
std::atomic<std::uint64_t> recover_calls{0}, recovered_bytes{0};
namespace {
#ifdef PS5_NATIVE
// The last resort of RecoverMemory: blocks the service holds idle (kept for unpacking, then the
// guest's reserve) go back to the pool. Late in a long session without Pro memory the heap and
// the JIT were refused their next piece and the app ended, while 768 MiB sat in these two; a
// guest write or unpack that later finds the pool empty asks for packing as before.
std::size_t GiveBackIdleBlocks(std::size_t bytes, bool quiet = false) noexcept {
    std::size_t freed = 0;
    while (freed < bytes) {
        std::int64_t physical = -1;
        Spin(unpack_keep_lock);
        if (const std::size_t count = unpack_keep_count.load(std::memory_order_relaxed); count > 0) {
            physical = unpack_keep[count - 1];
            unpack_keep_count.store(count - 1, std::memory_order_relaxed);
        }
        unpack_keep_lock.clear(std::memory_order_release);
        if (physical < 0) {
            Spin(reserve_lock);
            if (const std::size_t count = reserve_count.load(std::memory_order_relaxed); count > 0) {
                physical = reserve[count - 1];
                reserve_count.store(count - 1, std::memory_order_relaxed);
            }
            reserve_lock.clear(std::memory_order_release);
        }
        if (physical < 0) break;
        if (sceKernelReleaseDirectMemory(physical, SparseSlot) != 0) std::abort();
        freed += SparseSlot;
    }
    if (freed != 0) {
        reserve_hold_ns.store(SwapNow() + 2'000'000'000ull, std::memory_order_relaxed);
        idle_given_back.fetch_add(freed, std::memory_order_relaxed);
        if (idle_give_back_calls.fetch_add(1, std::memory_order_relaxed) < 24 && !quiet) {
            char line[96];
            const int length = std::snprintf(line, sizeof(line), "EDEN_CRAM idle blocks given back: %zu MiB\n", freed >> 20);
            if (length > 0) Report(line);
        }
    }
    return freed;
}
#endif
std::atomic<std::uint64_t> recover_short{0};
constexpr unsigned RecoverSlotBits = 23;      // a slot's number in an entry of recover_order
constexpr std::size_t LargeRequest = std::size_t{64} << 20; // see RecoverMemory
constexpr std::uint32_t KeeperColdSeconds = 60; // see RecoverMemory
// `driver`: the driver may be asked to trim (not from a fault handler: the thread may be in the driver).
// `keeper`: the keeper's own call (KeepFree): no memory was refused yet; it never waits for the lock,
// takes nothing from the blocks kept for the guest, and prints nothing. 1: it only packs; 2: it may
// write to the swap file too.
std::size_t RecoverMemory(std::size_t bytes, bool wait, bool driver = true, int keeper = 0) noexcept {
#ifdef PS5_NATIVE
    SparseRange* const range = service_range.load(std::memory_order_acquire);
    // Device memory no resource uses goes back first: it costs nothing later (packing makes the
    // guest fault its data back).
    std::size_t trimmed = 0;
    if (driver && &eden_radv_private_radv_ps5_eden_trim != nullptr)
        trimmed = static_cast<std::size_t>(eden_radv_private_radv_ps5_eden_trim(bytes));
    if (trimmed >= bytes) return trimmed;
    bytes -= trimmed;
    if (range == nullptr || !PackEnabled() || pack_refused.load(std::memory_order_relaxed) || holding_pack)
        return trimmed + (keeper ? 0 : GiveBackIdleBlocks(bytes, !driver));
    std::unique_lock pack_lock{pack_mutex, std::defer_lock};
    if (keeper) {
        if (!pack_lock.try_lock()) return trimmed; // a pass or a request is at it: again in 200 ms
    } else if (wait) {
        pack_lock.lock();
    } else {
        // The heap and the fault handlers: never a wait without end (whoever packs may be waiting for
        // the heap's lock), but up to half a second: a pass writing to the swap file holds the lock
        // that long, and the heap's growth that found it taken got nothing back and failed
        // (std::bad_alloc ended a Mario Kart World session with 1.3 GB of the budget unused).
        for (unsigned tries = 0; !pack_lock.try_lock(); ++tries) {
            if (tries == 500) return trimmed + GiveBackIdleBlocks(bytes, !driver);
            const timespec pause{0, 1'000'000};
            nanosleep(&pause, nullptr);
        }
    }
    holding_pack = true;
    struct Held { ~Held() { holding_pack = false; } } held;
    if (packed_slots.load(std::memory_order_acquire) == nullptr || cold_since.empty() ||
        recover_order.capacity() < cold_since.size() || cold_since.size() >> RecoverSlotBits != 0)
        return trimmed + (keeper ? 0 : GiveBackIdleBlocks(bytes, !driver));
    // What is asked for is all the taker needs, and the pool may have part of it: the driver asks
    // for the whole 560 MiB of one image and 16 more (a texture array of Mario Kart World's, at a
    // track's load), again at each of its tries, and 574 MiB of the guest's memory gave their
    // blocks up each time, up to 250 MiB of them by way of the swap file, whatever the pool had
    // free. Only what the pool lacks is taken now (and no less than LargeRequest, for the takers
    // that come in between).
    const std::size_t pool_free = driver && !keeper ? PoolFreePieces() : 0;
    const std::size_t asked = bytes;
    if (bytes > LargeRequest && pool_free != 0 && pool_free < bytes) bytes = std::max(bytes - pool_free, LargeRequest);
    // Only slots nothing touches are taken: Idle for UrgentIdleSeconds or more, the longest idle
    // first. It used to take Own slots unwritten for 10 s as well, and when those ran out, for 3 s.
    // Unwritten is not unread: at a race start of Mario Kart World such slots were the game's own
    // working memory. Its threads faulted them back in at once (20,000 unpacks in two minutes, each
    // needing a block that the next request took from another such slot), the game held 2.4 GB of
    // the 4.4 GB it uses, and frames took three times as long for the rest of the race. A request
    // that finds no such slot now gets nothing, and its allocation waits or fails.
    // (A slot's state may change while this runs: PackSlot looks at it again under the slot's lock.)
    const std::uint32_t now = ColdNow();
    const bool idle_mode = IdleEnabled() && idle_since != nullptr;
    recover_order.clear();
    if (idle_mode) {
        // (The keeper, with no allocation failed yet, takes what the scanner's own passes would.)
        const std::size_t idle_for = keeper ? IdleSeconds() : UrgentIdleSeconds();
        for (std::size_t slot = 0; slot < cold_since.size(); ++slot)
            if (SlotState(*range, slot) == SlotIdle && now - idle_since[slot] >= idle_for)
                recover_order.push_back(slot);
        // Slots the swap file already has first (giving their blocks up writes nothing). Then those
        // that were never written again after a while unwritten, before memory the game reuses
        // (swap_rewritten), and of each kind the longest unwritten first. The order is fixed here,
        // from what the slots are now (fault handlers change them while this runs): each entry
        // carries its place, then the slot.
        PackedSlot* const in_file = packed_slots.load(std::memory_order_acquire);
        for (std::size_t& entry : recover_order) {
            const std::size_t slot = entry;
            entry = (in_file[slot].swap == 0 ? std::size_t{1} << 63 : 0) | std::size_t{Rewritten(slot)} << 55 |
                    std::size_t{cold_since[slot]} << RecoverSlotBits | slot;
        }
        std::sort(recover_order.begin(), recover_order.end());
        for (std::size_t& entry : recover_order) entry &= (std::size_t{1} << RecoverSlotBits) - 1;
        // More of them for the requests that follow: every slot unwritten for a while is probed now
        // (once a second at most), not at the scanner's next pass.
        static std::uint32_t probed_at = 0;
        if (probed_at != now) {
            probed_at = now;
            std::size_t made = 0;
            for (std::size_t slot = 0; slot < cold_since.size() && made < 4 * IdlePerPass; ++slot)
                made += ProbeSlot(*range, slot, now);
        }
    } else {
        // Without Idle slots (EDEN_CRAM_IDLE=0), as before them: Own slots unwritten for 10 s.
        for (std::size_t slot = 0; slot < cold_since.size(); ++slot)
            if (SlotState(*range, slot) == SlotOwn && now - cold_since[slot] >= 10) recover_order.push_back(slot);
        std::sort(recover_order.begin(), recover_order.end(), [](std::size_t a, std::size_t b) { return cold_since[a] < cold_since[b]; });
    }
    // A load can take the pool from comfortable to empty between two scanner passes: the swap file
    // is made ready here then (the driver's call: a thread that may allocate and print).
    if (driver && (wait || keeper) && idle_mode) (void)SwapStart();
    // Four steps, each only if the ones before did not give enough:
    //   0. What costs no write: slots that compress go to the packed store, slots the swap file
    //      already has give their blocks up.
    //   1. Slots that were never written again after a while unwritten (swap_rewritten) are written
    //      to the file, the longest unwritten first.
    //   2. The blocks the service keeps idle go back (GiveBackIdleBlocks).
    //   3. For a request that would otherwise get nothing, the slots the game reuses are written
    //      too: the budget is what limits the writes, and inside it an allocation that fails ends
    //      the session.
    // The keeper (no allocation failed yet) stops after step 0, or after step 1 when it may write.
    std::size_t freed = 0, given = 0, to_file = 0, last_resort = 0;
    const std::uint64_t written_before = swap_written.load(std::memory_order_relaxed);
    for (int step = 0; step < 4 && freed + given < bytes; ++step) {
        if (step == 2) {
            if (keeper) break;
            given = GiveBackIdleBlocks(bytes - freed, !driver);
            continue;
        }
        if (step != 0 && (!idle_mode || keeper == 1 || swap_state.load(std::memory_order_acquire) == 2)) continue;
        static const bool last_resort_on = EnvOr("EDEN_SWAP_LAST_RESORT", 1) != 0;
        if (step == 3 && !last_resort_on) break;
        // (The keeper leaves the quarter of the burst that is kept for failed allocations, and writes
        // only slots unwritten for KeeperColdSeconds.)
        const Swap swap = step == 0 ? Swap::Reuse : keeper ? Swap::Yes : Swap::Urgent;
        PackedSlot* const entries = packed_slots.load(std::memory_order_acquire);
        recovering = true;
        for (const std::size_t slot : recover_order) {
            if (freed + given >= bytes) break;
            const bool in_file = entries[slot].swap != 0;
            if (step != 0 && !in_file) {
                if ((Rewritten(slot) != 0) != (step == 3)) continue;
                if (keeper && now - cold_since[slot] < KeeperColdSeconds) continue;
            }
            if (PackSlot(*range, slot, swap)) {
                if (pack_released) freed += SparseSlot;
                if (!in_file && entries[slot].swap != 0) {
                    ++to_file;
                    last_resort += step == 3;
                }
                cold_since[slot] = now;
                if (pack_times[slot] < 255) ++pack_times[slot];
            }
        }
        recovering = false;
    }
    if (keeper) {
        keeper_calls.fetch_add(1, std::memory_order_relaxed);
        keeper_bytes.fetch_add(freed, std::memory_order_relaxed);
        return trimmed + freed;
    }
    swap_last_resort.fetch_add(last_resort, std::memory_order_relaxed);
    const auto calls = recover_calls.fetch_add(1, std::memory_order_relaxed);
    recovered_bytes.fetch_add(freed, std::memory_order_relaxed);
    const bool fell_short = freed + given < bytes && recover_short.fetch_add(1, std::memory_order_relaxed) < 100;
    if (driver && (calls < 24 || fell_short || (to_file != 0 && calls < 400))) {
        char line[320];
        const int length = std::snprintf(line, sizeof(line),
            "EDEN_CRAM_PACK recover%s asked=%zuMiB pool_free=%zuMiB packed=%zuMiB idle_blocks=%zuMiB to_swap=%zuMiB "
            "last_resort=%zuMiB swap_written=%lluMiB candidates=%zu\n",
            wait ? "" : "_try", asked >> 20, pool_free >> 20, freed >> 20, given >> 20, to_file * (SparseSlot >> 20),
            last_resort * (SparseSlot >> 20),
            (unsigned long long)((swap_written.load(std::memory_order_relaxed) - written_before) >> 20), recover_order.size());
        if (length > 0) Report(line);
    }
    return trimmed + freed + given;
#else
    (void)bytes;
    (void)wait;
    return 0;
#endif
}
} // namespace

namespace {
// The pool is kept from running empty. Asked back only once an allocation had failed, memory came
// from the thread that draws: 30 requests in a row at a race start of Mario Kart World, each one
// packing slots while a frame waited. So five times a second the keeper's thread looks at what
// the pool has free in 2 MiB pieces, and under EDEN_CRAM_KEEP_FREE_MIB (512; 0: off) it has slots
// nothing has touched for IdleSeconds give their blocks up until 256 MiB more than that is free.
//
// It does that with what costs no write: slots that compress go to the packed store, and slots
// the swap file already has give their blocks up. It used to write slots to the file as well
// (once the pool had stayed under the mark for 3 s). That kept 512 MiB free at all times at the
// storage's cost: in a 30-minute Mario Kart World session the keeper had most of 3 GiB written,
// two thirds of it for slots the game wrote over within minutes, while requests after a failed
// allocation, the only ones that show memory is needed, came 42 times. Free memory also draws
// takers: the graphics caches keep more when the pool has more. EDEN_SWAP_KEEPER=1 lets it
// write again (slots unwritten for KeeperColdSeconds and never written again after a while).
std::size_t KeepFreeMark() noexcept {
    static const std::size_t bytes = EnvOr("EDEN_CRAM_KEEP_FREE_MIB", 512) << 20;
    return bytes;
}
void KeepFree() noexcept {
#ifdef PS5_NATIVE
    const std::size_t mark = KeepFreeMark();
    if (mark == 0 || service_range.load(std::memory_order_acquire) == nullptr || !PackEnabled() ||
        packed_slots.load(std::memory_order_acquire) == nullptr)
        return;
    // One call first: a pool whose largest free block is that large has that much free.
    const auto total = sceKernelGetDirectMemorySize();
    std::int64_t start = 0;
    std::size_t largest = 0;
    if (total <= 0 || &sceKernelAvailableDirectMemorySize == nullptr ||
        sceKernelAvailableDirectMemorySize(0, total, SparseSlot, &start, &largest) != 0 || largest >= mark)
        return;
    const std::size_t free_pieces = PoolFreePieces();
    static std::uint64_t below_since = 0; // the keeper's thread only
    if (free_pieces >= mark) {
        below_since = 0;
        return;
    }
    // Packing in memory at once. To the swap file, when it may at all (EDEN_SWAP_KEEPER=1), only
    // when the pool has stayed under the mark for 3 s or has a quarter of it left: a race start
    // takes 2 GB and gives half of it back within seconds.
    static const bool may_write = EnvOr("EDEN_SWAP_KEEPER", 0) != 0;
    const std::uint64_t now = SwapNow();
    if (below_since == 0) below_since = now;
    const bool write = may_write && (now - below_since >= 3'000'000'000ull || free_pieces < mark / 4);
    (void)RecoverMemory(mark + (std::size_t{256} << 20) - free_pieces, false, true, write ? 2 : 1);
#endif
}
} // namespace

// Waits for the scanner if it is packing (the driver's allocations).
extern "C" std::size_t eden_memory_recover(std::size_t bytes) noexcept { return RecoverMemory(bytes, true); }
// Never waits: for the heap, which may hold its growth lock while the scanner allocates.
extern "C" std::size_t eden_memory_recover_try(std::size_t bytes) noexcept { return RecoverMemory(bytes, false); }


// ---- What the guest's kernel tells the service ----

// Development switches (dev-settings cram=off, cram_scan=off, cram_dedup=off), before the guest's
// memory is taken.
void SetGuestMemoryService(bool on, bool scan, bool dedup) noexcept {
    service_requested.store(on, std::memory_order_relaxed);
    scan_requested.store(scan, std::memory_order_relaxed);
    dedup_requested.store(dedup, std::memory_order_relaxed);
}

// Settings > Performance > Compact memory (off unless chosen): written slots with many pages of
// zeroes keep only the others. It gives back more memory, and a game that writes such a slot again
// waits for it to be made whole (a fault and a 2 MiB copy, about 0.3 ms).
void SetGuestMemoryFine(bool on) noexcept {
    fine_requested.store(on, std::memory_order_relaxed);
}

// The guest's kernel hands pages out: they are committed now only without the service.
void CommitGuestMemoryOnHandOut(std::uint64_t offset, std::size_t bytes) noexcept {
    SparseRange* const range = guest_range.load(std::memory_order_acquire);
    if (range != nullptr && range->promote) return;
    CommitGuestMemory(offset, bytes);
}

// The guest's kernel fills pages (`at` is the backing at `offset`). With the service, a fill with
// zeroes leaves Zero slots alone and gives whole Own or Shared slots back; anything else is written.
void ClearGuestMemory(std::uint8_t* at, std::uint64_t offset, std::size_t bytes, int fill) noexcept {
    SparseRange* const range = guest_range.load(std::memory_order_acquire);
    if (range == nullptr || !range->promote || fill != 0) {
        if (range != nullptr && !range->promote) CommitGuestMemory(offset, bytes);
        std::memset(at, fill, bytes); // (with the service, a write to a Zero slot promotes it)
        return;
    }
    const std::uint64_t end = offset + bytes;
    for (std::uint64_t cursor = offset; cursor < end;) {
        const std::size_t slot = static_cast<std::size_t>(cursor / SparseSlot);
        const std::uint64_t slot_end = (cursor / SparseSlot + 1) * SparseSlot;
        const std::uint64_t piece_end = std::min(end, slot_end);
        const bool whole = cursor % SparseSlot == 0 && piece_end == slot_end;
        if (whole) {
            LockSlot(*range, slot);
            const auto state = SlotState(*range, slot);
            if (state != SlotZero) {
                ReturnSlot(*range, slot);
                counters.returns_clear.fetch_add(1, std::memory_order_relaxed);
            }
            UnlockSlot(*range, slot, SlotZero);
        } else if (SlotState(*range, slot) != SlotZero) {
            std::memset(at + (cursor - offset), 0, static_cast<std::size_t>(piece_end - cursor));
        }
        cursor = piece_end;
    }
}

// A page the guest's kernel fills one at a time (optimized allocation): the same rule.
void FillGuestMemory(std::uint8_t* at, int fill, std::size_t bytes) noexcept {
    SparseRange* const range = guest_range.load(std::memory_order_acquire);
    if (range != nullptr && range->promote && fill == 0) {
        const auto begin = range->begin.load(std::memory_order_relaxed);
        const auto address = reinterpret_cast<std::uintptr_t>(at);
        if (address >= begin && address < range->end && SlotState(*range, (address - begin) / SparseSlot) == SlotZero)
            return;
    }
    std::memset(at, fill, bytes);
}

// The guest's kernel freed pages (a free block of 2 MiB or more in its page heap): every slot they
// cover whole goes back. EDEN_GUEST_FREE_RETURN=0 keeps freed slots held until they are reused.
bool ReturnFreedSlots() noexcept {
    static const bool on = [] {
        const char* value = std::getenv("EDEN_GUEST_FREE_RETURN");
        return value == nullptr || value[0] != '0';
    }();
    return on;
}

void GuestMemoryFreed(std::uint64_t offset, std::size_t bytes) noexcept {
    SparseRange* const range = guest_range.load(std::memory_order_acquire);
    if (range == nullptr || !range->promote || bytes < SparseSlot || !ReturnFreedSlots()) return;
    const std::uint64_t first = (offset + SparseSlot - 1) / SparseSlot;
    const std::uint64_t last = (offset + bytes) / SparseSlot; // exclusive
    for (std::uint64_t slot = first; slot < last; ++slot) {
        LockSlot(*range, static_cast<std::size_t>(slot));
        if (SlotState(*range, static_cast<std::size_t>(slot)) != SlotZero) {
            ReturnSlot(*range, static_cast<std::size_t>(slot));
            counters.returns_free.fetch_add(1, std::memory_order_relaxed);
        }
        UnlockSlot(*range, static_cast<std::size_t>(slot), SlotZero);
    }
}

// The guest's kernel freed pages that do not make a free block of 2 MiB (a region's tail, a page
// group's blocks): GuestMemoryFreed never sees them, and the slot they sit in stays held whole.
// MKW unmaps ~1.7 GB in a 5-minute session in pieces of ~370 KiB, and ~640 MiB of it was never
// given back. Nothing in the guest can read freed pages (the kernel fills or hands them out again
// before use), so they are cleared where the slot has a block of its own; the scanner then gives
// back the slot (all zero) or its pages of zeroes (Fine). EDEN_GUEST_FREE_ZERO=0 leaves them.
bool ZeroFreedParts() noexcept {
    static const bool on = [] {
        const char* value = std::getenv("EDEN_GUEST_FREE_ZERO");
        return value == nullptr || value[0] != '0';
    }();
    return on;
}

void GuestMemoryFreedPart(std::uint64_t offset, std::size_t bytes) noexcept {
    SparseRange* const range = guest_range.load(std::memory_order_acquire);
    if (range == nullptr || !range->promote || bytes == 0 || !ZeroFreedParts()) return;
    const std::size_t slots = (range->end - range->begin.load(std::memory_order_relaxed)) / SparseSlot;
    freed_part_calls.fetch_add(1, std::memory_order_relaxed);
    for (std::uint64_t at = offset, end = offset + bytes; at < end;) {
        const std::size_t slot = static_cast<std::size_t>(at / SparseSlot);
        const std::uint64_t slot_end = std::min<std::uint64_t>(end, (at / SparseSlot + 1) * SparseSlot);
        if (slot >= slots) break;
        LockSlot(*range, slot);
        const std::uint8_t state = SlotState(*range, slot);
        if (state == SlotOwn) {
            std::memset(reinterpret_cast<void*>(SlotAddress(*range, slot) + at % SparseSlot), 0,
                        static_cast<std::size_t>(slot_end - at));
            freed_part_bytes.fetch_add(slot_end - at, std::memory_order_relaxed);
        }
        UnlockSlot(*range, slot, state);
        at = slot_end;
    }
}

// Host memory behind [offset, offset + bytes) of the guest's memory (the memory report splits it
// by the guest kernel's pools). Whole slots count 2 MiB, Fine slots their kept pages.
std::size_t GuestMemoryHeldIn(std::uint64_t offset, std::size_t bytes) noexcept {
    SparseRange* const range = guest_range.load(std::memory_order_acquire);
    if (range == nullptr || range->state == nullptr || bytes == 0) return 0;
    const std::size_t slots = (range->end - range->begin.load(std::memory_order_relaxed)) / SparseSlot;
    std::size_t held = 0;
    const std::size_t last = std::min<std::size_t>(slots, (offset + bytes + SparseSlot - 1) / SparseSlot);
    for (std::size_t slot = static_cast<std::size_t>(offset / SparseSlot); slot < last; ++slot) {
        const auto state = SlotState(*range, slot);
        if (state == SlotOwn || state == SlotShared || state == SlotIdle || state == SlotClean) held += SparseSlot;
        else if (state == SlotFine) held += FineKept(*range, slot) * FinePage;
    }
    return held;
}

// For the host checks: one scan pass now, and a slot's state (0 Zero, 1 Own, 2 Shared, -1 none).
void GuestMemoryScanNow() noexcept {
    if (SparseRange* const range = service_range.load(std::memory_order_acquire)) ScanPass(*range);
}
int GuestMemorySlotState(std::uint64_t offset) noexcept {
    SparseRange* const range = guest_range.load(std::memory_order_acquire);
    if (range == nullptr || range->state == nullptr) return -1;
    return SlotState(*range, static_cast<std::size_t>(offset / SparseSlot));
}

// ---- The 64-bit fastmem window (src/host_memory.cpp, Window64; backported from ProsperoEden) ----

// From a JIT fault in the window: shows the slot at `backing` (2 MiB aligned) at `window_at`. An Own
// slot shows its block, read-write when `writable`; a Zero or Shared slot shows its block read-only
// for a read. A write to a slot without a block of its own sets `promote` and shows nothing: the
// caller promotes it as a write to the backing would and asks again. False otherwise (the access
// then takes the page-table path).
bool AliasGuestSlot(std::uintptr_t backing, std::uintptr_t window_at, bool writable, bool write, bool* promote) noexcept {
#ifdef PS5_NATIVE
    auto* range = guest_range.load(std::memory_order_acquire);
    if (!range || !range->state || !range->aliases) return false;
    const std::uintptr_t begin = range->begin.load(std::memory_order_relaxed);
    if (backing < begin || backing >= range->end || (backing - begin) % SparseSlot || window_at % SparseSlot) return false;
    const std::size_t slot = (backing - begin) / SparseSlot;
    LockSlot(*range, slot);
    const std::uint8_t state = SlotState(*range, slot);
    std::int64_t physical = -1;
    if (state == SlotOwn) {
        physical = range->physical[slot];
    } else if (write || state == SlotPacked || state == SlotIdle) {
        *promote = true; // (a Packed or Idle slot is woken by any access, as through the backing)
    } else if (state == SlotZero) {
        physical = zero_block, writable = false;
    } else if (state == SlotShared || state == SlotClean) {
        physical = range->physical[slot], writable = false;
    }
    bool shown = false;
    if (physical >= 0) {
        const int protection = writable ? PROT_READ | PROT_WRITE : PROT_READ;
        const std::uintptr_t entry = window_at | (writable ? 0 : AliasReadOnly);
        std::uintptr_t* free_entry = nullptr;
        bool known = false;
        for (unsigned i = 0; i < AliasesPerSlot; ++i) {
            auto& alias = range->aliases[slot * AliasesPerSlot + i];
            if ((alias & ~AliasFlags) == window_at) {
                // Another core showed it first, or it shows it read-only or in pieces.
                known = true;
                shown = alias == entry || (MapSlot(window_at, protection, physical) && ((alias = entry), true));
                break;
            }
            if (!alias && !free_entry) free_entry = &alias;
        }
        if (!known && free_entry && MapSlot(window_at, protection, physical)) {
            *free_entry = entry;
            shown = true;
            aliases_made.fetch_add(1, std::memory_order_relaxed);
        }
    }
    if (!shown) aliases_refused.fetch_add(1, std::memory_order_relaxed);
    UnlockSlot(*range, slot, state);
    return shown;
#else
    (void)backing, (void)window_at, (void)writable, (void)write, (void)promote;
    return false; // the host's guest memory is private: nothing to alias
#endif
}

// From a JIT fault in a window region that cannot show a whole block (not on a 2 MiB boundary of the
// backing, or partly protected): shows the 16 KiB at `backing` of an Own slot at `at`, inside the
// window region `region_at`. The slot's entry names the whole region, so any change of the slot voids
// it. A write to a slot that is not Own sets `promote`, as for AliasGuestSlot.
bool AliasGuestPage(std::uintptr_t backing, std::uintptr_t at, std::uintptr_t region_at, bool writable, bool write,
                    bool* promote) noexcept {
#ifdef PS5_NATIVE
    auto* range = guest_range.load(std::memory_order_acquire);
    if (!range || !range->state || !range->aliases) return false;
    const std::uintptr_t begin = range->begin.load(std::memory_order_relaxed);
    if (backing < begin || backing >= range->end || (backing - begin) % FinePage || at % FinePage) return false;
    const std::size_t slot = (backing - begin) / SparseSlot;
    LockSlot(*range, slot);
    const std::uint8_t state = SlotState(*range, slot);
    bool shown = false;
    // A Zero slot shows the zero block and a Shared one its shared block, read-only; a write there
    // promotes the slot first (the caller). A Fine slot keeps its pages apart: not shown.
    const std::size_t offset = backing - SlotAddress(*range, slot);
    std::int64_t physical = -1;
    if (state == SlotOwn || (!write && (state == SlotShared || state == SlotClean))) {
        physical = range->physical[slot] >= 0 ? range->physical[slot] + std::int64_t(offset) : -1;
    } else if (!write && state == SlotZero) {
        physical = zero_block + std::int64_t(offset);
    } else if (write || state == SlotPacked || state == SlotIdle) {
        *promote = true;
    }
    if (state != SlotOwn) writable = false;
    if (physical >= 0) {
        std::uintptr_t* entry = nullptr;
        bool known = false;
        for (unsigned i = 0; i < AliasesPerSlot && !known; ++i) {
            auto& alias = range->aliases[slot * AliasesPerSlot + i];
            if ((alias & ~AliasFlags) == region_at) entry = &alias, known = true;
            else if (!alias && !entry) entry = &alias;
        }
        void* address = reinterpret_cast<void*>(at);
        if (entry && sceKernelMapDirectMemory(&address, FinePage, writable ? PROT_READ | PROT_WRITE : PROT_READ, MAP_FIXED,
                                              physical, FinePage) == 0 && address == reinterpret_cast<void*>(at)) {
            if (!known) aliases_made.fetch_add(1, std::memory_order_relaxed);
            *entry = region_at | AliasSmall;
            shown = true;
        }
    }
    if (!shown) aliases_refused.fetch_add(1, std::memory_order_relaxed);
    UnlockSlot(*range, slot, state);
    return shown;
#else
    (void)backing, (void)at, (void)region_at, (void)writable, (void)write, (void)promote;
    return false;
#endif
}

// The window region at `window_at` no longer shows the slot at `backing` (guest unmap, protection).
// True when it did.
bool DropGuestAlias(std::uintptr_t backing, std::uintptr_t window_at) noexcept {
    auto* range = guest_range.load(std::memory_order_acquire);
    if (!range || !range->state || !range->aliases) return false;
    const std::uintptr_t begin = range->begin.load(std::memory_order_relaxed);
    if (backing < begin || backing >= range->end) return false;
    const std::size_t slot = (backing - begin) / SparseSlot;
    bool dropped = false;
    LockSlot(*range, slot);
    for (unsigned i = 0; i < AliasesPerSlot; ++i)
        if (auto& alias = range->aliases[slot * AliasesPerSlot + i]; alias && (alias & ~AliasFlags) == window_at) {
            VoidWindowRegion(window_at);
            alias = 0;
            dropped = true;
            aliases_dropped.fetch_add(1, std::memory_order_relaxed);
        }
    UnlockSlot(*range, slot, SlotState(*range, slot));
    return dropped;
}

void GuestAliasCounts(std::uint64_t* made, std::uint64_t* dropped, std::uint64_t* refused) noexcept {
    *made = aliases_made.load(std::memory_order_relaxed);
    *dropped = aliases_dropped.load(std::memory_order_relaxed);
    *refused = aliases_refused.load(std::memory_order_relaxed);
}
