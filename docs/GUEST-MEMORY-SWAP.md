# Swap for the guest's memory

Status, 2026-11-23: implemented in `src/guest_memory_service.inc`, on by default, tested with
Mario Kart World and Ocarina of Time on a PS5 Pro. `"performance": { "swap_file": false }` in the
settings file turns it off, as does `EDEN_SWAP=0` in `env.txt`.

Mario Kart World's crashes turned out to have two causes, and the first is one no swap file can
cover. Both are described under [Mario Kart World](#mario-kart-world-two-causes).

In short, on a PS5 Pro: Mario Kart World, which ended early on every try, runs 30-minute
sessions at 30 to 31 FPS with 2.3 to 2.8 GiB written to the storage. Without the swap file the same
session ends after 10 minutes. Its first races (5 minutes) and Ocarina of Time write nothing.
The numbers are under [Results](#results).

## What it is for

The guest's memory, the driver's GPU memory and Eden's heap share one pool of direct memory
(13,440 MiB with PS5 Pro memory, about 1.2 GB less without). When the pool runs out, an allocation
fails and the session ends.

The guest memory service already packs slots (2 MiB pieces of the guest's memory) with LZ4 when the
pool is short. That gives back only what LZ4 saves. Texture, model and sound data does not compress,
and a game keeps a lot of it once the GPU has its copy. The swap file gives those slots somewhere to
go: the console's internal storage. A slot written to the file gives its 2 MiB block back to the
pool. The next access reads it back.

## How writes are kept low

The storage's flash wears with every byte written, so the design starts from the writes.

- **Only when an allocation has failed.** Slots go to the file when an allocation found no memory
  and asked for some back, and at no other time. A keeper thread does try to hold 512 MiB of the
  pool free (`EDEN_CRAM_KEEP_FREE_MIB`), but only with what costs no write: slots that compress,
  and slots the file already has. A game that fits writes one 16 KiB test page when the pool first
  runs short, and nothing else.
- **Only what the pool lacks.** A request names all its taker needs: the driver asks for the
  whole 560 MiB of one texture array when Mario Kart World loads a track, and again at each of
  its tries. The pool often has part of that free already. Only the difference is taken from the
  guest (and no less than 64 MiB).
- **Memory first.** A slot that compresses to 60% or less goes to the packed store in memory, as
  before. Only the others go to the file, LZ4-compressed when that saves a tenth or more.
- **Only slots nothing touches.** While the pool is short the scanner takes all access away from
  slots that have not been written for 8 s (the *Idle* state: the block stays, only its protection
  changes). A slot the game or the emulator touches faults once and is left alone for longer each
  time, up to 40 s. Only slots still Idle after 2 s are taken, the longest unwritten first. When
  there are none, the request gets nothing: the game's working memory is never written to the file.
- **Written once.** A slot read back from the file is *Clean*: read-only, and the file keeps its
  bytes. Giving a Clean slot's block up again writes nothing. Only a write to the slot drops the
  file's copy.
- **Memory the game reuses goes last.** A game loads its next level into the memory the last one
  used. Such memory sits untouched for minutes and looks like the best there is to take, and what
  is written to the file for it is thrown away at the game's next write. In a 14-minute Mario Kart
  World session 684 of 979 slots written ended that way, whether they had been unwritten for one
  minute (79%) or for five to ten (61%). How long a slot has been unwritten does not tell the two
  kinds apart; its history does. The scanner already samples every slot to see when it was last
  written, so it counts, for nothing, each time a slot is written again after 30 s or more
  unwritten. Slots never counted go to the file first, the longest unwritten first. The others go
  only when a request would otherwise get nothing (`EDEN_SWAP_LAST_RESORT=0`: never).
- **A budget.** At most `EDEN_SWAP_BURST_MIB` at once, refilled at `EDEN_SWAP_RATE_MIB_MIN` a
  minute, and no more than `EDEN_SWAP_SESSION_MIB` a session or `EDEN_SWAP_DAY_MIB` a day. With
  the budget used up nothing more is written; what the file holds stays readable, and the game
  runs as it did without swap.
- **A ledger that never counts less than was written.** A one-line file beside the swap file
  (`guest-memory.swap.ledger`) carries the day's and the lifetime's totals from session to session.
  It is charged before the writes it counts, 32 MiB ahead, so a session that ends without warning
  (out of memory, the power button) is counted in full, and at most 32 MiB over. A session that
  ends in order leaves the exact count. If the ledger cannot be written, nothing else is.
- **Never synced, never left behind.** The file is unlinked as soon as it is open, so the storage
  takes it back when the app ends, however it ends. It is at most `EDEN_SWAP_MIB`, and it is not
  made when the storage has less than `EDEN_SWAP_KEEP_FREE_MIB` free.

| Switch (`env.txt`) | Default | Meaning |
|---|---|---|
| `EDEN_SWAP` | `1` | `0`: no swap file (also `"swap_file": false` in the settings file) |
| `EDEN_SWAP_MIB` | 2048 | Most the file may hold |
| `EDEN_SWAP_BURST_MIB` | 1024 | Most that may be written at once |
| `EDEN_SWAP_RATE_MIB_MIN` | 128 | Refill of the burst, per minute |
| `EDEN_SWAP_SESSION_MIB` | 4096 | Most written in one session |
| `EDEN_SWAP_DAY_MIB` | 8192 | Most written in one day, over all sessions |
| `EDEN_SWAP_KEEP_FREE_MIB` | 8192 | Storage space the file never takes |
| `EDEN_SWAP_LAST_RESORT` | 1 | `0`: memory the game reuses is never written, and such a request gets nothing |
| `EDEN_SWAP_KEEPER` | 0 | `1`: the keeper may write too, once the pool has stayed short for 3 s |
| `EDEN_SWAP_LOG` | 0 | `1`: one `EDEN_SWAP_PASS` line per scanner pass |
| `EDEN_CRAM_KEEP_FREE_MIB` | 512 | Free pool the keeper holds; `0`: no keeper |
| `EDEN_CRAM_IDLE` | 1 | `0`: no Idle slots (packing chooses by writes, and no swap) |

### What the day's limit means for the storage

The console's internal drive is 825 GB to 2 TB of flash, depending on the model. Sony does not
publish its endurance. Consumer TLC drives of about 1 TB are commonly rated near 600 TBW; taking
that as a guide, a console that reached the 8 GiB limit every single day would write 3.1 TB a
year, about 0.5% of it. Of the sessions measured below only Mario Kart World's 30 minutes wrote
gigabytes (2.3 and 2.8 GiB); its 5-minute sessions and Ocarina of Time wrote nothing.

## How it is kept safe

- The file is tested once (a page written and read back) before any slot goes to it.
- A slot's 2 MiB are hashed when written. What is read back is checked against the hash before the
  game sees it. Data that does not match ends the session with a line in the log; it never reaches
  the game. (24,795 slots were read back in the test runs kept; none failed.)
- A write that fails or is cut short (storage full, an I/O error) leaves the slot as it was, with
  its block, and ends swapping for the session.
- No slot is ever readable half-written or half-read: a slot is locked while it changes, has no
  access until its data is complete, and is unpacked in a mapping of its own before it is shown.
- The system never reads guest memory on the app's behalf. A slot that is Idle or in the file has
  no access, and a system call given a pointer into it (a file write, a socket send) would fail
  instead of faulting it back in. The input buffers of the guest's service requests are now always
  copies made by the emulator's own read (`hle_ipc.cpp`), and slots being written are never made
  Idle.
- Without storage access (`/data` not reachable) there is no swap file.

## Mario Kart World: two causes

On `main` Mario Kart World ended early on every try (48 s after launch in this set's run, and
in all 6 runs of another series the day before). Two things were behind that, and the swap file
is the answer to the second one only.

### The render resolution, changing every frame

Mario Kart World chooses its render resolution frame by frame, from how long the GPU took for
the last frames. That is what filled the pool, and the swap file cannot fix it: it fed it.

What happened, measured with `EDEN_NX2_TC_SHAPES=1` and `EDEN_NX2_MEM_TRACE=1`:

1. A frame comes late. Any cause will do: a request for memory that packs slots while the frame
   waits, slots read back from the file, the game streaming the next part of a track.
2. The game lowers its render resolution, then changes it again every frame: 1616x900, 1604x900,
   1608x904, and so on. Every size needs its own set of render targets, at new addresses.
3. The texture cache makes them: 25,000 to 28,000 images in 600 frames instead of 1,000 to 5,000,
   in 3,000 to 4,000 different shapes. GPU-written images go from 0.48 GB to 1.2-2.2 GB, the
   driver's memory from 4.5 GB to 6-7.6 GB, within seconds.
4. The pool runs out. Allocations wait for memory, frames take 60 to 120 ms with single frames of
   1 to 5 s, the resolution keeps changing, and the state holds itself up until something fails.

It started in three places: at the main menu (the world is drawn behind it), in the first seconds
of a race, and about 70 s into a race, when the next part of the track streams in.

The fix is the GPU clock, a setting the core already has: how much shorter the GPU's time for a
frame looks to the game. With boost (the core's default) it looks 4 times shorter, with overclock
16 times. The port adds a third step, fixed: 256 times (`gpu_clock_divisor`,
`headless/performance.h`, read in `GPU::GetTicks`).

| Mario Kart World, PS5 Pro, GPU clock | Runs | Ended early |
|---|---|---|
| boost, `main` | 1 (and 6 the day before) | all (this one after 48 s) |
| boost, with the swap file (its variants while it was made), 5 minutes | 21 | 13; 3 more ran at 12 to 22 FPS |
| overclock, with the swap file, 5 minutes | 9 | 0 |
| overclock, with the swap file, 10 to 30 minutes | 7 | 6, after 8 to 17 minutes |
| fixed, with the swap file, 30 minutes | 4 | 0 |
| fixed, no swap file, 5 minutes | 2 | 0 |
| fixed, no swap file, 30 minutes | 1 | 1, after 10 minutes |

Overclock holds the resolution through frames of up to about 270 ms. That is enough for a race.
In a session a longer frame comes sooner or later: a request for a 128 MiB staging buffer that
waited 300 ms for memory set the loop off 9 and 14 minutes into two sessions, and both ended
within the minute. With fixed, frames of 1 to 12 s (allocations waiting for memory late in a
session) did not: the cache made at most 8,914 images in 600 frames.

Mario Kart World gets fixed unless the settings file says otherwise
(`"performance": { "gpu_clock": "normal" | "boost" | "overclock" | "fixed" }`, for every game or
under `"games"` for one; `EDEN_GPU_CLOCK` in `env.txt` goes before both). Other games keep boost.
The game then renders at its full resolution all the time. On a PS5 Pro that costs nothing that
could be measured: the 5-minute runs with overclock (33.3 to 35.9 FPS) and with fixed (34.4 to
35.5) are in the same range.

### A session that needs more than the pool has

With the GPU clock fixed and no swap file, Mario Kart World runs its first races: 2 of 2
five-minute runs at 34 FPS, with no allocation refused at all. The pool is still too small for a
session. Each track brings its own textures, among them arrays the emulator has to decode (one
of 1434x1434 with 46 layers takes 560 MiB of graphics memory), and what the game holds grows
with every race. Ten and a half minutes into the session without the swap file an allocation
found the pool empty (4 MiB free of 13,440), 99 of 118 requests for memory had come back with
nothing, and the session ended. With the swap file the same session ran its 30 minutes; see
[Results](#results).

## What else changed, and why

Testing the swap file showed faults that are not about the file itself. Each ended a Mario Kart
World session on its own.

1. **Memory asked back was counted but not given back.** A recovery told the driver "32 MiB back"
   while the blocks had gone to refill the service's own stock for unpacking. The driver's second
   try found nothing. A recovery now gives blocks straight to the pool and counts only those.
2. **One retry.** The driver asked for memory back once and tried once more. By then another taker
   often had it. Buffers and images are now tried again for up to 2 s after asking memory back
   (`RetryAfterRecovery`, `vulkan_memory_allocator.cpp`), the heap for up to 2 s, and a guest
   write that waits for a block asks for one itself instead of only waiting for the caches.
3. **A mapping taken away under its writer.** The heap gave memory back with `munmap`, then reserved
   the range again. In between, a mapping another thread asked for without a fixed address could
   land there, and the reservation then removed it. The guest memory service unpacks and zeroes
   blocks through such mappings, inside fault handlers. A fault there ends the app with no crash
   report: this was the "silent exit" at Mario Kart World's second race load. Ranges are now
   reserved in one call over the mapping (`ReserveOver`), and the service's views have address space
   of their own (`MapView`).
4. **The game's working memory was packed.** A recovery took slots unwritten for 10 s. Unwritten is
   not unread: the game's threads faulted them back in at once (20,000 unpacks in two minutes) and
   frames took three times as long. A recovery now takes only Idle slots.
5. **Memory was asked back by the thread that draws.** Each request packed or wrote slots while a
   frame waited. A keeper thread now looks at the pool five times a second and frees what costs no
   write before an allocation has to ask.
6. **A block in one piece, asked of 2 MiB blocks.** The heap wanted an 88 MiB block in one piece
   when the next part of a track streamed in. The pool had none, and guest memory was packed and
   written to the file twenty times over while the thread waited, although no amount of 2 MiB
   blocks makes one 88 MiB block. The heap now takes such a block in pieces at once.
7. **The packed store kept its blocks.** A block of the packed store went back only when all of its
   data had. Ten minutes into a session the store held 544 MiB of blocks for 282 MiB of data. Data
   in blocks less than half full now moves together (`StoreCompact`).
8. **A taker that did not ask.** The emulator's sparse tables (the GPU's page tables among them)
   and the guest's memory when it is handed out take their 2 MiB pieces in `CommitSparsePage`,
   which ended the app at the first refusal. 21 minutes into a session a piece of the GPU's page
   tables found 1 MiB free, with 1 GB of Idle slots and most of the swap budget unused (1 of the 4
   thirty-minute sessions run after the keeper stopped writing to the file; while it wrote, it
   kept more of the pool free). The commit now asks for memory back and is tried again for up to 2 s
   (`CommitWithRecovery`, `memory_pages.cpp`; `EDEN_SPARSE_COMMIT_TEST=N` refuses the first try
   of every N-th commit, to test that path: with 20, a 5-minute session ran at 35.0 FPS and each
   refused commit had its memory at the next try). Two 30-minute sessions since then ran through.

## Results

A PS5 Pro (the pool is 13,440 MiB) unless said otherwise, release builds with scripted input.
Mario Kart World at 1x: from the title into a Grand Prix, race after race; the first race starts
about two minutes after launch. FPS and frames over 100 ms are counted from 2:30 on. "Written"
is what went to the storage in the session.

### Mario Kart World, 5 minutes

| | Runs | Ended early | FPS | Frames over 100 ms | Written |
|---|---|---|---|---|---|
| `main` | 1 (and 6 the day before) | all (this one after 48 s) | | | |
| this branch | 6 | 0 | 34.4 to 35.5 | 0 to 1 | 0 |
| this branch, `EDEN_SWAP=0` | 2 | 0 | 34.1 to 34.2 | 0 to 1 | 0 |

No allocation was refused in any of the eight: the first races fit once the render resolution
holds still, and the file is not written to.

### Mario Kart World, 30 minutes

| | Outcome | FPS | Frames over 100 ms | Worst frame | Written | Slots written | Dropped by a later write |
|---|---|---|---|---|---|---|---|
| `EDEN_SWAP=0` | ended at 10:29 (out of device memory) | 30.7 | 21 | 12.6 s | 0 | | |
| the keeper writes, copies ahead (as first built) | ran | 30.8 | 116 | 11.1 s | 3,429 MiB | 2,139 | 1,594 (75%) |
| + the count of reuse, last resort | ran | 30.9 | 111 | 7.5 s | 3,762 MiB | 2,326 | 1,630 (70%) |
| on request only, no copies | ran | 30.7 | 116 | 5.0 s | 2,757 MiB | 1,705 | 948 (56%) |
| + only what the pool lacks, 3 runs | 2 ran; 1 ended at 21:14 (a table's commit, see above) | 30.5, 31.2 | 129, 124 | 4.2 s, 4.8 s | 2,287 and 2,765 MiB | 1,414, 1,717 | 546 (39%), 896 (52%) |
| + commits that ask (this branch), 2 runs | ran | 30.6, 30.8 | 148, 128 | 5.7 s, 5.2 s | 2,728 and 2,490 MiB | 1,681, 1,533 | 864 (51%), 708 (46%) |

A long session of this game needs more than the pool holds: at the end of the last four runs
1.5 GB of its memory had their bytes in the file. Most of what it can give up is memory it reuses,
which is why so many slots are dropped by the game's own next write however they are chosen.
What the choices above changed is how much is written for the same 30 minutes; the frame rate
is the same in all of them. Writing on request only is what made the difference; taking only
what the pool lacks shows no effect of its own beyond the 0.5 GiB that lie between runs of the
same build. In the first run of that row 450 MiB had been written after 21 minutes. The rest went at two loads, 580 MiB at 22 minutes and
1,250 MiB in the last three: a track's textures (the 560 MiB array and a 128 MiB staging buffer
among them) found the pool empty and nothing left to take but memory the game reuses (996 to
1,136 slots as a last resort in the four runs, two thirds of the slots they wrote).
`EDEN_SWAP_LAST_RESORT=0` leaves such requests empty-handed instead; some allocations can do
without (an image is not made), others end the session, as the run without the file shows.

The budget never refused a write in these runs. At this rate the session's limit (4 GiB) is
reached after about 45 minutes of Mario Kart World, and the day's (8 GiB) by two such sessions.
When the budget is used up the session ends the way it does without the file, at its next
large need. With the session's limit set to 512 MiB for a test, the limit was reached 13.5
minutes in (510 MiB written), 104,469 slots were refused from then on, and the session ended
three and a half minutes later, out of device memory at a load, with 1.1 GB of Idle slots that
could no longer be written. A longer Mario Kart World session needs a higher
`EDEN_SWAP_SESSION_MIB` (and `EDEN_SWAP_DAY_MIB`), at about 85 MiB a minute.

### The Legend of Zelda: Ocarina of Time

| | Runs | FPS | Frames over 100 ms | Written |
|---|---|---|---|---|
| this branch, 3 minutes | 6 | 33.5 to 34.2 | 0 | 0 |

The pool gets short (its largest free block goes down to 0.9 GB, and 2.9 GB of the game's memory
sit Idle), no allocation is refused, and nothing is written but the test page. `main` ran the
same scene at 33.2 and 33.7 FPS the same night (2 runs; by the game's frames, which gives 33.6
to 34.2 for the six runs above).

### Donkey Kong Bananza

One 3-minute run each: 30.5 FPS on this branch, 30.2 on `main`. The pool never gets short (its
largest free block stays above 6 GB), so the swap file is not even made.

### Without PS5 Pro memory

The same console with the app's memory mode of a PS5 (12,224 MiB: `attribute3` `0x80040` in
`param.json`).

| | Runs | Ended early | FPS | Frames over 100 ms | Worst frame | Written |
|---|---|---|---|---|---|---|
| Mario Kart World, 5 minutes | 2 | 0 | 33.8 to 34.9 | 6 to 8 | 0.39 s | 433 and 479 MiB |
| Ocarina of Time, 3 minutes | 1 | 0 | 33.7 | 0 | 0.06 s | 0 |

With 1.2 GB less, Mario Kart World's first race does need the file: 268 and 304 slots went to
it, none as a last resort, and no request came back empty. (On 2026-11-07 a build of that day
ended 4 of 4 times in this mode, 80 to 154 s into the session.) A longer session was not run in
this mode: it will reach the budget's limits sooner than with PS5 Pro memory.

### What the tests wrote to this console

58.6 GiB in two days (the ledger's `written_ever` after the last run), most of it by the
variants that wrote ahead of need and by the 30-minute sessions. The tests ran with the day's
limit raised (`EDEN_SWAP_DAY_MIB=65536`); with the default the day's 8 GiB would have ended
the writing on both days.

## What was tried and dropped

- **Writing ahead of need** (slots idle for 30 s go to the file while the pool has less than 3 GiB
  free). The race then started with the room it needs. But 1 GB had gone to the file, and the game
  wanted 800 MiB of it back within seconds when it streamed the next part of the track. 3 of 3
  runs ended worse than without it, with 2.0 to 2.3 GiB written each. The switch is gone.
- **Taking slots that are only unwritten** when the idle ones run out. It fed the state described
  under Mario Kart World instead of ending it: 0.7 to 2.0 GiB written per run for nothing.
- **The keeper writing to the file** to hold 512 MiB free at all times. In a 30-minute Mario Kart
  World session it had most of 3 GiB written, two thirds of it for slots the game wrote over
  within minutes, while requests after a failed allocation came 42 times. Free memory also draws
  takers: the graphics caches keep more when the pool has more.
- **Copies made ahead** for slots that are read and not written (while the slot stays, so that a
  busy moment can take it for nothing). 248 of 280 copies were dropped by a write: early in a
  session nothing tells memory the game keeps from memory it has not reused yet.
- **Letting reused memory go to the file after 2, 4, 8, then 16 minutes unwritten.** Of the slots
  written after one and two such counts, 74% and 83% were dropped by the game's next write.

## Reading the log

`EDEN_SWAP now on=1 (ok) out=152MiB clean=0MiB file_used=119MiB written=116MiB written_today=…
written_ever=… writes=77 raw=7 reused=0 dirtied=0 reads=1 read_avg_us=1366 … kept_free=210MiB/1
last_resort=0`

- `out`: guest memory whose only copy is in the file now. `clean`: resident, with a copy there.
- `written`: bytes written this session; `written_today`, `written_ever`: with the ledger's.
- `raw`: slots written as they were (LZ4 saved under a tenth).
- `reused`: Clean slots that gave their block up again without a write.
- `dirtied`: slots whose copy in the file a write dropped.
- `denied_rate`, `denied_cap`: slots not written because the budget said no.
- `idle_now`, `idled`, `woken`: Idle slots now, made, and touched again.
- `kept_free`: what the keeper freed before any allocation failed, and in how many passes.
- `last_resort`: slots the game reuses that were written because a request had nothing else.

`EDEN_SWAP_AGES now written=a/b/c/d/e/f dropped=… by_rewrites=g/h/i/j dropped=… slots=k/l/m/n`:
the slots written by how long they had been unwritten (under 30 s, 60 s, 2, 5, 10 minutes, more)
and by how often they had been written again before (never, once, twice, more), each with how
many a later write dropped; then all slots that hold data, by the same count.

`EDEN_CRAM_PACK recover asked=… pool_free=… packed=… idle_blocks=… to_swap=… last_resort=…
swap_written=… candidates=…` is one request for memory back after a failed allocation: what was
asked, what the pool had free then, what went back to the pool from slots and from the service's
idle blocks, how much of it by way of the file, and how many Idle slots there were to take.

`EDEN_SPARSE_TABLES a commit found no memory and got it after asking (N so far, try T)` says that
a table's or the guest's next 2 MiB found the pool empty and had memory given back (printed the
1st, 2nd, 4th, 8th ... time).

`EDEN_NX2_TC_SHAPES=1` (with `EDEN_NX2_TC_LOG=1`) lists the shapes of the images the texture cache
made in each 600 frames. `EDEN_NX2_MEM_TRACE=1` now also prints the allocator's pressure counters,
the collector's margins, and what the cache holds unused.
