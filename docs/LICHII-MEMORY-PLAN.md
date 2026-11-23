# More of PS5_Lichii's memory and speed work, on top of the swap store

Status, 2026-11-23: being implemented in this branch, stacked on `feature/safe-swap` (PR #135),
which brought PS5_Lichii's swap store. **None of it has run on the console yet**: by decision the
console runs come once, at the end (two of Mario Kart World, one of Donkey Kong Bananza, one of
Ocarina of Time, against the numbers PR #135 has). Each step has a switch that restores what was
there before, so that a result can be traced to its step afterwards.

| Step | State | Switch back |
|---|---|---|
| 1a. The transcode's decode image, a few layers at a time | written (`TranscodeAstcToBcn`), syntax and arithmetic checked | `EDEN_PS5_TRANSCODE_LAYER_BATCH=0` |
| 1b. BC7 texture store (emulator side and tool) | written, `docs/TEXTURE-STORE.md`; no store exists for any game yet, so it is idle until the tool has been run over one | `EDEN_NX2_TEXTURE_STORE=0` |
| 2, 3. Slots nothing touches in 64 KiB units: zstd in memory, the swap store, one unit back at a touch | written, host-checked (`eden_swap_check` steps 64 to 67), compiles for the console | `EDEN_CRAM_UNITS=0` |
| 4. JIT code caches backed as written | written, host-checked (`CheckCodeRanges`), `docs/JIT-CODE-CACHE-LAZY.md` | `EDEN_JIT_LAZY_CACHE=0` |
| 5. What the pool has free, counted in 2 MiB pieces for the caches | written | `EDEN_POOL_FREE_PIECES=0` |
| 6. The fastmem window by default | not done, see below | |
| 7. Our own memcmp | ported, host-checked (`tools/check-memcmp.py`) | build-time only |

Source: PS5_Lichii (mihawk, `codeberg.org/mihawk/PS5_Lichii`, commit `5a424f7`), its
`docs/ps5/levers.md` and the topic files beside it. What Lichii reports there for the PS5 Pro is
23.5 to 25 fps in Ocarina of Time's field and about 27 ms a frame in a Mario Kart World race,
with 1.5 to 2 GB of the pool free. The swap store is one of eleven memory levers there, and not
the largest.

## What EglantinePS5 has already (first look, each still to be checked against Lichii's code)

| Lichii's lever | Here |
|---|---|
| Inline page-table lookup, exclusive loads and stores inline | `docs/JIT-DIRECT-PAGE-TABLE.md`, `docs/JIT-FAST-PATHS.md` |
| Read traps on a bit of the page entry | in the core (`device_memory_manager`, `memory.cpp`) |
| The fastmem window | `docs/FASTMEM64-WINDOW.md`: there, but opt-in (`EDEN_FASTMEM64=1`) |
| Async shaders, bounded pipeline waits | a setting in the launcher; the core has the waits |
| ASTC to BCn on the GPU | `TranscodeAstcToBcn` in `vk_texture_cache.cpp` |
| Swap store | PR #135 |

## What it lacks, in the order proposed

1. **The texture array's 560 MiB, and the BC7 texture store.** Looked at on 2026-11-23:
   - The 560 MiB image is not what the game's texture ends as. It is the RGBA8 image the GPU
     transcode decodes into (`TranscodeAstcToBcn`, `vk_texture_cache.cpp`): all 46 layers and 11
     levels at once, alive until the GPU has encoded them to BC3 (140 MiB), with a 128 MiB
     staging buffer for the result. Decoding and encoding a few layers at a time keeps that
     under the 64 MiB the transcode already holds itself to for smaller images. That is the
     first change of this step, and it needs nothing from outside.
   - Lichii's BC7 store is made ahead of time, off the console: a tool (`ps5/tools/texture-store`)
     reads the unpacked game, decodes each ASTC texture and encodes it with bc7e at "very slow",
     one file per texture named by the hash of its guest bytes. It was made for one title
     (11,739 textures). The emulator's side (look a texture up by hash, upload its BC7 levels)
     can be ported; a store for a game exists only once someone has run the tool over that game.
2. **zstd on each 64 KiB unit as the memory step**, inside the 2 MiB slots as they are. Measured
   on 1 GiB of what Mario Kart World had the store hold: 36% of the units would stay in memory
   at half their size, for about a fifth fewer bytes written.
3. **Guest memory backed on touch in 64 KiB units**, zero units given back. Lichii's base. Here
   a slot is 2 MiB and a touch brings all of it back: this is a rewrite of the guest memory
   service (`src/guest_memory_service.inc`, 3,600 lines), with the fault handler in it. With it,
   Lichii's packing pass (`EDEN_SWAP_PASS=2`), which ended Mario Kart World sessions early on
   2 MiB slots, can be measured again.
4. **JIT code caches backed as code is written**, and their size. Here 160 MiB for core 0 down
   to 16 MiB for the last cores of a Switch 2 title, committed at once; Lichii reserves 320 MiB
   a core and backs 1.0 to 1.2 GB of 1.9.
5. **The driver's memory counted once.** Lichii's driver reported twice what it held and its
   texture cache collected as if the heap were full (PS5_Mesa `75238ae`). To check: whether the
   driver build used here has the same fault.
6. **The fastmem window by default**, once it is qualified for the games tested here.
7. **Our own memcmp** for the executable (the system's is 5 to 6 times slower there).

Each step is measured on the console before the next, with Mario Kart World (30 minutes and an
hour) and Ocarina of Time, as PR #135 was. Steps 1 and 3 are the large ones.

## Rules for the console tests

The launcher's "Update available" dialog takes the focus about 20 s after it opens. No test
presses Cross in the launcher without a screenshot first.

## What changed while implementing

- **Steps 2 and 3 are one piece of work, and not a rewrite.** Memory in use stays in 2 MiB slots
  (the driver takes the pool in 2 MiB pieces, and Lichii lists "2 MiB slab allocation" among what
  its own design still lacks). It is the slot nothing touches that goes in 64 KiB units; a touch
  brings back one unit, from a slab of the pool cut in 32. See "Units" in
  `src/guest_memory_service.inc`.
- **Step 5 turned out differently.** Our driver has Lichii's double count, but nothing here reads
  that number (`docs/LICHII-SMALL-PORTS.md`). What the port did get wrong is the other half of
  that lever: the caches were told the pool's largest free block, where the driver can take what
  is free in 2 MiB pieces.
- **Step 4's premise was wrong.** The code caches already took their memory as the JIT reached
  it, in 16 MiB steps (192 MiB at boot for a Switch 2 title's six cores). The step is now one
  2 MiB piece (120 MiB at boot), the executable views lie in one span of just under 2 GiB, and a
  Switch 2 title's cores get Lichii's 320 MiB each (`docs/JIT-CODE-CACHE-LAZY.md`).
- **Step 1b needs a store.** The emulator side looks textures up by the hash of their guest bytes
  in `/data/eglantineps5/texture-store/<title id>/`; the files come from
  `tools/texture-store`, run on a PC over the unpacked game. Until someone has done that for a
  game, the step changes nothing for it.
- **Step 6 is not done.** `docs/FASTMEM64-WINDOW.md` measured the window at the defaults' frame
  rate on Mario Kart World, not above it, and the window shows whole 2 MiB blocks: with it on, a
  touch has to bring a slot in units back whole, which is what step 3 is there to avoid. It
  stays opt-in (`EDEN_FASTMEM64=1`).
