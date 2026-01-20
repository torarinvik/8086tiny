# Headless DOS Emulator + Coverage-Guided Explorer (Beginner-Friendly Spec)

This document describes a practical system for **automatically exploring a DOS program** (e.g., a game EXE/COM) to uncover as much executed code as possible—especially “mystery code” whose control flow is hard to recover statically.

The approach is intentionally simple first (works shockingly well), then extends to more powerful features like checkpointing and targeted exploration. The system is designed to be built by **modifying a small emulator** (e.g., a tiny 8086 emulator) rather than writing everything from scratch.

---

## 0) Goal and Non-Goals

### Goal

Build a tool that:

- Runs a DOS program **headlessly** (no graphics/sound required).
- Feeds it **synthetic keyboard input**.
- Tracks which code addresses are executed (**coverage**).
- Automatically generates new inputs that cause the program to execute **new code paths**.
- Saves the best input sequences and (optionally) machine snapshots so exploration goes deeper over time.

### Non-goals (for the first version)

- Perfect DOS compatibility (we can start with “enough to run *this* target”).
- Perfect semantic equivalence proofs (that’s a later layer).
- Full soundness (“all possible code”). Instead, we aim for **maximum observed coverage** with reproducibility.

---

## 1) System Overview (High Level)

The system is a loop:

1. **Start from a known initial state** (fresh boot / program start / or a checkpoint).
2. **Choose an input sequence** to try (keystrokes + timings).
3. Run the emulator for a **fixed instruction budget** or until a stop condition.
4. Collect **coverage**: which addresses were executed.
5. If the input yields **new coverage**, keep it in a **corpus**.
6. Mutate corpus inputs to generate new candidates.
7. Repeat indefinitely, steadily pushing into deeper program states.

This is the exact idea behind modern fuzzers—applied to a game-like program controlled through a keyboard.

---

## 2) Key Concepts

### 2.1 Determinism

The whole system depends on deterministic runs:

- Same initial state + same input sequence ⇒ same execution.
- Timer interrupts must be scheduled deterministically.
- Randomness should be controllable (fixed seed or captured behavior).

Determinism is critical because:

- It makes results reproducible.
- It allows you to debug divergences.
- It ensures your “best input” stays best.

### 2.2 Coverage

Coverage is the score function: “did we learn something new?”

The simplest coverage is:

- Record the executed instruction address at each step.

Common ways to store coverage:

- **Bitmap** indexed by address (fast, fixed size).
- **Hashed bitmap** (AFL-style) if address space is large.
- **Set of addresses** (simple but slower).

For 16-bit DOS:

- A natural key is the **physical address**: `phys = (CS << 4) + IP` (20-bit, 1MB address space).

### 2.3 Corpus

The corpus is a set of input sequences that produced interesting new behavior (new coverage, deep progress, etc.).

Corpus items are used as “seeds” for new tests via mutation.

### 2.4 Mutation

Mutation generates new inputs from old successful ones.

Example mutations:

- Flip a key: change ‘Left’ to ‘Right’.
- Insert a key.
- Delete a key.
- Swap two key events.
- Change timing between keys.
- Extend a sequence with extra keys.

Mutation is the engine that walks the program’s state space.

---

## 3) Emulator Requirements (Minimal)

You do not need full DOSBox for the first version. You need:

### 3.1 CPU core

- 8086/80186/286-ish core (depending on the target)
- Correct registers + flags + memory semantics
- Correct instruction decoding/execution for the target’s instruction set

### 3.2 Memory model

- Flat 1MB memory array
- Segment:offset translation: `phys = (seg << 4) + off` masked to 20 bits

### 3.3 Keyboard input interface

Implement BIOS keyboard interrupt behavior (common):

- INT 16h AH=00h (read key)
- INT 16h AH=01h (check key)
  Or emulate keyboard port behavior if needed.
  For the first iteration, implementing INT 16h is usually simplest and sufficient.

### 3.4 DOS/BIOS interrupts (subset)

You need only the subset your program uses. Frequently:

- INT 21h for DOS services (exit, file I/O, console I/O)
- INT 10h for video (can be stubbed if headless, but must not break logic)
- Timer tick (INT 08h/1Ch) if the program depends on it

A practical tactic:

- Start by logging which INT functions are called.
- Implement only those as the program demands.

---

## 4) The “Simple Version” (Step 6) — Minimal Fuzz Explorer

### 4.1 Run parameters

Choose:

- `MAX_INSTRUCTIONS`: how many instructions per test run (e.g., 200,000).
- `MAX_WALLTIME`: optional real-time cap per run (should rarely be needed if deterministic).
- `KEYSEQ_MAXLEN`: maximum input length (e.g., 1–200 key events).

### 4.2 Coverage bitmap

A simple and extremely effective approach:

- Allocate a byte array for 1MB physical memory:
  - `coverage[1<<20]` (1,048,576 bytes)
- Each instruction executed:
  - `coverage[phys] = 1`

You also maintain a global “ever covered” bitmap:

- `global_coverage[1<<20]`

At the end of a run:

- Compare run coverage vs global coverage.
- Count how many new bits are set (the “coverage gain”).

**Coverage gain** is the reward.

#### Performance note

Updating coverage per instruction is cheap (a few operations). Avoid printing/logging per instruction.

### 4.3 Input representation

Represent input as a list of events:

```text
Event:
  kind: KeyDown | KeyUp | KeyPress
  scancode: u8
  ascii: u8 (optional, depending on how INT 16h returns keys)
  delay_ticks: u16 (how long to wait after this event)
```

Simplify initially:

- Use KeyPress only (a press event produces a key code in the BIOS queue).
- Use a small delay after each press (e.g., 0–3 ticks).

### 4.4 Feeding keys to the emulator

Provide an internal queue:

- The fuzzer preloads a queue with events.
- INT 16h reads from this queue.

If the program calls “wait for key” and the queue is empty:

- You can return “no key” for AH=01h (check)
- For AH=00h (blocking read), you have options:
  1) Stall until a key arrives (not good for bounded runs)
  2) Inject a default “no-op” key
  3) End the run early
     For fuzzing, option (3) is often best: if it blocks, the run ends.

### 4.5 The main exploration loop (pseudo)

```
global_coverage = all zeros
corpus = []
best_inputs = []

seed_inputs = [empty, [Enter], [Space], [Esc], ...]  // a few sensible seeds

for input in seed_inputs:
    run = execute(input)
    gain = compute_new_coverage(run.coverage, global_coverage)
    if gain > 0:
        add_to_corpus(input, run, gain)

repeat forever:
    parent = choose_from_corpus(corpus)           // weighted by gain or novelty
    child  = mutate(parent.input)
    run    = execute(child)
    gain   = compute_new_coverage(run.coverage, global_coverage)

    if gain > 0:
        add_to_corpus(child, run, gain)
```

### 4.6 Corpus management

Store for each corpus item:

- input sequence
- total coverage gain
- optional “depth” metrics (e.g., maximum CS:IP reached, or time survived)
- optional “signature” of interesting behavior (hash of coverage or state)

Keep corpus from growing forever:

- Cap size (e.g., 5k items).
- Prefer items with unique coverage signatures.
- Evict items that become redundant.

---

## 5) “Maximize info about THIS region” (Targeted Exploration)

Often you care about a particular address range:

- e.g., `[mystery_start, mystery_end)`

Add a second reward component:

- `gain_in_region = newly covered addresses within target range`
- Boost tests that cover new addresses in that region.

Total reward:

- `reward = a * global_gain + b * region_gain + c * rare_edges_gain`

In practice:

- Use `b >> a` if your goal is to “magnetize” exploration into the mystery code.

---

## 6) Checkpoints (Huge Upgrade for Games)

Random fuzzing from boot gets stuck in menus. Checkpoints fix this.

### 6.1 What is a checkpoint?

A checkpoint is a serialized machine state:

- CPU regs + flags
- full RAM (1MB) OR copy-on-write pages
- interrupt vector table and BIOS/DOS state (if modeled)
- emulator “device” state (keyboard queue, timer counters, etc.)
- any disk/file-system state if the program reads/writes files (optional first)

### 6.2 When to snapshot

Good snapshot times:

- After leaving menu / starting a new game
- After level load completes
- After a cutscene ends
- After a “state gate” (e.g., game_mode changes)

You can detect these either:

- manually (first version)
- automatically by heuristics (later): e.g. large coverage jump, or specific call patterns

### 6.3 How checkpoints help exploration

With checkpoints:

- You fuzz “from level 3 start” directly instead of replaying the whole game.
- You can branch exploration from a deep state.
- You can do hierarchical search: find level 2, snapshot; from there find level 3, snapshot; etc.

### 6.4 Minimal checkpoint implementation (easy but big)

- Save entire 1MB RAM + regs to a file.
- Restore by memcpy.

This is heavy but simple. Optimize later with paging/diffs.

---

## 7) Efficient Coverage Tracking

### 7.1 Basic block coverage (optional improvement)

Instruction-level coverage is fine. But basic block edge coverage is better for guiding exploration.

Basic block edges:

- Track `(prev_pc, curr_pc)` transitions.
- Hash them into a bitmap:
  - `idx = hash(prev_pc ^ (curr_pc << 1)) & (MAP_SIZE-1)`
    This is the classic AFL trick.

Why better?

- It distinguishes paths, not just visited addresses.
- It rewards different control flow through the same code.

### 7.2 Region-only coverage

If your target is a specific region, record only those addresses to reduce overhead.

---

## 8) Avoiding “Boring Loops” and “Cheating Paths”

Programs can get stuck:

- waiting for input
- stuck in a loop that produces coverage once and then repeats

Add stop conditions:

- If no new coverage in last N instructions → terminate run
- If PC repeats too often → terminate run
- If the program calls INT 21h exit → stop

Also consider “time outside region” penalty in targeted mode.

---

## 9) Input Mutation Details (Practical Recipes)

Start simple. Your mutation operators are your entire “search intelligence”.

### 9.1 Keyboard alphabet

Define a small set of keys:

- arrows, enter, space, esc, ctrl, alt, letters/numbers if needed

Smaller alphabet = faster discovery (less randomness).

### 9.2 Mutation operators

Given a sequence `S`:

- **Flip**: replace one event with a random key
- **Insert**: insert random key at random position
- **Delete**: remove a random event
- **Duplicate**: repeat a small subsequence
- **Splice**: combine prefix of A with suffix of B (two corpus items)
- **Delay tweak**: change delay ticks around an event

### 9.3 Mutation scheduling

Early exploration benefits from more randomness (big mutations). Later, do small mutations to refine deep paths.

A practical schedule:

- 70% small mutations
- 25% medium
- 5% large

---

## 10) Reproducibility and Logging

Every run should be reproducible:

- Save random seed used for mutation.
- Save the exact input sequence and timing.
- Save the checkpoint identifier used.

For debugging:

- On divergence or crash, save:
  - last N executed PCs
  - minimal trace around the event
  - snapshot

---

## 11) Data Outputs You Want

Even without decompilation, this explorer produces valuable artifacts:

1) **Coverage maps**:
   - heat map of executed addresses
   - “mystery region” uncovered percent

2) **Discovered jump targets**:
   - for indirect jumps/calls, log the resolved target PCs

3) **State gate detection hints**:
   - memory addresses that correlate with “level number” or “game_mode” changes
   - useful for later state injection

4) **Trace snippets**:
   - short PC sequences leading into rare code

---

## 12) How This Helps Reverse Engineering

Once you can reliably reach code:

- You can mark bytes as “definitely code” vs “never observed”
- You can identify function boundaries by observing call/ret structure
- You can discover jump tables by logging indirect targets
- You can build signatures of routines by their observed behavior
- You can build an annotation layer (@origin or semantic lifts) with confidence

This is the foundation under a proof-oriented or equivalence-oriented decompilation pipeline.

---

## 13) Suggested Build Order (Incremental, Lowest Pain)

### Phase 1 (Very Fast)

- Headless emulator runs your program to completion or until instruction budget.
- Keyboard queue injection (INT 16h stubs).
- Instruction coverage bitmap.
- Corpus + mutation loop.

### Phase 2 (Deeper)

- Stop-on-stall heuristics (no new coverage → stop).
- Better coverage metric (edge coverage).
- Save best seeds + run statistics.

### Phase 3 (Level 5 problem)

- Checkpoint/restore.
- Checkpoint selection strategy (fuzz from deeper checkpoints more often).
- Targeted reward for mystery region.

### Phase 4 (Smarter)

- Log indirect jump/call targets.
- Detect state gates and build “milestone” snapshots automatically.

---

## 14) Common Pitfalls

1) **Non-deterministic timer behavior**
   - Fix timer scheduling; run ticks only when you choose.

2) **Blocking input calls**
   - Terminate run if the program blocks with no keys.

3) **Too-large input alphabet**
   - Keep it small at first. Expand only if needed.

4) **Corpus bloat**
   - Use deduplication by coverage signature. Evict redundant seeds.

5) **Emulator not modeling required INT calls**
   - Add logging; implement only what your program uses.

---

## 15) Appendix: Concrete “Coverage Gain” Computation

Given:

- `run_cov[ ]` bitmap for this run
- `global_cov[ ]` bitmap for all runs

Compute:

- `new = run_cov & ~global_cov`
- `gain = popcount(new)` (count bits or bytes != 0)
- Then `global_cov |= run_cov`

Fast byte-level approximation (good enough):

- Use bytes rather than bits:
  - `gain += (run_cov[i] && !global_cov[i])`

---

## 16) Appendix: What “Headless” Means Here

You don’t need to render graphics or play sound.
But your emulator must still:

- keep video memory in RAM (if the game writes it)
- respond to key BIOS/DOS calls
- keep timing deterministic enough that logic doesn’t break

The program doesn’t care if pixels are shown; it cares that its memory and interrupts behave consistently.

---

## 17) What You Get If This Works

When the system “clicks,” you’ll have:

- A growing set of inputs that reliably reach deeper states
- A coverage map showing exactly which parts of the binary execute
- A reproducible harness to reach “mystery code”
- The perfect foundation for semantic lifting, proof-carrying origins, and equivalence testing

---

**End of spec.**