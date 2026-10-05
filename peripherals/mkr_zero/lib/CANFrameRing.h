#ifndef CAN_FRAME_RING_H
#define CAN_FRAME_RING_H 1

/**
 * @file CANFrameRing.h
 * @brief Single-producer / single-consumer RAM ring of raw CAN frames.
 *
 * The producer is the drain timer's interrupt (@c canDrainIsr() in
 * CANSniffFunctions.cpp), which empties the MCP2515's two receive buffers every
 * @c CAN_DRAIN_PERIOD_US. The consumer is loop(), which streams the frames to
 * the Orin over USB and, in SNIFF only, decodes them. The ring is what lets the
 * two run at different rates: the drain has to keep up with the BUS (a two-deep
 * hardware buffer at 500 kbps gives it a few hundred microseconds), while loop()
 * only has to keep up ON AVERAGE, because one pass can hold the CPU for up to
 * 750 ms during GNSS bring-up.
 *
 * ── WHY THIS IS SAFE WITHOUT A LOCK ──────────────────────────────────────────
 * One writer per index. The ISR alone writes @c head (and the slot it points
 * at, and @c drops); loop() alone writes @c tail. Each side only READS the
 * other's index, and a 16-bit aligned load or store is a single instruction on
 * the Cortex-M0+, so neither can ever see a half-written index.
 *
 * What remains is ORDER, and on a single core that is purely a compiler
 * problem: the ISR runs on the same core as loop(), and the M0+ executes and
 * retires memory accesses in program order, so the only thing that can move a
 * slot write past the index store that publishes it is the optimiser. A
 * compiler barrier (@c CAN_RING_BARRIER) at each hand-over is therefore the
 * whole requirement — no DMB, no critical section, no masked interrupts:
 *
 *   producer: write slot  -> barrier -> store head    (publish)
 *   consumer: load head   -> barrier -> read slot     (acquire)
 *   consumer: read slot   -> barrier -> store tail    (release the slot)
 *
 * The indices run free (they are never wrapped to the slot count), so
 * @c head - @c tail is the fill level directly, a full ring and an empty one
 * are told apart without sacrificing a slot, and the slot is the low bits.
 *
 * ── FULL MEANS THE NEWEST FRAME IS LOST ──────────────────────────────────────
 * The producer cannot take room from the consumer — moving @c tail from the ISR
 * would make it a second writer of that index — so a frame arriving at a full
 * ring is refused and counted in @c drops. That is the @c ringdrop figure on
 * the Orin's FS line.
 *
 * Header-only on purpose: the host tests exercise exactly this code, with no
 * MCP2515 and no ISR in the way.
 */

#include <stdint.h>
#include <string.h>
#include <atomic>   // std::atomic_signal_fence only: a compiler barrier, no code

/**
 * Ring capacity, in frames. 1024 x 16 bytes = 16 KB of the SAMD21's 32 KB.
 *
 * Sized for the worst single loop() pass, not the average one. A GNSS bring-up
 * stage can hold the CPU for up to 750 ms (see gpsInitTick), and the measured
 * bus delivers ~1160 frames/s with all IDs accepted (2026-10-03 baseline), so
 * that pass alone brings ~870 frames: 1024 holds ~880 ms of the measured bus.
 *
 * Bigger is not free. The compile left 25.4 KB for heap and stack before this
 * ring; with it about 9 KB remain, against ~1 KB of heap (the GNSS buffers,
 * claimed once in setup()) and a stack that peaks well under 2 KB — the
 * heaviest frames are tickCommMaster() with sendFrame()'s 261-byte wire buffer
 * underneath it. Doubling the ring would leave the stack less room than that.
 *
 * A power of two so the slot is the index's low bits, and at most 32768 so the
 * free-running uint16 difference can never alias a full ring as an empty one.
 */
#define CAN_RING_SLOTS 1024u

static_assert((CAN_RING_SLOTS & (CAN_RING_SLOTS - 1u)) == 0u, "CAN_RING_SLOTS must be a power of two");
static_assert(CAN_RING_SLOTS <= 32768u, "uint16_t free-running indices need CAN_RING_SLOTS <= 32768");

/// Compiler barrier, and nothing more — see the file header for why that is
/// enough on this single-core part. @c std::atomic_signal_fence rather than an
/// asm statement so the same header also builds under MSVC (RunTests.cmd).
#define CAN_RING_BARRIER() std::atomic_signal_fence(std::memory_order_seq_cst)

// ─── one frame ────────────────────────────────────────────────────────────────

#define CAN_RAW_ID_MASK 0x1FFFFFFFu  ///< 11- or 29-bit identifier.
#define CAN_RAW_EXT     0x80000000u  ///< Extended (29-bit) identifier.
#define CAN_RAW_RTR     0x40000000u  ///< Remote frame: no data; the DLC is the length requested.
/**
 * DLC is 8. Clear means the DLC (0-7) is in @c data[7].
 *
 * This is what keeps a frame at 16 bytes rather than 20, which is 4 KB of ring
 * at 1024 slots. The identifier needs 29 bits, the two flags above take two of
 * the remaining three, and a DLC needs four — but a frame with fewer than eight
 * data bytes never uses @c data[7], and a remote frame uses none, so the length
 * only needs a home of its own when it is exactly 8. Always read it through
 * @c canRawDlc(), never from @c data[7] directly.
 */
#define CAN_RAW_DLC8    0x20000000u

/** @brief One received frame, exactly as the controller delivered it. */
struct CanRawFrame {
    uint32_t us;      ///< micros() when it was read out of the MCP2515 (wraps every ~71.6 min).
    uint32_t idf;     ///< Identifier in the low 29 bits, plus @c CAN_RAW_EXT / _RTR / _DLC8.
    uint8_t  data[8]; ///< Payload; bytes past the DLC are zero, except as @c CAN_RAW_DLC8 says.
};
static_assert(sizeof(CanRawFrame) == 16u, "CanRawFrame is sized into the RAM budget at 16 bytes");

inline uint32_t canRawId (const CanRawFrame &f) { return f.idf & CAN_RAW_ID_MASK; }
inline bool     canRawExt(const CanRawFrame &f) { return (f.idf & CAN_RAW_EXT) != 0u; }
inline bool     canRawRtr(const CanRawFrame &f) { return (f.idf & CAN_RAW_RTR) != 0u; }
inline uint8_t  canRawDlc(const CanRawFrame &f)
{
    return (f.idf & CAN_RAW_DLC8) ? 8u : (uint8_t)(f.data[7] & 0x0Fu);
}

/**
 * @brief Fills @p f, packing the DLC as @c CAN_RAW_DLC8 describes.
 *
 * @param dlc   Clamped to 8: ISO 11898-1 says any DLC above 8 still means eight
 *              data bytes, and the controller stores no more than that.
 * @param data  @p dlc payload bytes; ignored for a remote frame, which has none.
 */
inline void canRawPack(CanRawFrame &f, uint32_t us, uint32_t id, bool ext, bool rtr,
                       uint8_t dlc, const uint8_t *data)
{
    if (dlc > 8u) dlc = 8u;
    f.us  = us;
    f.idf = (id & CAN_RAW_ID_MASK) | (ext ? CAN_RAW_EXT : 0u) | (rtr ? CAN_RAW_RTR : 0u) |
            ((dlc == 8u) ? CAN_RAW_DLC8 : 0u);
    for (uint8_t i = 0; i < 8u; ++i) f.data[i] = (!rtr && i < dlc) ? data[i] : 0u;
    if (dlc < 8u) f.data[7] = dlc;
}

// ─── the ring ─────────────────────────────────────────────────────────────────

/** @brief The ring. Zero-initialised storage is a valid empty ring. */
struct CanFrameRing {
    volatile uint16_t head;    ///< Next slot the producer fills. Written ONLY by the producer.
    volatile uint16_t tail;    ///< Oldest slot not yet released. Written ONLY by the consumer.
    volatile uint32_t drops;   ///< Frames refused because the ring was full. Producer-owned, wraps.
    CanRawFrame       slot[CAN_RING_SLOTS];
};

/** @brief Empties the ring. Only while neither side is running. */
inline void canRingReset(CanFrameRing &r)
{
    r.head  = 0u;
    r.tail  = 0u;
    r.drops = 0u;
}

/** @brief Frames published and not yet released. Either side may ask. */
inline uint16_t canRingCount(const CanFrameRing &r)
{
    return (uint16_t)(r.head - r.tail);
}

/**
 * @brief PRODUCER ONLY. Appends @p f, or counts it as dropped when full.
 * @return false when the ring was full and the frame is lost.
 */
inline bool canRingPush(CanFrameRing &r, const CanRawFrame &f)
{
    const uint16_t h = r.head;
    if ((uint16_t)(h - r.tail) >= CAN_RING_SLOTS) {
        r.drops = r.drops + 1u;
        return false;
    }
    r.slot[h & (CAN_RING_SLOTS - 1u)] = f;
    CAN_RING_BARRIER();             // the slot is complete before head says it exists
    r.head = (uint16_t)(h + 1u);
    return true;
}

/**
 * @brief CONSUMER ONLY. Copies the frame at free-running index @p idx.
 *
 * @p idx must lie in [tail, head]; at @c head there is nothing yet and this
 * returns false. Taking an index rather than always the tail is what lets the
 * SNIFF decoder run AHEAD of the USB stream — both are loop(), so both are the
 * one consumer — while the slot is released only once the slower has finished.
 */
inline bool canRingPeek(const CanFrameRing &r, uint16_t idx, CanRawFrame &out)
{
    const uint16_t h = r.head;
    if (idx == h) return false;
    CAN_RING_BARRIER();             // head is read before the slot it vouches for
    out = r.slot[idx & (CAN_RING_SLOTS - 1u)];
    return true;
}

/**
 * @brief CONSUMER ONLY. Releases the @p n oldest frames for the producer to reuse.
 *
 * The caller has finished reading them, and must not release more than
 * @c canRingCount() says are there.
 */
inline void canRingRelease(CanFrameRing &r, uint16_t n)
{
    CAN_RING_BARRIER();             // every read of those slots is done before they are given back
    r.tail = (uint16_t)(r.tail + n);
}

#endif // CAN_FRAME_RING_H
