/*
 * Copyright (c) 2026 Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 *  ======== stdlib_override.c ========
 *  Efficient memcpy and memset implementations to replace newlib-nano functions.
 *  The functions provided below allows newlib-nano to still be used for a small
 *  general footprint for the standard library, but the two functions below
 *  provide a ~5x speed increase. For these two functions, we prioritize speed
 *  over code footprint.
 *
 *  The functions are provided as __wrap_memcpy and __wrap_memset for use with
 *  the GNU linker --wrap flag so it intercepts calls to the standard library
 *  functions memcpy without requiring a direct symbol replacement.
 *
 *  The implementations align the destination pointer to a 4-byte boundary
 *  before entering a word-at-a-time copy/fill loop, then handle any remaining
 *  tail bytes.  The inner loop is unrolled 8 times (32 bytes per iteration)
 *  to keep the pipeline busy on Cortex-M3/M4/M33 cores.  The byte-by-byte
 *  alignment prologue ensures correctness on Cortex-M0/M0+, which do not
 *  support hardware unaligned word accesses.
 */

#include <stddef.h>
#include <stdint.h>

/*
 *  ======== __wrap_memcpy ========
 *  Link with --wrap=memcpy to redirect all memcpy calls here.
 *
 *  Strategy:
 *    1. Copy individual bytes until dest is 4-byte aligned.
 *    2. If src is also 4-byte aligned, copy 32 bytes per iteration,
 *       then 4 bytes per iteration for the remainder.
 *    3. Copy any trailing bytes one at a time.
 *
 *  When src and dest have different alignments the function falls through
 *  to the byte-copy tail, which is safe on all Cortex-M variants.
 *
 * "unroll-loops": allows the compiler to unroll the word-fill loop, reducing
 * branch overhead per byte written.
 * "no-tree-loop-distribute-patterns": suppresses the GCC pass that recognises
 * byte-fill loops and replaces them with a call to memset.  Without it the
 * compiler would emit a call to memset inside __wrap_memset, which the linker
 * wraps back to this function, causing infinite recursion.
 */
__attribute__((weak, optimize("unroll-loops", "no-tree-loop-distribute-patterns"))) void *__wrap_memcpy(
    void *restrict dest,
    const void *restrict src,
    size_t n)
{
    uint8_t *destination  = (uint8_t *)dest;
    const uint8_t *source = (const uint8_t *)src;

    /* Handle small sizes or non-word alignment via byte copy.
     * Performing unaligned word access might be possible on some architectures,
     * but the implementation is kept aligned to be valid on all devices.
     */
    if (n < 4U || ((uintptr_t)destination & 3U) != ((uintptr_t)source & 3U))
    {
        while (n--)
        {
            *destination++ = *source++;
        }
        return dest;
    }

    /* Align destination and source pointers to 32-bit word boundary.
     * At this point in the code, the source and destination are guaranteed
     * to be aligned to each other, and n >= 4.
     */
    while ((uintptr_t)destination & 3U)
    {
        *destination++ = *source++;
        n--;
    }

    /* Bulk 32-bit word copies. The loop below will perform a series of word
     * load/store operations to reduce the overhead of increasing the load/
     * store addresses (8 immediate offsets should be used instead), as well as
     * loop bounds checking.
     */
    uint32_t *d32       = (uint32_t *)destination;
    const uint32_t *s32 = (const uint32_t *)source;

    /* 8 words / 32 bytes are copied in each iteration. */
    size_t iterations = n / 32U;

    while (iterations--)
    {
        d32[0] = s32[0];
        d32[1] = s32[1];
        d32[2] = s32[2];
        d32[3] = s32[3];
        d32[4] = s32[4];
        d32[5] = s32[5];
        d32[6] = s32[6];
        d32[7] = s32[7];

        /* Increment destination and source pointers by how many words were copied. */
        d32 += 8U;
        s32 += 8U;
    }

    /* Update remaining bytes to copy. */
    n %= 32U;

    /* Copy remaining words. */
    iterations = n / 4U;

    while (iterations--)
    {
        *d32++ = *s32++;
    }

    /* Update remaining bytes to copy. */
    n %= 4U;

    destination = (uint8_t *)d32;
    source      = (const uint8_t *)s32;

    /* Copy remaining bytes. */
    while (n--)
    {
        *destination++ = *source++;
    }

    return dest;
}

/*
 *  ======== __wrap_memset ========
 *  Link with --wrap=memset to redirect all memset calls here.
 *
 *  Strategy:
 *    1. Fill individual bytes until dest is 4-byte aligned.
 *    2. Replicate the fill byte into all four lanes of a 32-bit word and
 *       store 32 bytes per iteration.
 *    3. Fill any remaining whole words.
 *    4. Fill any trailing bytes one at a time.
 *
 * "unroll-loops": allows the compiler to unroll the word-fill loop, reducing
 * branch overhead per byte written.
 * "no-tree-loop-distribute-patterns": suppresses the GCC pass that recognises
 * byte-fill loops and replaces them with a call to memset.  Without it the
 * compiler would emit a call to memset inside __wrap_memset, which the linker
 * wraps back to this function, causing infinite recursion.
 */
__attribute__((weak, optimize("unroll-loops", "no-tree-loop-distribute-patterns"))) void *__wrap_memset(void *str,
                                                                                                        int c,
                                                                                                        size_t n)
{
    uint8_t *destination = (uint8_t *)str;
    const uint8_t value  = (uint8_t)c;

    /* Byte-fill until destination pointer reaches a 4-byte boundary. */
    while ((n > 0U) && ((uintptr_t)destination & 3U))
    {
        *destination++ = value;
        n--;
    }

    /* Word-fill the bulk of the buffer. */
    if (n >= 4U)
    {
        const uint32_t word = (uint32_t)value | ((uint32_t)value << 8U) | ((uint32_t)value << 16U) |
                              ((uint32_t)value << 24U);
        uint32_t *d32 = (uint32_t *)destination;

        /* 8 words / 32 bytes are set in each iteration. */
        size_t iterations = n / 32U;

        while (iterations--)
        {
            d32[0] = word;
            d32[1] = word;
            d32[2] = word;
            d32[3] = word;
            d32[4] = word;
            d32[5] = word;
            d32[6] = word;
            d32[7] = word;

            /* Increment word-aligned pointer by how many words were written. */
            d32 += 8U;
        }

        /* Update remaining bytes to copy. */
        n %= 32U;

        /* Write remaining words */
        iterations = n / 4U;

        while (iterations--)
        {
            *d32++ = word;
        }

        /* Update remaining bytes to copy. */
        n %= 4U;

        destination = (uint8_t *)d32;
    }

    /* Set trailing bytes. */
    while (n--)
    {
        *destination++ = value;
    }

    return str;
}
