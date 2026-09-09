/*
 * Copyright (c) 2025, Texas Instruments Incorporated
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
/*
 *  ======== mbedtls_config_bl3.h ========
 *
 *  Minimal mbedTLS configuration for BL3 image authentication.
 *  Enables only what bootutil needs: SHA-256 and ECDSA P-256.
 *  No entropy source, no TLS, no PSA — SW crypto only.
 */

#ifndef MBEDTLS_CONFIG_BL3_H
#define MBEDTLS_CONFIG_BL3_H

/* --- Math / bignum (required by ECP) --- */
#define MBEDTLS_BIGNUM_C

/* --- ECP: P-256 curve only --- */
#define MBEDTLS_ECP_C
#define MBEDTLS_ECP_DP_SECP256R1_ENABLED
#define MBEDTLS_ECP_NIST_OPTIM

/* --- ECDSA --- */
#define MBEDTLS_ECDSA_C
#define MBEDTLS_ASN1_PARSE_C
#define MBEDTLS_ASN1_WRITE_C

/* --- SHA-256 --- */
#define MBEDTLS_SHA256_C
/* Disable SHA-512 to save code size */
#undef MBEDTLS_SHA512_C

/* --- Platform --- */
#define MBEDTLS_PLATFORM_C
#define MBEDTLS_PLATFORM_MEMORY
/* No file I/O, no time, no entropy */
#undef MBEDTLS_TIMING_C
#undef MBEDTLS_NET_C
#undef MBEDTLS_ENTROPY_C
#undef MBEDTLS_CTR_DRBG_C
#undef MBEDTLS_HMAC_DRBG_C

/* --- No TLS, no X.509, no PSA --- */
#undef MBEDTLS_SSL_TLS_C
#undef MBEDTLS_X509_USE_C
#undef MBEDTLS_X509_CRT_PARSE_C
#undef MBEDTLS_PSA_CRYPTO_C

/* --- Suppress unused-module warnings --- */
#define MBEDTLS_NO_UDBL_DIVISION
#define MBEDTLS_HAVE_ASM

#endif /* MBEDTLS_CONFIG_BL3_H */
