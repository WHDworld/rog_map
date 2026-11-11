#ifndef _LKH_INTERFACE_H
#define _LKH_INTERFACE_H

/*
 * NOTE: Do not include LKH.h / Genetic.h here.
 *
 * Upstream LKH defines its global state directly in the headers without
 * `extern` guards (it is designed to be built as a single translation unit
 * with -fcommon).  Exposing those headers to other C++ translation units
 * produces "multiple definition" link errors.  Keep them private to
 * lkh_interface.cpp and expose only this C-linkage entry point.
 */
#ifdef __cplusplus
extern "C" {
#endif

int solveTSPLKH(const char* input_file);

#ifdef __cplusplus
}
#endif

#endif
