/*
 * Copyright (C) 2014 - 2016 Intel Corporation.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 1. Redistributions of source code must retain the above copyright notice(s),
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice(s),
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDER(S) ``AS IS'' AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO
 * EVENT SHALL THE COPYRIGHT HOLDER(S) BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
 * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
 * ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <numa.h>
#include <numaif.h>
#include <assert.h>
#include <sys/mman.h>

#include <memkind.h>

#include <memkind/internal/memkind_hbw.h>
#include <memkind/internal/memkind_default.h>
#include <memkind/internal/memkind_arena.h>
#include <memkind/internal/memkind_private.h>
#include <memkind/internal/memkind_pinned.h>

void memkind_hbw_pinned_init_once(void);

const struct memkind_ops MEMKIND_HBW_PINNED_OPS = {
    .create = memkind_arena_create,
    .destroy = memkind_arena_destroy,
    .malloc = memkind_arena_malloc,
    .calloc = memkind_arena_calloc,
    .posix_memalign = memkind_arena_posix_memalign,
    .realloc = memkind_arena_realloc,
    .free = memkind_default_free,
    .check_available = memkind_hbw_check_available,
    .mbind = memkind_default_mbind,
    .mmap = memkind_pinned_mmap,
    .get_mmap_flags = memkind_default_get_mmap_flags,
    .get_mbind_mode = memkind_default_get_mbind_mode,
    .get_mbind_nodemask = memkind_hbw_all_get_mbind_nodemask,
    .get_arena = memkind_thread_get_arena,
    .get_size = memkind_default_get_size,
    .init_once = memkind_hbw_pinned_init_once,
};

//right now we are assuming 4096 as page size, as this will make code working correctly for all
//eventual pagesizes (2MB, 1GB) but it should be improved in future
#define DEFAULT_PAGESIZE 4096
#define NSTEP 1024 // number of pages pinned per iteration

static void fault_pages(void* ptr, size_t size){
    char* tmp_ptr;
    char* start_addr = (char*) ptr;
    for(tmp_ptr = start_addr; tmp_ptr < (start_addr + size); tmp_ptr+= DEFAULT_PAGESIZE){
        *tmp_ptr = 0;
    }
}

void *memkind_pinned_mmap(struct memkind *kind, void *addr, size_t size)
{
	int err = 0;
    void *result = memkind_default_mmap(kind, addr, size); 
    if(result == MAP_FAILED)
    {
		return MAP_FAILED;
    }

	nodemask_t nodemask;
    struct bitmask nodemask_bm = {NUMA_NUM_NODES, nodemask.n};
    numa_bitmask_clearall(&nodemask_bm);

    kind->ops->get_mbind_nodemask(kind, nodemask.n, NUMA_NUM_NODES);

    void* address[NSTEP];
    int status[NSTEP];
    unsigned long n = size / DEFAULT_PAGESIZE;
    unsigned long step = 0;
    unsigned long i,j;

    for(i=0; i<n; i+=step ) {
        if(i+NSTEP < n) {
            step = NSTEP;
        }
        else {
            step = n - i;
        }

        for(j=0; j<step; j++)
        {
            address[j]= result + (i+j)*DEFAULT_PAGESIZE;
        }

        fault_pages(address[0], step*DEFAULT_PAGESIZE);

        move_pages(0, step, address, NULL, status, MPOL_MF_MOVE);

        for(j=step-1; j>0; j--)
        {
            if(numa_bitmask_isbitset(&nodemask_bm, status[j]) == 0){
                err=1;
                goto exit;
            }
        }
    }

exit:
    if(err)
    {
        munmap(result, size);
        return MAP_FAILED;
    }
    return result;
}

void memkind_hbw_pinned_init_once(void)
{
    int err = memkind_arena_create_map(MEMKIND_HBW_PINNED);
    assert(err == 0);
    err = numa_available();
    assert(err == 0);
    memkind_register_kind(MEMKIND_HBW_PINNED);
}
