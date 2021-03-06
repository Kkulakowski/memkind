#include "test/jemalloc_test.h"

unsigned arena1_ind;
unsigned arena2_ind;

JEMALLOC_INLINE_C void
time_func(timedelta_t *timer, uint64_t nwarmup, uint64_t niter, void (*func)(void))
{
	uint64_t i;

	for (i = 0; i < nwarmup; i++)
		func();
	timer_start(timer);
	for (i = 0; i < niter; i++)
		func();
	timer_stop(timer);
}

void
compare_funcs(uint64_t nwarmup, uint64_t niter, const char *name_a,
    void (*func_a), const char *name_b, void (*func_b))
{
	timedelta_t timer_a, timer_b;
	char ratio_buf[6];
	void *p;

	p = mallocx(1, 0);
	if (p == NULL) {
		test_fail("Unexpected mallocx() failure");
		return;
	}

	time_func(&timer_a, nwarmup, niter, func_a);
	time_func(&timer_b, nwarmup, niter, func_b);

	timer_ratio(&timer_a, &timer_b, ratio_buf, sizeof(ratio_buf));
	malloc_printf("%"PRIu64" iterations, %s=%"PRIu64"us, "
	    "%s=%"PRIu64"us, ratio=1:%s\n",
	    niter, name_a, timer_usec(&timer_a), name_b, timer_usec(&timer_b),
	    ratio_buf);

	dallocx(p, 0);
}

static void
malloc_free(void)
{
	/* The compiler can optimize away free(malloc(1))! */
	void *p = malloc(1);
	if (p == NULL) {
		test_fail("Unexpected malloc() failure");
		return;
	}
	free(p);
}

static void
mallocx_free(void)
{
	void *p = mallocx(1, 0);
	if (p == NULL) {
		test_fail("Unexpected mallocx() failure");
		return;
	}
	free(p);
}

static void
mallocxa1_free(void)
{
	void *p = mallocx(1, MALLOCX_ARENA(arena1_ind));
	if (p == NULL) {
		test_fail("Unexpected mallocx() failure");
		return;
	}
	free(p);
}

static void
mallocxa2_free(void)
{
	void *p = mallocx(1, MALLOCX_ARENA(arena2_ind));
	if (p == NULL) {
		test_fail("Unexpected mallocx() failure");
		return;
	}
	free(p);
}

TEST_BEGIN(test_malloc_vs_mallocx)
{

	compare_funcs(10*1000*1000, 100*1000*1000, "malloc",
	    malloc_free, "mallocx(0)", mallocx_free);
}
TEST_END

TEST_BEGIN(test_malloc_vs_mallocxa1)
{
	size_t sz;

	sz = sizeof(arena1_ind);
	assert_d_eq(mallctl("arenas.extend", &arena1_ind, &sz, NULL, 0), 0,
	    "Error in arenas.extend");

	compare_funcs(10*1000*1000, 100*1000*1000, "malloc",
	    malloc_free, "mallocx(a)", mallocxa1_free);
}
TEST_END

TEST_BEGIN(test_mallocx_vs_mallocxa1)
{
	size_t sz;

	sz = sizeof(arena1_ind);
	assert_d_eq(mallctl("arenas.extend", &arena1_ind, &sz, NULL, 0), 0,
	    "Error in arenas.extend");

	compare_funcs(10*1000*1000, 100*1000*1000, "mallocx(0)",
	    mallocx_free, "mallocx(a)", mallocxa1_free);
}
TEST_END

#ifdef JEMALLOC_ENABLE_MEMKIND
TEST_BEGIN(test_malloc_vs_mallocxa2)
{
	unsigned partition;
	size_t sz;

	sz = sizeof(arena2_ind);
	partition = 1;
	assert_d_eq(mallctl("arenas.extendk", &arena2_ind, &sz, &partition, sz), 0,
	    "Error in arenas.extendk");

	compare_funcs(10*1000*1000, 100*1000*1000, "malloc",
	    malloc_free, "mallocx(a+tc)", mallocxa2_free);
}
TEST_END

TEST_BEGIN(test_mallocx_vs_mallocxa2)
{
	unsigned partition;
	size_t sz;

	sz = sizeof(arena2_ind);
	partition = 1;
	assert_d_eq(mallctl("arenas.extendk", &arena2_ind, &sz, &partition, sz), 0,
	    "Error in arenas.extendk");

	compare_funcs(10*1000*1000, 100*1000*1000, "mallocx(0)",
	    mallocx_free, "mallocx(a+tc)", mallocxa2_free);
}
TEST_END

TEST_BEGIN(test_mallocxa1_vs_mallocxa2)
{
	unsigned partition;
	size_t sz;

	sz = sizeof(arena1_ind);
	assert_d_eq(mallctl("arenas.extend", &arena1_ind, &sz, NULL, 0), 0,
	    "Error in arenas.extend");
	sz = sizeof(arena2_ind);
	partition = 1;
	assert_d_eq(mallctl("arenas.extendk", &arena2_ind, &sz, &partition, sz), 0,
	    "Error in arenas.extendk");

	compare_funcs(10*1000*1000, 100*1000*1000, "mallocx(a)",
	    mallocxa1_free, "mallocx(a+tc)", mallocxa2_free);
}
TEST_END
#endif

static void
malloc_dallocx(void)
{
	void *p = malloc(1);
	if (p == NULL) {
		test_fail("Unexpected malloc() failure");
		return;
	}
	dallocx(p, 0);
}

TEST_BEGIN(test_free_vs_dallocx)
{

	compare_funcs(10*1000*1000, 100*1000*1000, "free", malloc_free,
	    "dallocx", malloc_dallocx);
}
TEST_END

static void
malloc_mus_free(void)
{
	void *p;

	p = malloc(1);
	if (p == NULL) {
		test_fail("Unexpected malloc() failure");
		return;
	}
	malloc_usable_size(p);
	free(p);
}

static void
malloc_sallocx_free(void)
{
	void *p;

	p = malloc(1);
	if (p == NULL) {
		test_fail("Unexpected malloc() failure");
		return;
	}
	if (sallocx(p, 0) < 1)
		test_fail("Unexpected sallocx() failure");
	free(p);
}

TEST_BEGIN(test_mus_vs_sallocx)
{

	compare_funcs(10*1000*1000, 100*1000*1000, "malloc_usable_size",
	    malloc_mus_free, "sallocx", malloc_sallocx_free);
}
TEST_END

static void
malloc_nallocx_free(void)
{
	void *p;

	p = malloc(1);
	if (p == NULL) {
		test_fail("Unexpected malloc() failure");
		return;
	}
	if (nallocx(1, 0) < 1)
		test_fail("Unexpected nallocx() failure");
	free(p);
}

TEST_BEGIN(test_sallocx_vs_nallocx)
{

	compare_funcs(10*1000*1000, 100*1000*1000, "sallocx",
	    malloc_sallocx_free, "nallocx", malloc_nallocx_free);
}
TEST_END

int
main(void)
{

	return (test(
	    test_malloc_vs_mallocx,
	    test_malloc_vs_mallocxa1,
	    test_mallocx_vs_mallocxa1,
#ifdef JEMALLOC_ENABLE_MEMKIND
	    test_malloc_vs_mallocxa2,
	    test_mallocx_vs_mallocxa2,
	    test_mallocxa1_vs_mallocxa2,
#endif
	    test_free_vs_dallocx,
	    test_mus_vs_sallocx,
	    test_sallocx_vs_nallocx));
}
