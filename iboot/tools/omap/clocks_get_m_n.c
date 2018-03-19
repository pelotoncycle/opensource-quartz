/*
 * Program for finding M & N values for DPLLs
 * To be run on Host PC
 *
 * (C) Copyright 2010
 * Texas Instruments, <www.ti.com>
 *
 * Aneesh V <aneesh@ti.com>
 *
 * See file CREDITS for list of people who contributed to this
 * project.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */
#include <stdlib.h>
#include <stdio.h>

//#define DEBUG

typedef unsigned int u32;
#define MAX_N	127

/*
 * get_m_n_optimized() - Finds optimal DPLL multiplier(M) and divider(N)
 * values based on the reference frequency, required output frequency,
 * maximum tolerance for output frequency etc.
 *
 * target_freq_khz - output frequency required in KHz
 * ref_freq_khz - reference(input) frequency in KHz
 * m - pointer to computed M value
 * n - pointer to computed N value
 * tolerance_khz - tolerance for the output frequency. When the algorithm
 * succeeds in finding vialble M and N values the corresponding output
 * frequency will be in the range:
 *	[target_freq_khz - tolerance_khz, target_freq_khz]
 *
 * Formula:
 *	Fdpll = (2 * M * Fref) / (N + 1)
 *
 * Considerations for lock-time:
 *	- Smaller the N, better lock-time, especially lock-time will be
 *	- For acceptable lock-times:
 *		Fref / (M + 1) >= 1 MHz
 *
 * Considerations for power:
 *	- The difference in power for different N values giving the same
 *	  output is negligible. So, we optimize for lock-time
 *
 * Hard-constraints:
 *	- N can not be greater than 127(7 bit field for representing N)
 *
 * Usage:
 *	$ gcc clocks_get_m_n.c
 *	$ ./a.out
 */
int get_m_n_optimized(u32 target_freq_khz, u32 ref_freq_khz, u32 *M, u32 *N)
{
	u32 freq = target_freq_khz;
	u32 m_optimal, n_optimal, freq_optimal = 0, freq_old;
	u32 m, n;
	n = 1;
	while (1) {
		m = target_freq_khz / ref_freq_khz / 2 * n;
		freq_old = 0;
		while (1) {
			freq = ref_freq_khz * 2 * m / n;
			if (freq > target_freq_khz) {
				freq = freq_old;
				m--;
				break;
			}
			m++;
			freq_old = freq;
		}
		if (freq > freq_optimal) {
			freq_optimal = freq;
			m_optimal = m;
			n_optimal = n;
		}
		n++;
		if ((freq_optimal == target_freq_khz) ||
			((ref_freq_khz / n) < 1000)) {
			break;
		}
	}
	n--;
	*M = m_optimal;
	*N = n_optimal - 1;
	if (freq_optimal == target_freq_khz) {
		printf("ref %d m %d n %d target %d locked %d\n", ref_freq_khz,
			m_optimal, n_optimal - 1, target_freq_khz, freq_optimal);
	} else {
		printf("ref %d m %d n %d target %d locked %d [*]\n", ref_freq_khz,
			m_optimal, n_optimal - 1, target_freq_khz, freq_optimal);
	}
	return 0;
}

/* DPLL rate rounding: minimum DPLL multiplier, divider values */
#define DPLL_MIN_MULTIPLIER		2
#define DPLL_MIN_DIVIDER		1

/* Possible error results from _dpll_test_mult */
#define DPLL_MULT_UNDERFLOW		-1

/*
 * Scale factor to mitigate roundoff errors in DPLL rate rounding.
 * The higher the scale factor, the greater the risk of arithmetic overflow,
 * but the closer the rounded rate to the target rate.  DPLL_SCALE_FACTOR
 * must be a power of DPLL_SCALE_BASE.
 */
#define DPLL_SCALE_FACTOR		64
#define DPLL_SCALE_BASE			2
#define DPLL_ROUNDING_VAL		((DPLL_SCALE_BASE / 2) * \
					 (DPLL_SCALE_FACTOR / DPLL_SCALE_BASE))

#define OMAP4430_MAX_DPLL_MULT	2047
#define OMAP4430_MAX_DPLL_DIV	128

static unsigned long _dpll_compute_new_rate(unsigned long parent_rate,
					    unsigned int m, unsigned int n)
{
	unsigned long long num;

	num = (unsigned long long)parent_rate * (unsigned long long)m;
	return (unsigned long)(num / (unsigned long long)n);
}

/*
 * _dpll_test_mult - test a DPLL multiplier value
 * @m: pointer to the DPLL m (multiplier) value under test
 * @n: current DPLL n (divider) value under test
 * @new_rate: pointer to storage for the resulting rounded rate
 * @target_rate: the desired DPLL rate
 * @parent_rate: the DPLL's parent clock rate
 *
 * This code tests a DPLL multiplier value, ensuring that the
 * resulting rate will not be higher than the target_rate, and that
 * the multiplier value itself is valid for the DPLL.  Initially, the
 * integer pointed to by the m argument should be prescaled by
 * multiplying by DPLL_SCALE_FACTOR.  The code will replace this with
 * a non-scaled m upon return.  This non-scaled m will result in a
 * new_rate as close as possible to target_rate (but not greater than
 * target_rate) given the current (parent_rate, n, prescaled m)
 * triple. Returns DPLL_MULT_UNDERFLOW in the event that the
 * non-scaled m attempted to underflow, which can allow the calling
 * function to bail out early; or 0 upon success.
 */
static int _dpll_test_mult(int *m, int n, unsigned long *new_rate,
			   unsigned long target_rate,
			   unsigned long parent_rate)
{
	int r = 0, carry = 0;

	/* Unscale m and round if necessary */
	if (*m % DPLL_SCALE_FACTOR >= DPLL_ROUNDING_VAL)
		carry = 1;
	*m = (*m / DPLL_SCALE_FACTOR) + carry;

	/*
	 * The new rate must be <= the target rate to avoid programming
	 * a rate that is impossible for the hardware to handle
	 */
	*new_rate = _dpll_compute_new_rate(parent_rate, *m, n);
	if (*new_rate > target_rate) {
		(*m)--;
		*new_rate = 0;
	}

	/* Guard against m underflow */
	if (*m < DPLL_MIN_MULTIPLIER) {
		*m = DPLL_MIN_MULTIPLIER;
		*new_rate = 0;
		r = DPLL_MULT_UNDERFLOW;
	}

	if (*new_rate == 0)
		*new_rate = _dpll_compute_new_rate(parent_rate, *m, n);

	return r;
}

/* Refer to omap2_dpll_round_rate() in Linux Kernel */
int dpll_x2_clk_round_rate(const char* name, u32 target_rate_hz,
		u32 ref_rate_hz, u32 *M, u32 *N)
{
	int r, scaled_max_m;
	unsigned long scaled_rt_rp;
	unsigned long new_rate = 0;
	unsigned long target_rate = target_rate_hz / 2;
	unsigned long ref_rate = ref_rate_hz;
	unsigned long freq_optimal = 0;
	u32 m = 0, n = 0, m_optimal = 0, n_optimal = 0;

	printf("\n%s - %uHz (CLKOUTX2) -> %luHz (CLKOUT)\n",
		name, target_rate_hz, target_rate / 1000UL);

#ifdef DEBUG
	printf("ref %u Hz - starting DPLL round_rate, target rate %luHz\n",
		 ref_rate_hz, target_rate * 2UL);
#endif /* DEBUG */

	scaled_rt_rp = target_rate / (ref_rate / DPLL_SCALE_FACTOR);
	scaled_max_m = OMAP4430_MAX_DPLL_MULT * DPLL_SCALE_FACTOR;

	for (n = 0; n < OMAP4430_MAX_DPLL_DIV; n++) {
		/* Compute the scaled DPLL multiplier, based on the divider */
		m = scaled_rt_rp * (n + 1);

		/*
		 * Since we're counting n up, a m overflow means we
		 * can bail out completely (since as n increases in
		 * the next iteration, there's no way that m can
		 * increase beyond the current m)
		 */
		if (m > scaled_max_m)
			break;

		r = _dpll_test_mult(&m, (n + 1), &new_rate, target_rate, ref_rate);

		/* m can't be set low enough for this n - try with a larger n */
		if (r == DPLL_MULT_UNDERFLOW)
			continue;

#ifdef DEBUG
		printf("ref %uHz m = %u: n = %u: new_rate = %luHz\n",
			 ref_rate_hz, m, (n + 1), new_rate * 2UL);
#endif /* DEBUG */

		if (new_rate > freq_optimal) {
			freq_optimal = new_rate;
			m_optimal = m;
			n_optimal = n;
#ifdef DEBUG
		printf("ref %uHz [optimal] m = %u: n = %u: new_rate = %luHz\n",
			 ref_rate_hz, m_optimal, (n_optimal + 1), freq_optimal * 2UL);
#endif /* DEBUG */
		}

		if ((freq_optimal == target_rate) ||
				(((ref_rate_hz * 1000UL) / (n + 1)) < 1000)) {
			break;
		}
	}

	if (n >= OMAP4430_MAX_DPLL_DIV) {
		printf("ref %u m %u n %u target %u : can not find m & n\n",
				ref_rate_hz, m, n, target_rate_hz);
		return -1;
	}

	(*M) = m_optimal;
	(*N) = n_optimal;

	if (freq_optimal == target_rate) {
		printf("ref %uHz m %u n %u target %uHz locked %luHz -> %luHz\n",
			ref_rate_hz, m_optimal, n_optimal, target_rate_hz,
			freq_optimal * 2UL, freq_optimal);
	} else {
		printf("ref %uHz m %u n %u target %uHz locked %luHz -> %luHz [*]\n",
			ref_rate_hz, m_optimal, n_optimal, target_rate_hz,
			freq_optimal * 2UL, freq_optimal);
	}

	if (((ref_rate_hz * 1000UL) / (n_optimal + 1)) < 1000) {
		printf("\tREFCLK - CLKINP/(N+1) is less than 1 MHz - less than"
			" ideal, locking time will be high!\n");
	}

	return 0;
}

void main(void)
{
	u32 m, n;
	printf("\nMPU - 2000000\n");
	get_m_n_optimized(2000000, 12000, &m, &n);
	get_m_n_optimized(2000000, 13000, &m, &n);
	get_m_n_optimized(2000000, 16800, &m, &n);
	get_m_n_optimized(2000000, 19200, &m, &n);
	get_m_n_optimized(2000000, 26000, &m, &n);
	get_m_n_optimized(2000000, 27000, &m, &n);
	get_m_n_optimized(2000000, 38400, &m, &n);

	printf("\nMPU - 1200000\n");
	get_m_n_optimized(1200000, 12000, &m, &n);
	get_m_n_optimized(1200000, 13000, &m, &n);
	get_m_n_optimized(1200000, 16800, &m, &n);
	get_m_n_optimized(1200000, 19200, &m, &n);
	get_m_n_optimized(1200000, 26000, &m, &n);
	get_m_n_optimized(1200000, 27000, &m, &n);
	get_m_n_optimized(1200000, 38400, &m, &n);

	printf("\nMPU - 1584000\n");
	get_m_n_optimized(1584000, 12000, &m, &n);
	get_m_n_optimized(1584000, 13000, &m, &n);
	get_m_n_optimized(1584000, 16800, &m, &n);
	get_m_n_optimized(1584000, 19200, &m, &n);
	get_m_n_optimized(1584000, 26000, &m, &n);
	get_m_n_optimized(1584000, 27000, &m, &n);
	get_m_n_optimized(1584000, 38400, &m, &n);

	printf("\nMPU - 2200000\n");
	get_m_n_optimized(2200000, 12000, &m, &n);
	get_m_n_optimized(2200000, 13000, &m, &n);
	get_m_n_optimized(2200000, 16800, &m, &n);
	get_m_n_optimized(2200000, 19200, &m, &n);
	get_m_n_optimized(2200000, 26000, &m, &n);
	get_m_n_optimized(2200000, 27000, &m, &n);
	get_m_n_optimized(2200000, 38400, &m, &n);

	printf("\nMPU - 2016000\n");
	get_m_n_optimized(2016000, 12000, &m, &n);
	get_m_n_optimized(2016000, 13000, &m, &n);
	get_m_n_optimized(2016000, 16800, &m, &n);
	get_m_n_optimized(2016000, 19200, &m, &n);
	get_m_n_optimized(2016000, 26000, &m, &n);
	get_m_n_optimized(2016000, 27000, &m, &n);
	get_m_n_optimized(2016000, 38400, &m, &n);

	printf("\nMPU - 1840000\n");
	get_m_n_optimized(1840000, 12000, &m, &n);
	get_m_n_optimized(1840000, 13000, &m, &n);
	get_m_n_optimized(1840000, 16800, &m, &n);
	get_m_n_optimized(1840000, 19200, &m, &n);
	get_m_n_optimized(1840000, 26000, &m, &n);
	get_m_n_optimized(1840000, 27000, &m, &n);
	get_m_n_optimized(1840000, 38400, &m, &n);

	printf("\nMPU - 1600000\n");
	get_m_n_optimized(1600000, 12000, &m, &n);
	get_m_n_optimized(1600000, 13000, &m, &n);
	get_m_n_optimized(1600000, 16800, &m, &n);
	get_m_n_optimized(1600000, 19200, &m, &n);
	get_m_n_optimized(1600000, 26000, &m, &n);
	get_m_n_optimized(1600000, 27000, &m, &n);
	get_m_n_optimized(1600000, 38400, &m, &n);

	printf("\nMPU - 1400000\n");
	get_m_n_optimized(1400000, 12000, &m, &n);
	get_m_n_optimized(1400000, 13000, &m, &n);
	get_m_n_optimized(1400000, 16800, &m, &n);
	get_m_n_optimized(1400000, 19200, &m, &n);
	get_m_n_optimized(1400000, 26000, &m, &n);
	get_m_n_optimized(1400000, 27000, &m, &n);
	get_m_n_optimized(1400000, 38400, &m, &n);

	printf("\nMPU - 1100000\n");
	get_m_n_optimized(1100000, 12000, &m, &n);
	get_m_n_optimized(1100000, 13000, &m, &n);
	get_m_n_optimized(1100000, 16800, &m, &n);
	get_m_n_optimized(1100000, 19200, &m, &n);
	get_m_n_optimized(1100000, 26000, &m, &n);
	get_m_n_optimized(1100000, 27000, &m, &n);
	get_m_n_optimized(1100000, 38400, &m, &n);

	printf("\nMPU - 800000\n");
	get_m_n_optimized(800000, 12000, &m, &n);
	get_m_n_optimized(800000, 13000, &m, &n);
	get_m_n_optimized(800000, 16800, &m, &n);
	get_m_n_optimized(800000, 19200, &m, &n);
	get_m_n_optimized(800000, 26000, &m, &n);
	get_m_n_optimized(800000, 27000, &m, &n);
	get_m_n_optimized(800000, 38400, &m, &n);

	printf("\nMPU - 793600\n");
	get_m_n_optimized(793600, 12000, &m, &n);
	get_m_n_optimized(793600, 13000, &m, &n);
	get_m_n_optimized(793600, 16800, &m, &n);
	get_m_n_optimized(793600, 19200, &m, &n);
	get_m_n_optimized(793600, 26000, &m, &n);
	get_m_n_optimized(793600, 27000, &m, &n);
	get_m_n_optimized(793600, 38400, &m, &n);

	printf("\nMPU - 700000\n");
	get_m_n_optimized(700000, 12000, &m, &n);
	get_m_n_optimized(700000, 13000, &m, &n);
	get_m_n_optimized(700000, 16800, &m, &n);
	get_m_n_optimized(700000, 19200, &m, &n);
	get_m_n_optimized(700000, 26000, &m, &n);
	get_m_n_optimized(700000, 27000, &m, &n);
	get_m_n_optimized(700000, 38400, &m, &n);

	printf("\nMPU - 600000\n");
	get_m_n_optimized(600000, 12000, &m, &n);
	get_m_n_optimized(600000, 13000, &m, &n);
	get_m_n_optimized(600000, 16800, &m, &n);
	get_m_n_optimized(600000, 19200, &m, &n);
	get_m_n_optimized(600000, 26000, &m, &n);
	get_m_n_optimized(600000, 27000, &m, &n);
	get_m_n_optimized(600000, 38400, &m, &n);

	printf("\nCore 1866666\n");
	get_m_n_optimized(1866666, 12000, &m, &n);
	get_m_n_optimized(1866666, 13000, &m, &n);
	get_m_n_optimized(1866666, 16800, &m, &n);
	get_m_n_optimized(1866666, 19200, &m, &n);
	get_m_n_optimized(1866666, 26000, &m, &n);
	get_m_n_optimized(1866666, 27000, &m, &n);
	get_m_n_optimized(1866666, 38400, &m, &n);

	printf("\nCore 1866000\n");
	get_m_n_optimized(1866000, 12000, &m, &n);
	get_m_n_optimized(1866000, 13000, &m, &n);
	get_m_n_optimized(1866000, 16800, &m, &n);
	get_m_n_optimized(1866000, 19200, &m, &n);
	get_m_n_optimized(1866000, 26000, &m, &n);
	get_m_n_optimized(1866000, 27000, &m, &n);
	get_m_n_optimized(1866000, 38400, &m, &n);

	printf("\nCore 1600000\n");
	get_m_n_optimized(1600000, 12000, &m, &n);
	get_m_n_optimized(1600000, 13000, &m, &n);
	get_m_n_optimized(1600000, 16800, &m, &n);
	get_m_n_optimized(1600000, 19200, &m, &n);
	get_m_n_optimized(1600000, 26000, &m, &n);
	get_m_n_optimized(1600000, 27000, &m, &n);
	get_m_n_optimized(1600000, 38400, &m, &n);

	printf("\nPER 1536000\n");
	get_m_n_optimized(1536000, 12000, &m, &n);
	get_m_n_optimized(1536000, 13000, &m, &n);
	get_m_n_optimized(1536000, 16800, &m, &n);
	get_m_n_optimized(1536000, 19200, &m, &n);
	get_m_n_optimized(1536000, 26000, &m, &n);
	get_m_n_optimized(1536000, 27000, &m, &n);
	get_m_n_optimized(1536000, 38400, &m, &n);

	printf("\nIVA 1862400\n");
	get_m_n_optimized(1862400, 12000, &m, &n);
	get_m_n_optimized(1862400, 13000, &m, &n);
	get_m_n_optimized(1862400, 16800, &m, &n);
	get_m_n_optimized(1862400, 19200, &m, &n);
	get_m_n_optimized(1862400, 26000, &m, &n);
	get_m_n_optimized(1862400, 27000, &m, &n);
	get_m_n_optimized(1862400, 38400, &m, &n);

	printf("\nIVA 1862000\n");
	get_m_n_optimized(1862000, 12000, &m, &n);
	get_m_n_optimized(1862000, 13000, &m, &n);
	get_m_n_optimized(1862000, 16800, &m, &n);
	get_m_n_optimized(1862000, 19200, &m, &n);
	get_m_n_optimized(1862000, 26000, &m, &n);
	get_m_n_optimized(1862000, 27000, &m, &n);
	get_m_n_optimized(1862000, 38400, &m, &n);

	printf("\nIVA Nitro - 1290000\n");
	get_m_n_optimized(1290000, 12000, &m, &n);
	get_m_n_optimized(1290000, 13000, &m, &n);
	get_m_n_optimized(1290000, 16800, &m, &n);
	get_m_n_optimized(1290000, 19200, &m, &n);
	get_m_n_optimized(1290000, 26000, &m, &n);
	get_m_n_optimized(1290000, 27000, &m, &n);
	get_m_n_optimized(1290000, 38400, &m, &n);

	printf("\nABE 196608 sys clk\n");
	get_m_n_optimized(196608, 12000, &m, &n);
	get_m_n_optimized(196608, 13000, &m, &n);
	get_m_n_optimized(196608, 16800, &m, &n);
	get_m_n_optimized(196608, 19200, &m, &n);
	get_m_n_optimized(196608, 26000, &m, &n);
	get_m_n_optimized(196608, 27000, &m, &n);
	get_m_n_optimized(196608, 38400, &m, &n);

	printf("\nABE 196608 32K\n");
	get_m_n_optimized(196608000/4, 32768, &m, &n);

	printf("\nUSB 1920000\n");
	get_m_n_optimized(1920000, 12000, &m, &n);
	get_m_n_optimized(1920000, 13000, &m, &n);
	get_m_n_optimized(1920000, 16800, &m, &n);
	get_m_n_optimized(1920000, 19200, &m, &n);
	get_m_n_optimized(1920000, 26000, &m, &n);
	get_m_n_optimized(1920000, 27000, &m, &n);
	get_m_n_optimized(1920000, 38400, &m, &n);

	printf("\nCore ES1 1523712\n");
	get_m_n_optimized(1524000, 12000, &m, &n);
	get_m_n_optimized(1524000, 13000, &m, &n);
	get_m_n_optimized(1524000, 16800, &m, &n);
	get_m_n_optimized(1524000, 19200, &m, &n);
	get_m_n_optimized(1524000, 26000, &m, &n);
	get_m_n_optimized(1524000, 27000, &m, &n);

	/* exact recommendation for SDPs */
	get_m_n_optimized(1523712, 38400, &m, &n);

	printf("\n=================================================================\n");
	dpll_x2_clk_round_rate("MPU", 2000000000, 38400000, &m, &n);
	dpll_x2_clk_round_rate("MPU", 1200000000, 38400000, &m, &n);
	dpll_x2_clk_round_rate("MPU", 1584000000, 38400000, &m, &n);

	dpll_x2_clk_round_rate("MPU", 2200000000, 38400000, &m, &n);
	dpll_x2_clk_round_rate("MPU", 2016000000, 38400000, &m, &n);
	dpll_x2_clk_round_rate("MPU", 1840000000, 38400000, &m, &n);
	dpll_x2_clk_round_rate("MPU", 1600000000, 38400000, &m, &n);
	dpll_x2_clk_round_rate("MPU", 1400000000, 38400000, &m, &n);
	dpll_x2_clk_round_rate("MPU", 1100000000, 38400000, &m, &n);
	dpll_x2_clk_round_rate("MPU", 800000000, 38400000, &m, &n);
	dpll_x2_clk_round_rate("MPU", 793600000, 38400000, &m, &n);
	dpll_x2_clk_round_rate("MPU", 700000000, 38400000, &m, &n);
	dpll_x2_clk_round_rate("MPU", 600000000, 38400000, &m, &n);

	dpll_x2_clk_round_rate("Core", 1866666666, 38400000, &m, &n);
	dpll_x2_clk_round_rate("Core", 1866000000, 38400000, &m, &n);
	dpll_x2_clk_round_rate("Core", 1600000000, 38400000, &m, &n);
	dpll_x2_clk_round_rate("PER", 1536000000, 38400000, &m, &n);
	dpll_x2_clk_round_rate("IVA", 1862400000, 38400000, &m, &n);
	dpll_x2_clk_round_rate("IVA", 1862000000, 38400000, &m, &n);
	dpll_x2_clk_round_rate("ABE sysclk", 196608000, 38400000, &m, &n);
	dpll_x2_clk_round_rate("ABE 32K", 196608000/4, 32768, &m, &n);
	dpll_x2_clk_round_rate("USB", 1920000000, 38400000, &m, &n);
	printf("\n=================================================================\n\n");
}
