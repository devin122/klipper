// Support for gathering acceleration data from ADXL345 chip
//
// Copyright (C) 2020  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include <string.h> // memcpy
#include "board/irq.h" // irq_disable
#include "board/misc.h" // timer_read_time
#include "basecmd.h" // oid_alloc
#include "command.h" // DECL_COMMAND
#include "sched.h" // DECL_TASK
#include "spicmds.h" // spidev_transfer

struct mpu6500 {
    struct timer timer;
    uint32_t rest_ticks;
    struct spidev_s *spi;
    uint16_t sequence, limit_count;
    uint8_t flags, data_count;
    uint8_t data[60];
};

enum {
    AX_HAVE_START = 1<<0, AX_RUNNING = 1<<1, AX_PENDING = 1<<2,
};

static struct task_wake mpu6500_wake;

// Event handler that wakes mpu6500_task() periodically
static uint_fast8_t
mpu6500_event(struct timer *timer)
{
    struct mpu6500 *ax = container_of(timer, struct mpu6500, timer);
    ax->flags |= AX_PENDING;
    sched_wake_task(&mpu6500_wake);
    return SF_DONE;
}

void
command_config_mpu6500(uint32_t *args)
{
    struct mpu6500 *ax = oid_alloc(args[0], command_config_mpu6500
                                   , sizeof(*ax));
    ax->timer.func = mpu6500_event;
    ax->spi = spidev_oid_lookup(args[1]);
}
DECL_COMMAND(command_config_mpu6500, "config_mpu6500 oid=%c spi_oid=%c");

// Report local measurement buffer
static void
mpu_report(struct mpu6500 *ax, uint8_t oid)
{
    sendf("mpu6500_data oid=%c sequence=%hu data=%*s"
          , oid, ax->sequence, ax->data_count, ax->data);
    ax->data_count = 0;
    ax->sequence++;
}

// Report buffer and fifo status
static void
mpu_status(struct mpu6500 *ax, uint_fast8_t oid
            , uint32_t time1, uint32_t time2, uint16_t fifo)
{
    sendf("mpu6500_status oid=%c clock=%u query_ticks=%u next_sequence=%hu"
          " buffered=%c fifo=%c limit_count=%hu"
          , oid, time1, time2-time1, ax->sequence
          , ax->data_count, fifo, ax->limit_count);
}

// Helper code to reschedule the adxl345_event() timer
static void
mpu_reschedule_timer(struct mpu6500 *ax)
{
    irq_disable();
    ax->timer.waketime = timer_read_time() + ax->rest_ticks;
    sched_add_timer(&ax->timer);
    irq_enable();
}

// Chip registers
#if 0
#define AR_POWER_CTL   0x2D
#define AR_DATAX0      0x32
#define AR_FIFO_STATUS 0x39
#define AM_READ  0x80
#define AM_MULTI 0x40

#define SET_FIFO_CTL 0x90
#endif
#define AM_READ  0x80
#define REG_FIFO_COUNT 114
#define REG_FIFO 116
#define REG_FIFO_EN 35

#define FIFO_EN_ACC 8
// Query accelerometer data
static void
mpu_query(struct mpu6500 *ax, uint8_t oid)
{
	// Read bytes waiting in FIFO
	uint8_t msg1[] = {REG_FIFO_COUNT | AM_READ, 0, 0};
	spidev_transfer(ax->spi, 1, sizeof(msg1), msg1);
	uint16_t fifo_count = (msg1[1] << 8) | msg1[2];
	if(fifo_count >= 6) {
		// TODO check for overflow
		// data is ready to read
		uint8_t msg2[] = {REG_FIFO | AM_READ, 0,0,0,0,0,0};
		spidev_transfer(ax->spi,1, sizeof(msg2), msg2);
		uint8_t *d = &ax->data[ax->data_count];
		ax->data_count += 6;

		if (ax->data_count + 6 > ARRAY_SIZE(ax->data))
        	mpu_report(ax, oid);
		// TODO if we actually overflow we need to dump the fifo
		if(fifo_count >= 0x200)
			ax->limit_count++;
		memcpy(d, &msg2[1], 6);
		if(fifo_count > 6) {
			// More data in fifo - wake this task again
			sched_wake_task(&mpu6500_wake);
			return;
		}
	}

	// Sleep until next check time
	sched_del_timer(&ax->timer);
	ax->flags &= ~AX_PENDING;
	mpu_reschedule_timer(ax);
}

// Startup measurements
static void
mpu_start(struct mpu6500 *ax, uint8_t oid)
{
    sched_del_timer(&ax->timer);
    ax->flags = AX_RUNNING;
    uint8_t msg[2] = { REG_FIFO_EN, 0x08 };
    spidev_transfer(ax->spi, 0, sizeof(msg), msg);
    mpu_reschedule_timer(ax);
}

// End measurements
static void
mpu_stop(struct mpu6500 *ax, uint8_t oid)
{
    // Disable measurements
    sched_del_timer(&ax->timer);
    ax->flags = 0;
    uint8_t msg[2] = { REG_FIFO_EN, 0x00 };
    uint32_t end1_time = timer_read_time();
    spidev_transfer(ax->spi, 0, sizeof(msg), msg);
    uint32_t end2_time = timer_read_time();
    // Drain any measurements still in fifo
	uint8_t msg1[] = {REG_FIFO_COUNT | AM_READ, 0, 0};
	spidev_transfer(ax->spi, 1, sizeof(msg1), msg1);
	uint16_t fifo_count = (msg1[1] << 8) | msg1[2];
	fifo_count = fifo_count/6;
	for(int i = 0; i < fifo_count; ++i){
		mpu_query(ax, oid);
	}
	// TODO drain partial buffer?
  
    // Report final data
    if (ax->data_count)
        mpu_report(ax, oid);
    mpu_status(ax, oid, end1_time, end2_time, msg[1]);
}

void
command_query_mpu6500(uint32_t *args)
{
    struct mpu6500 *ax = oid_lookup(args[0], command_config_mpu6500);

    if (!args[2]) {
        // End measurements
        mpu_stop(ax, args[0]);
        return;
    }
    // Start new measurements query
    sched_del_timer(&ax->timer);
    ax->timer.waketime = args[1];
    ax->rest_ticks = args[2];
    ax->flags = AX_HAVE_START;
    ax->sequence = ax->limit_count = 0;
    ax->data_count = 0;
    sched_add_timer(&ax->timer);
}
DECL_COMMAND(command_query_mpu6500,
             "query_mpu6500 oid=%c clock=%u rest_ticks=%u");

void
command_query_mpu6500_status(uint32_t *args)
{
    struct mpu6500 *ax = oid_lookup(args[0], command_config_mpu6500);
    uint8_t msg[3] = {REG_FIFO_COUNT | AM_READ, 0, 0};
    uint32_t time1 = timer_read_time();
    spidev_transfer(ax->spi, 1, sizeof(msg), msg);
	uint16_t fifo_count = (msg[1] << 8) | msg[2];
    uint32_t time2 = timer_read_time();
    mpu_status(ax, args[0], time1, time2, fifo_count);
}
DECL_COMMAND(command_query_mpu6500_status, "query_mpu6500_status oid=%c");

void
mpu6500_task(void)
{
    if (!sched_check_wake(&mpu6500_wake))
        return;
    uint8_t oid;
    struct mpu6500 *ax;
    foreach_oid(oid, ax, command_config_mpu6500) {
        uint_fast8_t flags = ax->flags;
        if (!(flags & AX_PENDING))
            continue;
        if (flags & AX_HAVE_START)
            mpu_start(ax, oid);
        else
            mpu_query(ax, oid);
    }
}
DECL_TASK(mpu6500_task);
