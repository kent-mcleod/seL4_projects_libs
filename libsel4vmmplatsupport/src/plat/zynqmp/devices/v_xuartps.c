/*
 * Copyright 2019, DornerWorks
 *
 * SPDX-License-Identifier: BSD-2-Clause
 */

#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <pthread.h>

#include <sel4vm/guest_irq_controller.h>
#include <sel4vm/guest_vcpu_fault.h>
#include <sel4vm/guest_memory.h>
#include <sel4vm/boot.h>

#include <sel4vmmplatsupport/device.h>
#include <sel4vmmplatsupport/guest_memory_util.h>
#include <sel4vmmplatsupport/plat/device_map.h>
#include <sel4vmmplatsupport/plat/vuart.h>
#include <sel4vmmplatsupport/plat/devices.h>

#include <ringbuffer/ringbuffer.h>

#define VUART_BUFLEN 256

#define CR         0x00 /* Control Register */
#define MR         0x04 /* Mode Register */
#define IER        0x08 /* Interrupt Enable Register */
#define IDR        0x0C /* Interrupt Disable Register */
#define IMR        0x10 /* Interrupt Mask Register */
#define ISR        0x14 /* Channel Interrupt Status Register */
#define BAUDGEN    0x18 /* Baud Rate Generator Register */
#define RXTOUT     0x1C /* Receiver Timeout Register */
#define RXWM       0x20 /* Receiver FIFO Trigger Level Register */
#define MODEMCR    0x24 /* Modem Control Register */
#define MODEMSR    0x28 /* Modem Status Register */
#define SR         0x2C /* Channel Status Register */
#define FIFO       0x30 /* Transmit and Receive FIFO */
#define BAUDDIV    0x34 /* Baud Rate Divider Register */
#define FLOWDEL    0x38 /* Flow Control Delay Register */
#define PAD1       0x3C
#define PAD2       0x40
#define TXWM       0x44 /* Transmitter FIFO Trigger Level Register */
#define UART_SIZE  0x48

struct zynq_uart_regs {
    uint32_t cr;            /* 0x00 Control Register */
    uint32_t mr;            /* 0x04 Mode Register */
    uint32_t ier;           /* 0x08 Interrupt Enable Register */
    uint32_t idr;           /* 0x0C Interrupt Disable Register */
    uint32_t imr;           /* 0x10 Interrupt Mask Register */
    uint32_t isr;           /* 0x14 Channel Interrupt Status Register */
    uint32_t baudgen;       /* 0x18 Baud Rate Generator Register */
    uint32_t rxtout;        /* 0x1C Receiver Timeout Register */
    uint32_t rxwm;          /* 0x20 Receiver FIFO Trigger Level Register */
    uint32_t modemcr;       /* 0x24 Modem Control Register */
    uint32_t modemsr;       /* 0x28 Modem Status Register */
    uint32_t sr;            /* 0x2C Channel Status Register */
    uint32_t fifo;          /* 0x30 Transmit and Receive FIFO */
    uint32_t bauddiv;       /* 0x34 Baud Rate Divider Register */
    uint32_t flowdel;       /* 0x38 Flow Control Delay Register */
    uint32_t pad[2];
    uint32_t txwm;          /* 0x44 Transmitter FIFO Trigger Level Register */
    // Below this line is private device state not accessible by guest.
    // In multikernel mode this state is shared between multiple supervisors
    // The lock variable is used as a spinlock and requires multiple clients
    // to be on mutual exclusive cores.
    uint32_t pad2[0x22];
    uint32_t lock;
    // The _Atomic variables ensure stores, loads and incremental operators
    // happen atomically and with the necessaring store and load ordering.
    _Atomic uint32_t tx_in;
    _Atomic uint32_t tx_out;
    _Atomic uint32_t rx_in;
    _Atomic uint32_t rx_out;
    // Buffers for holding circular queues for implementing fifos
    uint32_t rx_buf[VUART_BUFLEN];
    uint32_t tx_buf[VUART_BUFLEN];
};
typedef struct zynq_uart_regs zynq_uart_regs_t;

#define UART_SR_RTRIG           BIT( 0)
#define UART_SR_REMPTY          BIT( 1)
#define UART_SR_RFUL            BIT( 2)
#define UART_SR_TEMPTY          BIT( 3)
#define UART_SR_TFUL            BIT( 4)
#define UART_SR_RACTIVE         BIT(10)
#define UART_SR_TACTIVE         BIT(11)
#define UART_SR_FDELT           BIT(12)
#define UART_SR_TTRIG           BIT(13)
#define UART_SR_TNFUL           BIT(14)

#define UART_ISR_RTRIG          BIT( 0)
#define UART_ISR_REMPTY         BIT( 1)
#define UART_ISR_RFUL           BIT( 2)
#define UART_ISR_TEMPTY         BIT( 3)
#define UART_ISR_TFUL           BIT( 4)
#define UART_ISR_ROVR           BIT( 5)
#define UART_ISR_FRAME          BIT( 6)
#define UART_ISR_PARE           BIT( 7)
#define UART_ISR_TIMEOUT        BIT( 8)
#define UART_ISR_DMSI           BIT( 9)
#define UART_ISR_TTRIG          BIT(10)
#define UART_ISR_TNFUL          BIT(11)
#define UART_ISR_TOVR           BIT(12)
#define UART_ISR_MASK           (BIT(13)-1)

#define UART_CR_SELF_CLEARING_BITS  (0x43)

#define COLOR_BUF_SZ      6
#define NAME_BUF_SZ       64

struct vuart_priv {
    void *regs;
    char buffer[VUART_BUFLEN];
    int virq;
    int buf_pos;
    int int_pending;
    vm_t *vm;
    print_func_t callback;
};

static struct vuart_priv *vuart_data;

static inline void *vuart_priv_get_regs(struct device *d)
{
    return ((struct vuart_priv *)d->priv)->regs;
}

// For when this device is shared across an SMP vm where
// its data can be accessed in parallel we use a spin lock
static inline void vuart_lock(struct vuart_priv *v) {
    zynq_uart_regs_t *uart_regs = v->regs;
    pthread_spin_lock(&uart_regs->lock);
}

static inline void vuart_unlock(struct vuart_priv *v) {
    zynq_uart_regs_t *uart_regs = v->regs;
    pthread_spin_unlock(&uart_regs->lock);
}

static inline bool vuart_rx_count(struct vuart_priv *v) {
    zynq_uart_regs_t *r = v->regs;
    return (r->rx_in - r->rx_out);

}


static inline bool vuart_rx_full(struct vuart_priv *v) {
    return vuart_rx_count(v) == VUART_BUFLEN;

}

static inline bool vuart_rx_empty(struct vuart_priv *v) {
    zynq_uart_regs_t *r = v->regs;
    return (r->rx_in == r->rx_out);
}


static inline void vuart_rx_push(struct vuart_priv *v, char c) {
    // Single sender, multi receiver
    zynq_uart_regs_t *r = v->regs;
    if (vuart_rx_full(v)) {
        return;
    }
    r->rx_buf[r->rx_in%VUART_BUFLEN] = c;
    // _Atomic store new value + 1.
    r->rx_in = r->rx_in + 1;

}

static inline char vuart_rx_pop(struct vuart_priv *v) {
    // single sender, multi receiver
    zynq_uart_regs_t *r = v->regs;
    if (vuart_rx_empty(v)) {
        return 0;
    }
    char c = r->rx_buf[r->rx_out%VUART_BUFLEN];
    // _Atomic store new value + 1.
    r->rx_out = r->rx_out + 1;

    return c;

}

static inline size_t vuart_tx_count(struct vuart_priv *v) {
    zynq_uart_regs_t *r = v->regs;
    return (r->tx_in - r->tx_out);
}

static inline bool vuart_tx_full(struct vuart_priv *v) {
    return vuart_tx_count(v) == VUART_BUFLEN;
}

static inline bool vuart_tx_empty(struct vuart_priv *v) {
    zynq_uart_regs_t *r = v->regs;
    return (r->tx_in == r->tx_out);
}


static inline void vuart_tx_push(struct vuart_priv *v, char c) {
    zynq_uart_regs_t *r = v->regs;
    if (vuart_tx_full(v)) {
        return;
    }
    r->tx_buf[r->tx_in%VUART_BUFLEN] = c;
    // _Atomic store new value + 1.
    r->tx_in = r->tx_in + 1;
}

static inline char vuart_tx_pop(struct vuart_priv *v) {

    zynq_uart_regs_t *r = v->regs;
    if (vuart_tx_empty(v)) {
        return 0;
    }
    char c = r->tx_buf[r->tx_out%VUART_BUFLEN];
    // _Atomic store new value + 1.
    r->tx_out = r->tx_out + 1;

    return c;
}

static void vuart_data_reset(struct device *d)
{
    void *uart_regs = vuart_priv_get_regs(d);

    /* Default UART registers as defined in the ZUS+ TRM. Since
     * we are emulating the device, we want the VM to see the
     * registers with the values it would expect on reset.
     */
    const uint32_t reset_data[] = { 0x00000128,
                                    0x00000000,
                                    0x00000000,
                                    0x00000000,
                                    0x00000000,
                                    0x00000208,
                                    0x0000028B,
                                    0x00000000,
                                    0x00000020,
                                    0x00000000,
                                    0x00000000,
                                    0x00000000,
                                    0x00000000,
                                    0x0000000F,
                                    0x00000000,
                                    0x00000000,
                                    0x00000000,
                                    0x00000020
                                  };
    memcpy(uart_regs, reset_data, sizeof(reset_data));
}

/* Called by the VM to ACK a virtual IRQ */
static void vuart_ack(vm_vcpu_t *vcpu, int irq, void *cookie)
{
    struct vuart_priv *vuart_data = cookie;
    zynq_uart_regs_t *uart_regs = (zynq_uart_regs_t *)vuart_data->regs;

    // Deliver another interrupt if the interrupt status is still active
    vuart_lock(vuart_data);
    bool another_irq = uart_regs->isr & uart_regs->imr;
    vuart_unlock(vuart_data);

    if (another_irq) {
        /* Another IRQ occured */
        vm_inject_irq(vuart_data->vm->vcpus[BOOT_VCPU], vuart_data->virq);
    } else {
        vuart_data->int_pending = 0;
    }
}


void vuart_handle_irq(int c)
{
    zynq_uart_regs_t *uart_regs = (zynq_uart_regs_t *)vuart_data->regs;

    // Push character into virtual fifo and inject VM interrupt
    vuart_rx_push(vuart_data, (unsigned char)c);

    vuart_lock(vuart_data);
    uart_regs->isr |= UART_ISR_RTRIG;
    vuart_unlock(vuart_data);

    // Inject irq if not IRQ already pending
    if (vuart_data->int_pending == 0) {
        vuart_data->int_pending = 1;
        vm_inject_irq(vuart_data->vm->vcpus[BOOT_VCPU], vuart_data->virq);
    }
}

static void flush_vconsole_device(struct vuart_priv *vuart_data)
{
    char *buf;

    // Flush pending tx fifo data.
    // If no callback is registerd, the data is thrown away.
    size_t count = vuart_tx_count(vuart_data);
    for (int i = 0; i < count; i++) {
        char c = vuart_tx_pop(vuart_data);
        if (vuart_data->callback) {
            vuart_data->callback(c);
        }
    }
}

static void vuart_putchar(struct device *d, char c)
{
    struct vuart_priv *vuart_data;
    assert(d->priv);
    zynq_uart_regs_t *uart_regs = (zynq_uart_regs_t *)vuart_priv_get_regs(d);
    vuart_data = (struct vuart_priv *)d->priv;

    // Push tx char onto the fifo. In a multikernel system multiple cores
    // could be writing to the tx fifo concurrently, so place a critical
    // section here.
    vuart_lock(vuart_data);
    vuart_tx_push(vuart_data, (unsigned char)c);
    vuart_unlock(vuart_data);

    /* We flush after every character is sent instead of only at newlines. This is so typing in characters on the
     * console doesn't look weird. This can be slow when displaying a lot of information quickly.
     *
     * We could probably implement some SW timeout that flushes every so often if there is data available.
     */
    // In multikernel case, only vm_id==0 has a synchronous interface to a hardware serial. On other cores,
    // a message + ipi needs to be transmitted to vmm for vm_id==0 to flush the buffer.
    if (!vuart_data->vm->is_multikernel || vuart_data->vm->vm_id == 0) {
        flush_vconsole_device(vuart_data);
    } else {
        ZF_LOGF_IF(!vuart_data->vm->run.send_message_callback, "Invalid VM state");
        vuart_data->vm->run.send_message_callback(vuart_data->vm->vm_id, 0, FLUSH_TX_QUEUE, 0, vuart_data->vm->run.send_message_callback_cookie);
    }
}

// Callback for flushing a buffer called in an IPI message handler for messages from other cores.
void vuart_flush_tx(void) {
    flush_vconsole_device(vuart_data);

}

static memory_fault_result_t handle_vuart_fault(vm_t *vm, vm_vcpu_t *vcpu, uintptr_t fault_addr, size_t fault_length,
                                                void *cookie)
{
    uint32_t *reg;
    int offset;
    uint32_t mask;
    struct device *dev;
    dev = (struct device *)cookie;
    UNUSED uint32_t v;
    UNUSED int data;
    zynq_uart_regs_t *uart_regs;
    struct vuart_priv *vuart_data = dev->priv;

    uart_regs = (zynq_uart_regs_t *)vuart_priv_get_regs(dev);

    /* Gather fault information */
    offset = fault_addr - dev->pstart;
    reg = (uint32_t *)(vuart_priv_get_regs(dev) + offset - (offset % 4));
    mask = get_vcpu_fault_data_mask(vcpu);

    /* Handle the fault */
    if (offset < 0 || UART_SIZE <= offset) {
        /* Out of range, treat as SBZ */
        set_vcpu_fault_data(vcpu, 0);
        return FAULT_IGNORE;

    } else if (is_vcpu_read_fault(vcpu)) {
        switch (offset) {
        case SR:
            data = 0;
            // Check if any characters available.
            // This doesn't need to be in a critical section.
            if (vuart_rx_count(vuart_data) == 0) {
                data |= UART_SR_REMPTY;
            }
            data |= UART_SR_TEMPTY;
            set_vcpu_fault_data(vcpu, data);
            break;
        case ISR:
            set_vcpu_fault_data(vcpu, uart_regs->isr);
            break;
        case FIFO:
            data = 0;
            // Use critical section to pull from RX fifo
            // multiple cores could be reading at same time.
            vuart_lock(vuart_data);
            size_t count = vuart_rx_count(vuart_data);
            if (count > 0) {
                data = vuart_rx_pop(vuart_data);
            }
            vuart_unlock(vuart_data);
            set_vcpu_fault_data(vcpu, data);
            // If count was 1 then there is no more characters
            if (count == 1) {
                uart_regs->isr &= ~UART_ISR_RTRIG;
            }
            break;
        default:
            /* Blindly read out data */
            set_vcpu_fault_data(vcpu, *reg);
        }
        advance_vcpu_fault(vcpu);

    } else { /* if(fault_is_write(fault))*/
        switch (offset) {
        case IER:
            /* Set bits get set in Interrupt Mask */
            v = (get_vcpu_fault_data(vcpu) & mask);
            // Lock updates so they are atomic
            vuart_lock(vuart_data);
            uart_regs->imr |= v;
            vuart_unlock(vuart_data);
            break;
        case IDR:
            /* Set bits get cleared in Interrupt Mask */
            v = ~(get_vcpu_fault_data(vcpu) & mask);
            // Lock updates so they are atomic
            vuart_lock(vuart_data);
            uart_regs->imr &= v;
            vuart_unlock(vuart_data);
            break;
        case ISR:
            /* Only clear set bits */
            data = get_vcpu_fault_data(vcpu);
            // Lock updates so they are atomic
            vuart_lock(vuart_data);
            v = uart_regs->isr & ~mask;
            v &= ~(data& mask);
            v |= UART_ISR_TEMPTY;
            uart_regs->isr = v;
            vuart_unlock(vuart_data);
            break;
        case BAUDGEN:
        case RXTOUT:
        case RXWM:
        case MODEMCR:
        case MODEMSR:
        case BAUDDIV:
        case FLOWDEL:
        case MR:
        case TXWM:
            /* Blindly write to the device */
            data = get_vcpu_fault_data(vcpu);
            // Lock updates so they are atomic
            vuart_lock(vuart_data);
            v = *reg & ~mask;
            v |= data & mask;
            *reg = v;
            vuart_unlock(vuart_data);
            break;
        case FIFO:
            vuart_putchar(dev, get_vcpu_fault_data(vcpu));
            break;
        case CR:
            data = get_vcpu_fault_data(vcpu);
            // Lock updates so they are atomic
            vuart_lock(vuart_data);
            v = *reg & ~mask;
            v |= data & mask;
            /* Always make sure self clearing bits are cleared
             * since we don't actually let the VM control the UART
             */
            v &= ~(UART_CR_SELF_CLEARING_BITS);
            *reg = v;
            vuart_unlock(vuart_data);
            break;
        default:
            return FAULT_IGNORE;
        }
        advance_vcpu_fault(vcpu);
    }
    return FAULT_HANDLED;
}

const struct device dev_uart0 = {
    .name = "uart0",
    .pstart = UART0_PADDR,
    .size = 0x1000,
    .handle_device_fault = NULL,
    .priv = NULL
};

const struct device dev_uart1 = {
    .name = "uart1",
    .pstart = UART1_PADDR,
    .size = 0x1000,
    .handle_device_fault = NULL,
    .priv = NULL
};

int vm_install_vconsole(vm_t *vm, print_func_t func)
{
    static int once = 0;

    ZF_LOGF_IF(once, "Only install vconsole once\n");

    struct device *d;
    int err;

    d = (struct device *)calloc(1, sizeof(struct device));
    if (!d) {
        return -1;
    }

    *d = dev_vconsole;

    /* Initialise the virtual device */
    vuart_data = calloc(1, sizeof(struct vuart_priv));
    ZF_LOGF_IF(NULL == vuart_data, "Failed to malloc vconsole device\n");

    vuart_data->vm = vm;
    vuart_data->int_pending = 0;
    vuart_data->callback = func;

    if (vm->is_multikernel) {
        // Going to do things differently.
        // Share the memory for regs from the shared buffer between VMs.
        ZF_LOGF_IF(vm->iq_shared_buf_size < 0x1000, "Not enough shared mem available");
        vuart_data->regs = vm->iq_shared_buf;
        vm->iq_shared_buf_size -= 0x1000;
        vm->iq_shared_buf += 0x1000;
    } else {
        vuart_data->regs = calloc(1, sizeof(zynq_uart_regs_t));
        if (vuart_data->regs == NULL) {
            assert(vuart_data->regs);
            return -1;
        }
    }

    vm_memory_reservation_t *reservation = vm_reserve_memory_at(vm, d->pstart, d->size,
                                                                handle_vuart_fault, (void *)d);
    if (!reservation) {
        return -1;
    }

    d->priv = vuart_data;

    // If on multikernel, reset the register state only if first vmid
    if (!vm->is_multikernel || vm->vm_id == 0) {
        vuart_data_reset(d);

        /* Initialise virtual IRQ */
        vuart_data->virq = VCONSOLE_IRQ;
        err = vm_register_irq(vm->vcpus[BOOT_VCPU], VCONSOLE_IRQ, &vuart_ack, vuart_data);
        ZF_LOGF_IF(err, "Failed to initialize vconsole virq\n");
    }

    once = 1;

    return 0;
}

int vm_uninstall_vconsole(vm_t *vm)
{
    return 0;
}
