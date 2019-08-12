/*
 * Copyright 2019, Data61, CSIRO (ABN 41 687 119 230)
 *
 * SPDX-License-Identifier: BSD-2-Clause
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "vm.h"


#define MAX_VIRQS   200
#define NUM_SGI_VIRQS   16
#define NUM_PPI_VIRQS   16
#define GIC_SPI_IRQ_MIN      (NUM_SGI_VIRQS + NUM_PPI_VIRQS)

struct virq_handle {
    int virq;
    irq_ack_fn_t ack;
    void *token;
};

typedef struct virq_handle *virq_handle_t;

/* TODO: A typical number of list registers supported by GIC is four, but not
 * always. One particular way to probe the number of registers is to inject a
 * dummy IRQ with seL4_ARM_VCPU_InjectIRQ(), using LR index high enough to be
 * not supported by any target; the kernel will reply with the supported range
 * of LR indexes.
 */
#define NUM_LIST_REGS 4
/* This is a rather abritrary number, increase if needed. */
#define MAX_IRQ_QUEUE_LEN 64
#define IRQ_QUEUE_NEXT(_i) (((_i) + 1) & (MAX_IRQ_QUEUE_LEN - 1))

static_assert((MAX_IRQ_QUEUE_LEN & (MAX_IRQ_QUEUE_LEN - 1)) == 0,
              "IRQ ring buffer size must be power of two");

struct irq_queue {
    struct virq_handle *irqs[MAX_IRQ_QUEUE_LEN]; /* circular buffer */
    size_t head;
    size_t tail;
};

typedef struct vgic {
/// Mirrors the vcpu list registers
    struct virq_handle *lr_shadow[CONFIG_MAX_NUM_NODES][NUM_LIST_REGS];
/// IRQs that would not fit in the vcpu list registers
    struct irq_queue irq_queue[CONFIG_MAX_NUM_NODES];
/// Complete set of virtual irqs
    struct virq_handle *sgi_ppi_irq[CONFIG_MAX_NUM_NODES][NUM_SGI_VIRQS + NUM_PPI_VIRQS];
    struct virq_handle *virqs[MAX_VIRQS];
/// Virtual distributor registers
    void *registers;
} vgic_t;

int vgic_vcpu_load_list_reg(vgic_t *vgic, vm_vcpu_t *vcpu, int idx, struct virq_handle *irq);

static inline struct vgic *vgic_device_get_vgic(struct vgic_dist_device *d)
{
    assert(d);
    assert(d->priv);
    return (vgic_t *)d->priv;
}

static inline void virq_ack(vm_vcpu_t *vcpu, struct virq_handle *irq)
{
    irq->ack(vcpu, irq->virq, irq->token);
}

static struct virq_handle *virq_get_sgi_ppi(vgic_t *vgic, vm_vcpu_t *vcpu, int virq)
{
    assert(vcpu->vcpu_id < CONFIG_MAX_NUM_NODES);
    return vgic->sgi_ppi_irq[vcpu->vcpu_id][virq];
}

static struct virq_handle *virq_find_spi_irq_data(struct vgic *vgic, int virq)
{
    for (int i = 0; i < MAX_VIRQS; i++) {
        if (vgic->virqs[i] && vgic->virqs[i]->virq == virq) {
            return vgic->virqs[i];
        }
    }
    return NULL;
}

static struct virq_handle *virq_find_irq_data(struct vgic *vgic, vm_vcpu_t *vcpu, int virq)
{
    if (virq < GIC_SPI_IRQ_MIN)  {
        return virq_get_sgi_ppi(vgic, vcpu, virq);
    }
    return virq_find_spi_irq_data(vgic, virq);
}

static int virq_spi_add(vgic_t *vgic, struct virq_handle *virq_data)
{
    for (int i = 0; i < MAX_VIRQS; i++) {
        if (vgic->virqs[i] == NULL) {
            vgic->virqs[i] = virq_data;
            return 0;
        }
    }
    return -1;
}

static int virq_sgi_ppi_add(vm_vcpu_t *vcpu, vgic_t *vgic, struct virq_handle *virq_data)
{
    if (vgic->sgi_ppi_irq[vcpu->vcpu_id][virq_data->virq] != NULL) {
        ZF_LOGE("VIRQ %d already registered for VCPU %u\n", virq_data->virq, vcpu->vcpu_id);
        return -1;
    }
    vgic->sgi_ppi_irq[vcpu->vcpu_id][virq_data->virq] = virq_data;
    return 0;
}

static int virq_add(vm_vcpu_t *vcpu, vgic_t *vgic, struct virq_handle *virq_data)
{
    int virq = virq_data->virq;
    if (virq < GIC_SPI_IRQ_MIN) {
        return virq_sgi_ppi_add(vcpu, vgic, virq_data);
    }
    return virq_spi_add(vgic, virq_data);
}


static inline void vgic_shadow_irq(vgic_t *vgic, int i, struct virq_handle *irq, vm_vcpu_t *vcpu)
{
    vgic->lr_shadow[vcpu->vcpu_id][i] = irq;
}


static inline int vgic_irq_enqueue(vgic_t *vgic, vm_vcpu_t *vcpu, struct virq_handle *irq)
{
    struct irq_queue *q = &vgic->irq_queue[vcpu->vcpu_id];

    if (unlikely(IRQ_QUEUE_NEXT(q->tail) == q->head)) {
        return -1;
    }

    q->irqs[q->tail] = irq;
    q->tail = IRQ_QUEUE_NEXT(q->tail);

    return 0;
}

static inline struct virq_handle *vgic_irq_dequeue(vgic_t *vgic, vm_vcpu_t *vcpu)
{
    struct irq_queue *q = &vgic->irq_queue[vcpu->vcpu_id];
    struct virq_handle *virq = NULL;

    if (q->head != q->tail) {
        virq = q->irqs[q->head];
        q->head = IRQ_QUEUE_NEXT(q->head);
    }

    return virq;
}

static inline int vgic_find_empty_list_reg(vgic_t *vgic, vm_vcpu_t *vcpu)
{
    for (int i = 0; i < NUM_LIST_REGS; i++) {
        if (vgic->lr_shadow[vcpu->vcpu_id][i] == NULL) {
            return i;
        }
    }

    return -1;
}

static inline void virq_init(virq_handle_t virq, int irq, irq_ack_fn_t ack_fn, void *token)
{
    virq->virq = irq;
    virq->token = token;
    virq->ack = ack_fn;
}

