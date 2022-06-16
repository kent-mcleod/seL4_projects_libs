/*
 * Copyright 2019, Data61, CSIRO (ABN 41 687 119 230)
 *
 * SPDX-License-Identifier: BSD-2-Clause
 */

//#define DEBUG_IRQ
//#define DEBUG_DIST

#ifdef DEBUG_IRQ
#define DIRQ(...) do{ printf("VDIST: "); printf(__VA_ARGS__); }while(0)
#else
#define DIRQ(...) do{}while(0)
#endif

#ifdef DEBUG_DIST
#define DDIST(...) do{ printf("VDIST: "); printf(__VA_ARGS__); }while(0)
#else
#define DDIST(...) do{}while(0)
#endif

/* GIC Distributor register access utilities */
#define GIC_DIST_REGN(offset, reg) ((offset-reg)/sizeof(uint32_t))
#define RANGE32(a, b) a ... b + (sizeof(uint32_t)-1)

#define IRQ_IDX(irq) ((irq) / 32)
#define IRQ_BIT(irq) (1U << ((irq) % 32))

extern const struct vgic_dist_device dev_vgic_dist;



static inline void set_sgi_ppi_pending(struct gic_dist_map *gic_dist, int irq, bool set_pending, int vcpu_id)
{
    if (set_pending) {
        gic_dist->pending_set0[vcpu_id] |= IRQ_BIT(irq);
        gic_dist->pending_clr0[vcpu_id] |= IRQ_BIT(irq);
    } else {
        gic_dist->pending_set0[vcpu_id] &= ~IRQ_BIT(irq);
        gic_dist->pending_clr0[vcpu_id] &= ~IRQ_BIT(irq);
    }
}

static inline void set_spi_pending(struct gic_dist_map *gic_dist, int irq, bool set_pending)
{
    if (set_pending) {
        gic_dist->pending_set[IRQ_IDX(irq)] |= IRQ_BIT(irq);
        gic_dist->pending_clr[IRQ_IDX(irq)] |= IRQ_BIT(irq);
    } else {
        gic_dist->pending_set[IRQ_IDX(irq)] &= ~IRQ_BIT(irq);
        gic_dist->pending_clr[IRQ_IDX(irq)] &= ~IRQ_BIT(irq);
    }
}

static inline void set_pending(struct gic_dist_map *gic_dist, int irq, bool set_pending, int vcpu_id)
{
    if (irq < GIC_SPI_IRQ_MIN) {
        set_sgi_ppi_pending(gic_dist, irq, set_pending, vcpu_id);
        return;
    }
    set_spi_pending(gic_dist, irq, set_pending);
}

static inline bool is_sgi_ppi_pending(struct gic_dist_map *gic_dist, int irq, int vcpu_id)
{
    return !!(gic_dist->pending_set0[vcpu_id] & IRQ_BIT(irq));
}

static inline bool is_spi_pending(struct gic_dist_map *gic_dist, int irq)
{
    return !!(gic_dist->pending_set[IRQ_IDX(irq)] & IRQ_BIT(irq));
}

static inline bool is_pending(struct gic_dist_map *gic_dist, int irq, int vcpu_id)
{
    if (irq < GIC_SPI_IRQ_MIN) {
        return is_sgi_ppi_pending(gic_dist, irq, vcpu_id);

    }
    return is_spi_pending(gic_dist, irq);
}

static inline void set_sgi_ppi_enable(struct gic_dist_map *gic_dist, int irq, bool set_enable, int vcpu_id)
{
    if (set_enable) {
        gic_dist->enable_set0[vcpu_id] |= IRQ_BIT(irq);
        gic_dist->enable_clr0[vcpu_id] |= IRQ_BIT(irq);
    } else {
        gic_dist->enable_set0[vcpu_id] &= ~IRQ_BIT(irq);
        gic_dist->enable_clr0[vcpu_id] &= ~IRQ_BIT(irq);
    }
}


static inline void set_spi_enable(struct gic_dist_map *gic_dist, int irq, bool set_enable)
{
    if (set_enable) {
        gic_dist->enable_set[IRQ_IDX(irq)] |= IRQ_BIT(irq);
        gic_dist->enable_clr[IRQ_IDX(irq)] |= IRQ_BIT(irq);
    } else {
        gic_dist->enable_set[IRQ_IDX(irq)] &= ~IRQ_BIT(irq);
        gic_dist->enable_clr[IRQ_IDX(irq)] &= ~IRQ_BIT(irq);
    }
}

static inline void set_enable(struct gic_dist_map *gic_dist, int irq, bool set_enable, int vcpu_id)
{
    if (irq < GIC_SPI_IRQ_MIN) {
        set_sgi_ppi_enable(gic_dist, irq, set_enable, vcpu_id);
        return;
    }
    set_spi_enable(gic_dist, irq, set_enable);
}

static inline bool is_sgi_ppi_enabled(struct gic_dist_map *gic_dist, int irq, int vcpu_id)
{
    return !!(gic_dist->enable_set0[vcpu_id] & IRQ_BIT(irq));
}

static inline bool is_spi_enabled(struct gic_dist_map *gic_dist, int irq)
{
    return !!(gic_dist->enable_set[IRQ_IDX(irq)] & IRQ_BIT(irq));
}

static inline bool is_enabled(struct gic_dist_map *gic_dist, int irq, int vcpu_id)
{
    if (irq < GIC_SPI_IRQ_MIN) {
        return is_sgi_ppi_enabled(gic_dist, irq, vcpu_id);
    }
    return is_spi_enabled(gic_dist, irq);
}

static inline bool is_sgi_ppi_active(struct gic_dist_map *gic_dist, int irq, int vcpu_id)
{
    return !!(gic_dist->active0[vcpu_id] & IRQ_BIT(irq));
}

static inline bool is_spi_active(struct gic_dist_map *gic_dist, int irq)
{
    return !!(gic_dist->active[IRQ_IDX(irq)] & IRQ_BIT(irq));
}

static inline bool is_active(struct gic_dist_map *gic_dist, int irq, int vcpu_id)
{
    if (irq < GIC_SPI_IRQ_MIN) {
        return is_sgi_ppi_active(gic_dist, irq, vcpu_id);
    }
    return is_spi_active(gic_dist, irq);
}

static int vgic_dist_enable(struct gic_dist_map *gic_dist)
{
    DDIST("enabling gic distributor\n");
    gic_dist_enable(gic_dist);
    return 0;
}

static int vgic_dist_disable(struct gic_dist_map *gic_dist)
{
    DDIST("disabling gic distributor\n");
    gic_dist_disable(gic_dist);
    return 0;
}

static int vgic_dist_enable_irq(vgic_t *vgic, struct gic_dist_map *gic_dist, vm_vcpu_t *vcpu, int irq)
{
    struct virq_handle *virq_data = virq_find_irq_data(vgic, vcpu, irq);
    DDIST("enabling irq %d\n", irq);
    set_enable(gic_dist, irq, true, vcpu->vcpu_id);
    if (virq_data) {
        /* STATE b) */
        if (!is_pending(gic_dist, virq_data->virq, vcpu->vcpu_id)) {
            virq_ack(vcpu, virq_data);
        }
    } else {
        DDIST("enabled irq %d has no handle", irq);
    }
    return 0;
}

static int vgic_dist_disable_irq(struct gic_dist_map *gic_dist, vm_vcpu_t *vcpu, int irq)
{
    /* STATE g) */

    /* It is IMPLEMENTATION DEFINED if a GIC allows disabling SGIs. Our vGIC
     * implementation does not allows it, such requests are simply ignored.
     * Since it is not uncommon that a guest OS tries disabling SGIs, e.g. as
     * part of the platform initialization, no dedicated messages are logged
     * here to avoid bloating the logs.
     */
    if (irq >= NUM_SGI_VIRQS) {
        DDIST("disabling irq %d\n", irq);
        set_enable(gic_dist, irq, false, vcpu->vcpu_id);
    }
    return 0;
}

static int vgic_dist_set_pending_irq(vgic_t *vgic, vm_vcpu_t *vcpu, int irq)
{
    /* STATE c) */
    struct gic_dist_map *gic_dist = priv_get_dist(vgic->dist);
    struct virq_handle *virq_data = virq_find_irq_data(vgic, vcpu, irq);

    if (!virq_data || !gic_dist_is_enabled(gic_dist) || !is_enabled(gic_dist, irq, vcpu->vcpu_id)) {
        DDIST("IRQ not enabled (%d) on vcpu %d\n", irq, vcpu->vcpu_id);
        return -1;
    }

    if (is_pending(gic_dist, virq_data->virq, vcpu->vcpu_id)) {
        return 0;
    }

    DDIST("Pending set: Inject IRQ from pending set (%d)\n", irq);
    set_pending(gic_dist, virq_data->virq, true, vcpu->vcpu_id);

    /* Enqueueing an IRQ and dequeueing it right after makes little sense
     * now, but in the future this is needed to support IRQ priorities.
     */
    int err = vgic_irq_enqueue(vgic, vcpu, virq_data);
    if (err) {
        ZF_LOGF("Failure enqueueing IRQ, increase MAX_IRQ_QUEUE_LEN");
        return -1;
    }

    int idx = vgic_find_empty_list_reg(vgic, vcpu);
    if (idx < 0) {
        /* There were no empty list registers available, but that's not a big
         * deal -- we have already enqueued this IRQ and eventually the vGIC
         * maintenance code will load it to a list register from the queue.
         */
        return 0;
    }

    struct virq_handle *virq = vgic_irq_dequeue(vgic, vcpu);
    assert(virq);

    return vgic_vcpu_load_list_reg(vgic, vcpu, idx, virq);

}

static int vgic_dist_clr_pending_irq(struct gic_dist_map *gic_dist, vm_vcpu_t *vcpu, int irq)
{
    DDIST("clr pending irq %d\n", irq);
    set_pending(gic_dist, irq, false, vcpu->vcpu_id);
    /* TODO: remove from IRQ queue and list registers as well */
    return 0;
}

static memory_fault_result_t handle_vgic_dist_read_fault(vm_t *vm, vm_vcpu_t *vcpu, uintptr_t fault_addr,
                                                         size_t fault_length,
                                                         void *cookie)
{
    int err = 0;
    fault_t *fault = vcpu->vcpu_arch.fault;
    struct vgic_dist_device *d = (struct vgic_dist_device *)cookie;
    struct vgic *vgic = vgic_device_get_vgic(d);
    struct gic_dist_map *gic_dist = priv_get_dist(vgic->dist);
    int offset = fault_get_address(fault) - d->pstart;
    int vcpu_id = vcpu->vcpu_id;
    uint32_t reg = 0;
    int reg_offset = 0;
    uintptr_t base_reg;
    uint32_t *reg_ptr;
    switch (offset) {
    case RANGE32(GIC_DIST_CTLR, GIC_DIST_CTLR):
        reg = gic_dist->enable;
        break;
    case RANGE32(GIC_DIST_TYPER, GIC_DIST_TYPER):
        reg = gic_dist->ic_type;
        break;
    case RANGE32(GIC_DIST_IIDR, GIC_DIST_IIDR):
        reg = gic_dist->dist_ident;
        break;
    case RANGE32(0x00C, 0x01C):
        /* Reserved */
        break;
    case RANGE32(0x020, 0x03C):
        /* Implementation defined */
        break;
    case RANGE32(0x040, 0x07C):
        /* Reserved */
        break;
    case RANGE32(GIC_DIST_IGROUPR0, GIC_DIST_IGROUPR0):
        reg = gic_dist->irq_group0[vcpu->vcpu_id];
        break;
    case RANGE32(GIC_DIST_IGROUPR1, GIC_DIST_IGROUPRN):
        reg_offset = GIC_DIST_REGN(offset, GIC_DIST_IGROUPR1);
        reg = gic_dist->irq_group[reg_offset];
        break;
    case RANGE32(GIC_DIST_ISENABLER0, GIC_DIST_ISENABLER0):
        reg = gic_dist->enable_set0[vcpu->vcpu_id];
        break;
    case RANGE32(GIC_DIST_ISENABLER1, GIC_DIST_ISENABLERN):
        reg_offset = GIC_DIST_REGN(offset, GIC_DIST_ISENABLER1);
        reg = gic_dist->enable_set[reg_offset];
        break;
    case RANGE32(GIC_DIST_ICENABLER0, GIC_DIST_ICENABLER0):
        reg = gic_dist->enable_clr0[vcpu->vcpu_id];
        break;
    case RANGE32(GIC_DIST_ICENABLER1, GIC_DIST_ICENABLERN):
        reg_offset = GIC_DIST_REGN(offset, GIC_DIST_ICENABLER1);
        reg = gic_dist->enable_clr[reg_offset];
        break;
    case RANGE32(GIC_DIST_ISPENDR0, GIC_DIST_ISPENDR0):
        reg = gic_dist->pending_set0[vcpu->vcpu_id];
        break;
    case RANGE32(GIC_DIST_ISPENDR1, GIC_DIST_ISPENDRN):
        reg_offset = GIC_DIST_REGN(offset, GIC_DIST_ISPENDR1);
        reg = gic_dist->pending_set[reg_offset];
        break;
    case RANGE32(GIC_DIST_ICPENDR0, GIC_DIST_ICPENDR0):
        reg = gic_dist->pending_clr0[vcpu->vcpu_id];
        break;
    case RANGE32(GIC_DIST_ICPENDR1, GIC_DIST_ICPENDRN):
        reg_offset = GIC_DIST_REGN(offset, GIC_DIST_ICPENDR1);
        reg = gic_dist->pending_clr[reg_offset];
        break;
    case RANGE32(GIC_DIST_ISACTIVER0, GIC_DIST_ISACTIVER0):
        reg = gic_dist->active0[vcpu->vcpu_id];
        break;
    case RANGE32(GIC_DIST_ISACTIVER1, GIC_DIST_ISACTIVERN):
        reg_offset = GIC_DIST_REGN(offset, GIC_DIST_ISACTIVER1);
        reg = gic_dist->active[reg_offset];
        break;
    case RANGE32(GIC_DIST_ICACTIVER0, GIC_DIST_ICACTIVER0):
        reg = gic_dist->active_clr0[vcpu->vcpu_id];
        break;
    case RANGE32(GIC_DIST_ICACTIVER1, GIC_DIST_ICACTIVERN):
        reg_offset = GIC_DIST_REGN(offset, GIC_DIST_ICACTIVER1);
        reg = gic_dist->active_clr[reg_offset];
        break;
    case RANGE32(GIC_DIST_IPRIORITYR0, GIC_DIST_IPRIORITYR7):
        reg_offset = GIC_DIST_REGN(offset, GIC_DIST_IPRIORITYR0);
        reg = gic_dist->priority0[vcpu->vcpu_id][reg_offset];
        break;
    case RANGE32(GIC_DIST_IPRIORITYR8, GIC_DIST_IPRIORITYRN):
        reg_offset = GIC_DIST_REGN(offset, GIC_DIST_IPRIORITYR8);
        reg = gic_dist->priority[reg_offset];
        break;
    case RANGE32(0x7FC, 0x7FC):
        /* Reserved */
        break;
    case RANGE32(GIC_DIST_ITARGETSR0, GIC_DIST_ITARGETSR7):
        reg_offset = GIC_DIST_REGN(offset, GIC_DIST_ITARGETSR0);
        reg = gic_dist->targets0[vcpu->vcpu_id][reg_offset];
        break;
    case RANGE32(GIC_DIST_ITARGETSR8, GIC_DIST_ITARGETSRN):
        reg_offset = GIC_DIST_REGN(offset, GIC_DIST_ITARGETSR8);
        reg = gic_dist->targets[reg_offset];
        break;
    case RANGE32(0xBFC, 0xBFC):
        /* Reserved */
        break;
    case RANGE32(GIC_DIST_ICFGR0, GIC_DIST_ICFGRN):
        reg_offset = GIC_DIST_REGN(offset, GIC_DIST_ICFGR0);
        reg = gic_dist->config[reg_offset];
        break;
    case RANGE32(0xD00, 0xDE4):
        base_reg = (uintptr_t) & (gic_dist->spi[0]);
        reg_ptr = (uint32_t *)(base_reg + (offset - 0xD00));
        reg = *reg_ptr;
        break;
    case RANGE32(0xDE8, 0xEFC):
        /* Reserved [0xDE8 - 0xE00) */
        /* GIC_DIST_NSACR [0xE00 - 0xF00) - Not supported */
        break;
    case RANGE32(GIC_DIST_SGIR, GIC_DIST_SGIR):
        reg = gic_dist->sgi_control;
        break;
    case RANGE32(0xF04, 0xF0C):
        /* Implementation defined */
        break;
    case RANGE32(GIC_DIST_CPENDSGIR0, GIC_DIST_CPENDSGIRN):
        reg_offset = GIC_DIST_REGN(offset, GIC_DIST_CPENDSGIR0);
        reg = gic_dist->sgi_pending_clr[vcpu->vcpu_id][reg_offset];
        break;
    case RANGE32(GIC_DIST_SPENDSGIR0, GIC_DIST_SPENDSGIRN):
        reg_offset = GIC_DIST_REGN(offset, GIC_DIST_SPENDSGIR0);
        reg = gic_dist->sgi_pending_set[vcpu->vcpu_id][reg_offset];
        break;
    case RANGE32(0xF30, 0xFBC):
        /* Reserved */
        break;
    case RANGE32(0xFC0, 0xFFB):
        base_reg = (uintptr_t) & (gic_dist->periph_id[0]);
        reg_ptr = (uint32_t *)(base_reg + (offset - 0xFC0));
        reg = *reg_ptr;
        break;
    default:
        ZF_LOGE("Unknown register offset 0x%x\n", offset);
        err = ignore_fault(fault);
        goto fault_return;
    }
    uint32_t mask = fault_get_data_mask(fault);
    fault_set_data(fault, reg & mask);
    err = advance_fault(fault);

fault_return:
    if (err) {
        return FAULT_ERROR;
    }
    return FAULT_HANDLED;
}

static memory_fault_result_t handle_vgic_dist_write_fault(vm_t *vm, vm_vcpu_t *vcpu, uintptr_t fault_addr,
                                                          size_t fault_length,
                                                          void *cookie)
{
    int err = 0;
    fault_t *fault = vcpu->vcpu_arch.fault;
    struct vgic_dist_device *d = (struct vgic_dist_device *)cookie;
    struct vgic *vgic = vgic_device_get_vgic(d);
    struct gic_dist_map *gic_dist = priv_get_dist(vgic->dist);
    int offset = fault_get_address(fault) - d->pstart;
    int vcpu_id = vcpu->vcpu_id;
    uint32_t reg = 0;
    uint32_t mask = fault_get_data_mask(fault);
    uint32_t reg_offset = 0;
    uint32_t data;
    switch (offset) {
    case RANGE32(GIC_DIST_CTLR, GIC_DIST_CTLR):
        data = fault_get_data(fault);
        if (data == GIC_ENABLED) {
            vgic_dist_enable(gic_dist);
        } else if (data == 0) {
            vgic_dist_disable(gic_dist);
        } else {
            ZF_LOGE("Unknown enable register encoding");
        }
        break;
    case RANGE32(GIC_DIST_TYPER, GIC_DIST_TYPER):
        break;
    case RANGE32(GIC_DIST_IIDR, GIC_DIST_IIDR):
        break;
    case RANGE32(0x00C, 0x01C):
        /* Reserved */
        break;
    case RANGE32(0x020, 0x03C):
        /* Implementation defined */
        break;
    case RANGE32(0x040, 0x07C):
        /* Reserved */
        break;
    case RANGE32(GIC_DIST_IGROUPR0, GIC_DIST_IGROUPR0):
        gic_dist->irq_group0[vcpu->vcpu_id] = fault_emulate(fault, gic_dist->irq_group0[vcpu->vcpu_id]);
        break;
    case RANGE32(GIC_DIST_IGROUPR1, GIC_DIST_IGROUPRN):
        reg_offset = GIC_DIST_REGN(offset, GIC_DIST_IGROUPR1);
        gic_dist->irq_group[reg_offset] = fault_emulate(fault, gic_dist->irq_group[reg_offset]);
        break;
    case RANGE32(GIC_DIST_ISENABLER0, GIC_DIST_ISENABLERN):
        data = fault_get_data(fault);
        /* Mask the data to write */
        data &= mask;
        while (data) {
            int irq;
            irq = CTZ(data);
            data &= ~(1U << irq);
            irq += (offset - GIC_DIST_ISENABLER0) * 8;
            vgic_dist_enable_irq(vgic, gic_dist, vcpu, irq);
        }
        break;
    case RANGE32(GIC_DIST_ICENABLER0, GIC_DIST_ICENABLERN):
        data = fault_get_data(fault);
        /* Mask the data to write */
        data &= mask;
        while (data) {
            int irq;
            irq = CTZ(data);
            data &= ~(1U << irq);
            irq += (offset - GIC_DIST_ICENABLER0) * 8;
            vgic_dist_disable_irq(gic_dist, vcpu, irq);
        }
        break;
    case RANGE32(GIC_DIST_ISPENDR0, GIC_DIST_ISPENDRN):
        data = fault_get_data(fault);
        /* Mask the data to write */
        data &= mask;
        while (data) {
            int irq;
            irq = CTZ(data);
            data &= ~(1U << irq);
            irq += (offset - GIC_DIST_ISPENDR0) * 8;
            vgic_dist_set_pending_irq(vgic, vcpu, irq);
        }
        break;
    case RANGE32(GIC_DIST_ICPENDR0, GIC_DIST_ICPENDRN):
        data = fault_get_data(fault);
        /* Mask the data to write */
        data &= mask;
        while (data) {
            int irq;
            irq = CTZ(data);
            data &= ~(1U << irq);
            irq += (offset - 0x280) * 8;
            vgic_dist_clr_pending_irq(gic_dist, vcpu, irq);
        }
        break;
    case RANGE32(GIC_DIST_ISACTIVER0, GIC_DIST_ISACTIVER0):
        gic_dist->active0[vcpu->vcpu_id] = fault_emulate(fault, gic_dist->active0[vcpu->vcpu_id]);
        break;
    case RANGE32(GIC_DIST_ISACTIVER1, GIC_DIST_ISACTIVERN):
        reg_offset = GIC_DIST_REGN(offset, GIC_DIST_ISACTIVER1);
        gic_dist->active[reg_offset] = fault_emulate(fault, gic_dist->active[reg_offset]);
        break;
    case RANGE32(GIC_DIST_ICACTIVER0, GIC_DIST_ICACTIVER0):
        gic_dist->active_clr0[vcpu->vcpu_id] = fault_emulate(fault, gic_dist->active0[vcpu->vcpu_id]);
        break;
    case RANGE32(GIC_DIST_ICACTIVER1, GIC_DIST_ICACTIVERN):
        reg_offset = GIC_DIST_REGN(offset, GIC_DIST_ICACTIVER1);
        gic_dist->active_clr[reg_offset] = fault_emulate(fault, gic_dist->active_clr[reg_offset]);
        break;
    case RANGE32(GIC_DIST_IPRIORITYR0, GIC_DIST_IPRIORITYRN):
        break;
    case RANGE32(0x7FC, 0x7FC):
        /* Reserved */
        break;
    case RANGE32(GIC_DIST_ITARGETSR0, GIC_DIST_ITARGETSRN):
        break;
    case RANGE32(0xBFC, 0xBFC):
        /* Reserved */
        break;
    case RANGE32(GIC_DIST_ICFGR0, GIC_DIST_ICFGRN):
        /* Not supported */
        break;
    case RANGE32(0xD00, 0xDE4):
        break;
    case RANGE32(0xDE8, 0xEFC):
        /* Reserved [0xDE8 - 0xE00) */
        /* GIC_DIST_NSACR [0xE00 - 0xF00) - Not supported */
        break;
    case RANGE32(GIC_DIST_SGIR, GIC_DIST_SGIR):
        data = fault_get_data(fault);
        int mode = (data & GIC_DIST_SGI_TARGET_LIST_FILTER_MASK) >> GIC_DIST_SGI_TARGET_LIST_FILTER_SHIFT;
        int virq = (data & GIC_DIST_SGI_INTID_MASK);
        uint16_t target_list = 0;
        switch (mode) {
        case GIC_DIST_SGI_TARGET_LIST_SPEC:
            /* Forward virq to vcpus specified in CPUTargetList */
            target_list = (data & GIC_DIST_SGI_CPU_TARGET_LIST_MASK) >> GIC_DIST_SGI_CPU_TARGET_LIST_SHIFT;
            break;
        case GIC_DIST_SGI_TARGET_LIST_OTHERS:
            /* Forward virq to all vcpus but the requesting vcpu */
            target_list = (1 << vcpu->vm->num_vcpus) - 1;
            target_list = target_list & ~(1 << vcpu->vcpu_id);
            break;
        case GIC_DIST_SGI_TARGET_SELF:
            /* Forward to virq to only the requesting vcpu */
            target_list = (1 << vcpu->vcpu_id);
            break;
        default:
            ZF_LOGE("Unknow SGIR Target List Filter mode");
            goto ignore_fault;
        }
        for (int i = 0; i < vcpu->vm->num_vcpus; i++) {
            vm_vcpu_t *target_vcpu = vcpu->vm->vcpus[i];
            if (!(target_list & (1 << i)) || !is_vcpu_online(target_vcpu)) {
                continue;
            }
            vm_inject_irq(target_vcpu, virq);
        }
        break;
    case RANGE32(0xF04, 0xF0C):
        break;
    case RANGE32(GIC_DIST_CPENDSGIR0, GIC_DIST_SPENDSGIRN):
        assert(!"vgic SGI reg not implemented!\n");
        break;
    case RANGE32(0xF30, 0xFBC):
        /* Reserved */
        break;
    case RANGE32(0xFC0, 0xFFB):
        break;
    default:
        ZF_LOGE("Unknown register offset 0x%x\n", offset);
    }
ignore_fault:
    err = ignore_fault(fault);
    if (err) {
        return FAULT_ERROR;
    }
    return FAULT_HANDLED;
}

static memory_fault_result_t handle_vgic_dist_fault(vm_t *vm, vm_vcpu_t *vcpu, uintptr_t fault_addr,
                                                    size_t fault_length,
                                                    void *cookie)
{
    if (fault_is_read(vcpu->vcpu_arch.fault)) {
        return handle_vgic_dist_read_fault(vm, vcpu, fault_addr, fault_length, cookie);
    }
    return handle_vgic_dist_write_fault(vm, vcpu, fault_addr, fault_length, cookie);
}
