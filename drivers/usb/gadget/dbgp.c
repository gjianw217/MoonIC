/*
 * dbgp.c -- EHCI Debug Port device gadget
 *
 * Copyright (C) 2010 Stephane Duverger
 *
 * Released under the GPLv2.
 *
 */

/* verbose messages */
#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/usb/ch9.h>
#include <linux/usb/gadget.h>

/* See comments in "zero.c" */
//#include "epautoconf.c"
/*********************************************************************/
/************************************************************************/

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/device.h>

#include <linux/ctype.h>
#include <linux/string.h>

#include <linux/usb/ch9.h>
//#include <linux/usb/gadget.h>

#include "gadget_chips.h"


/* we must assign addresses for configurable endpoints (like net2280) */
static unsigned epnum;

// #define MANY_ENDPOINTS
#ifdef MANY_ENDPOINTS
/* more than 15 configurable endpoints */
static unsigned in_epnum;
#endif


/*
 * This should work with endpoints from controller drivers sharing the
 * same endpoint naming convention.  By example:
 *
 *	- ep1, ep2, ... address is fixed, not direction or type
 *	- ep1in, ep2out, ... address and direction are fixed, not type
 *	- ep1-bulk, ep2-bulk, ... address and type are fixed, not direction
 *	- ep1in-bulk, ep2out-iso, ... all three are fixed
 *	- ep-* ... no functionality restrictions
 *
 * Type suffixes are "-bulk", "-iso", or "-int".  Numbers are decimal.
 * Less common restrictions are implied by gadget_is_*().
 *
 * NOTE:  each endpoint is unidirectional, as specified by its USB
 * descriptor; and isn't specific to a configuration or altsetting.
 */
static int
ep_matches (
	struct usb_gadget		*gadget,
	struct usb_ep			*ep,
	struct usb_endpoint_descriptor	*desc,
	struct usb_ss_ep_comp_descriptor *ep_comp
)
{
	u8		type;
	const char	*tmp;
	u16		max;

	int		num_req_streams = 0;

	/* endpoint already claimed? */
	if (NULL != ep->driver_data)
		return 0;

	/* only support ep0 for portable CONTROL traffic */
	type = desc->bmAttributes & USB_ENDPOINT_XFERTYPE_MASK;
	if (USB_ENDPOINT_XFER_CONTROL == type)
		return 0;

	/* some other naming convention */
	if ('e' != ep->name[0])
		return 0;

	/* type-restriction:  "-iso", "-bulk", or "-int".
	 * direction-restriction:  "in", "out".
	 */
	if ('-' != ep->name[2]) {
		tmp = strrchr (ep->name, '-');
		if (tmp) {
			switch (type) {
			case USB_ENDPOINT_XFER_INT:
				/* bulk endpoints handle interrupt transfers,
				 * except the toggle-quirky iso-synch kind
				 */
				if ('s' == tmp[2])	// == "-iso"
					return 0;
				/* for now, avoid PXA "interrupt-in";
				 * it's documented as never using DATA1.
				 */
				if (gadget_is_pxa (gadget)
						&& 'i' == tmp [1])
					return 0;
				break;
			case USB_ENDPOINT_XFER_BULK:
				if ('b' != tmp[1])	// != "-bulk"
					return 0;
				break;
			case USB_ENDPOINT_XFER_ISOC:
				if ('s' != tmp[2])	// != "-iso"
					return 0;
			}
		} else {
			tmp = ep->name + strlen (ep->name);
		}

		/* direction-restriction:  "..in-..", "out-.." */
		tmp--;
		if (!isdigit (*tmp)) {
			if (desc->bEndpointAddress & USB_DIR_IN) {
				if ('n' != *tmp)
					return 0;
			} else {
				if ('t' != *tmp)
					return 0;
			}
		}
	}

	/*
	 * Get the number of required streams from the EP companion
	 * descriptor and see if the EP matches it
	 */
	if (usb_endpoint_xfer_bulk(desc)) {
		if (ep_comp) {
			num_req_streams = ep_comp->bmAttributes & 0x1f;
			if (num_req_streams > ep->max_streams)
				return 0;
			/* Update the ep_comp descriptor if needed */
			if (num_req_streams != ep->max_streams)
				ep_comp->bmAttributes = ep->max_streams;
		}

	}

	/*
	 * If the protocol driver hasn't yet decided on wMaxPacketSize
	 * and wants to know the maximum possible, provide the info.
	 */
	if (desc->wMaxPacketSize == 0)
		desc->wMaxPacketSize = cpu_to_le16(ep->maxpacket);

	/* endpoint maxpacket size is an input parameter, except for bulk
	 * where it's an output parameter representing the full speed limit.
	 * the usb spec fixes high speed bulk maxpacket at 512 bytes.
	 */
	max = 0x7ff & le16_to_cpu(desc->wMaxPacketSize);
	switch (type) {
	case USB_ENDPOINT_XFER_INT:
		/* INT:  limit 64 bytes full speed, 1024 high/super speed */
		if (!gadget->is_dualspeed && max > 64)
			return 0;
		/* FALLTHROUGH */

	case USB_ENDPOINT_XFER_ISOC:
		/* ISO:  limit 1023 bytes full speed, 1024 high/super speed */
		if (ep->maxpacket < max)
			return 0;
		if (!gadget->is_dualspeed && max > 1023)
			return 0;

		/* BOTH:  "high bandwidth" works only at high speed */
		if ((desc->wMaxPacketSize & cpu_to_le16(3<<11))) {
			if (!gadget->is_dualspeed)
				return 0;
			/* configure your hardware with enough buffering!! */
		}
		break;
	}

	/* MATCH!! */

	/* report address */
	desc->bEndpointAddress &= USB_DIR_IN;
	if (isdigit (ep->name [2])) {
		u8	num = simple_strtoul (&ep->name [2], NULL, 10);
		desc->bEndpointAddress |= num;
#ifdef	MANY_ENDPOINTS
	} else if (desc->bEndpointAddress & USB_DIR_IN) {
		if (++in_epnum > 15)
			return 0;
		desc->bEndpointAddress = USB_DIR_IN | in_epnum;
#endif
	} else {
		if (++epnum > 15)
			return 0;
		desc->bEndpointAddress |= epnum;
	}

	/* report (variable) full speed bulk maxpacket */
	if ((USB_ENDPOINT_XFER_BULK == type) && !ep_comp) {
		int size = ep->maxpacket;

		/* min() doesn't work on bitfields with gcc-3.5 */
		if (size > 64)
			size = 64;
		desc->wMaxPacketSize = cpu_to_le16(size);
	}
	ep->address = desc->bEndpointAddress;
	return 1;
}

static struct usb_ep *
find_ep (struct usb_gadget *gadget, const char *name)
{
	struct usb_ep	*ep;

	list_for_each_entry (ep, &gadget->ep_list, ep_list) {
		if (0 == strcmp (ep->name, name))
			return ep;
	}
	return NULL;
}

/**
 * usb_ep_autoconfig_ss() - choose an endpoint matching the ep
 * descriptor and ep companion descriptor
 * @gadget: The device to which the endpoint must belong.
 * @desc: Endpoint descriptor, with endpoint direction and transfer mode
 *    initialized.  For periodic transfers, the maximum packet
 *    size must also be initialized.  This is modified on
 *    success.
 * @ep_comp: Endpoint companion descriptor, with the required
 *    number of streams. Will be modified when the chosen EP
 *    supports a different number of streams.
 *
 * This routine replaces the usb_ep_autoconfig when needed
 * superspeed enhancments. If such enhancemnets are required,
 * the FD should call usb_ep_autoconfig_ss directly and provide
 * the additional ep_comp parameter.
 *
 * By choosing an endpoint to use with the specified descriptor,
 * this routine simplifies writing gadget drivers that work with
 * multiple USB device controllers.  The endpoint would be
 * passed later to usb_ep_enable(), along with some descriptor.
 *
 * That second descriptor won't always be the same as the first one.
 * For example, isochronous endpoints can be autoconfigured for high
 * bandwidth, and then used in several lower bandwidth altsettings.
 * Also, high and full speed descriptors will be different.
 *
 * Be sure to examine and test the results of autoconfiguration
 * on your hardware.  This code may not make the best choices
 * about how to use the USB controller, and it can't know all
 * the restrictions that may apply. Some combinations of driver
 * and hardware won't be able to autoconfigure.
 *
 * On success, this returns an un-claimed usb_ep, and modifies the endpoint
 * descriptor bEndpointAddress.  For bulk endpoints, the wMaxPacket value
 * is initialized as if the endpoint were used at full speed and
 * the bmAttribute field in the ep companion descriptor is
 * updated with the assigned number of streams if it is
 * different from the original value. To prevent the endpoint
 * from being returned by a later autoconfig call, claim it by
 * assigning ep->driver_data to some non-null value.
 *
 * On failure, this returns a null endpoint descriptor.
 */
static struct usb_ep *usb_ep_autoconfig_ss2(
	struct usb_gadget		*gadget,
	struct usb_endpoint_descriptor	*desc,
	struct usb_ss_ep_comp_descriptor *ep_comp
)
{
	struct usb_ep	*ep;
	u8		type;

	type = desc->bmAttributes & USB_ENDPOINT_XFERTYPE_MASK;

	/* First, apply chip-specific "best usage" knowledge.
	 * This might make a good usb_gadget_ops hook ...
	 */
	if (gadget_is_net2280 (gadget) && type == USB_ENDPOINT_XFER_INT) {
		/* ep-e, ep-f are PIO with only 64 byte fifos */
		ep = find_ep (gadget, "ep-e");
		if (ep && ep_matches(gadget, ep, desc, ep_comp))
			return ep;
		ep = find_ep (gadget, "ep-f");
		if (ep && ep_matches(gadget, ep, desc, ep_comp))
			return ep;

	} else if (gadget_is_goku (gadget)) {
		if (USB_ENDPOINT_XFER_INT == type) {
			/* single buffering is enough */
			ep = find_ep(gadget, "ep3-bulk");
			if (ep && ep_matches(gadget, ep, desc, ep_comp))
				return ep;
		} else if (USB_ENDPOINT_XFER_BULK == type
				&& (USB_DIR_IN & desc->bEndpointAddress)) {
			/* DMA may be available */
			ep = find_ep(gadget, "ep2-bulk");
			if (ep && ep_matches(gadget, ep, desc,
					      ep_comp))
				return ep;
		}

#ifdef CONFIG_BLACKFIN
	} else if (gadget_is_musbhdrc(gadget)) {
		if ((USB_ENDPOINT_XFER_BULK == type) ||
		    (USB_ENDPOINT_XFER_ISOC == type)) {
			if (USB_DIR_IN & desc->bEndpointAddress)
				ep = find_ep (gadget, "ep5in");
			else
				ep = find_ep (gadget, "ep6out");
		} else if (USB_ENDPOINT_XFER_INT == type) {
			if (USB_DIR_IN & desc->bEndpointAddress)
				ep = find_ep(gadget, "ep1in");
			else
				ep = find_ep(gadget, "ep2out");
		} else
			ep = NULL;
		if (ep && ep_matches(gadget, ep, desc, ep_comp))
			return ep;
#endif
	}

	/* Second, look at endpoints until an unclaimed one looks usable */
	list_for_each_entry (ep, &gadget->ep_list, ep_list) {
		if (ep_matches(gadget, ep, desc, ep_comp))
			return ep;
	}

	/* Fail */
	return NULL;
}

/**
 * usb_ep_autoconfig() - choose an endpoint matching the
 * descriptor
 * @gadget: The device to which the endpoint must belong.
 * @desc: Endpoint descriptor, with endpoint direction and transfer mode
 *	initialized.  For periodic transfers, the maximum packet
 *	size must also be initialized.  This is modified on success.
 *
 * By choosing an endpoint to use with the specified descriptor, this
 * routine simplifies writing gadget drivers that work with multiple
 * USB device controllers.  The endpoint would be passed later to
 * usb_ep_enable(), along with some descriptor.
 *
 * That second descriptor won't always be the same as the first one.
 * For example, isochronous endpoints can be autoconfigured for high
 * bandwidth, and then used in several lower bandwidth altsettings.
 * Also, high and full speed descriptors will be different.
 *
 * Be sure to examine and test the results of autoconfiguration on your
 * hardware.  This code may not make the best choices about how to use the
 * USB controller, and it can't know all the restrictions that may apply.
 * Some combinations of driver and hardware won't be able to autoconfigure.
 *
 * On success, this returns an un-claimed usb_ep, and modifies the endpoint
 * descriptor bEndpointAddress.  For bulk endpoints, the wMaxPacket value
 * is initialized as if the endpoint were used at full speed.  To prevent
 * the endpoint from being returned by a later autoconfig call, claim it
 * by assigning ep->driver_data to some non-null value.
 *
 * On failure, this returns a null endpoint descriptor.
 */
static struct usb_ep *usb_ep_autoconfig2(
	struct usb_gadget		*gadget,
	struct usb_endpoint_descriptor	*desc
)
{
	return usb_ep_autoconfig_ss2(gadget, desc, NULL);
}


/**
 * usb_ep_autoconfig_reset - reset endpoint autoconfig state
 * @gadget: device for which autoconfig state will be reset
 *
 * Use this for devices where one configuration may need to assign
 * endpoint resources very differently from the next one.  It clears
 * state such as ep->driver_data and the record of assigned endpoints
 * used by usb_ep_autoconfig().
 */
static void usb_ep_autoconfig_reset2 (struct usb_gadget *gadget)
{
	struct usb_ep	*ep;

	list_for_each_entry (ep, &gadget->ep_list, ep_list) {
		ep->driver_data = NULL;
		ep->desc = NULL;
	}
#ifdef	MANY_ENDPOINTS
	in_epnum = 0;
#endif
	epnum = 0;
}






/**********************************************************************/

/**********************************************************************/
#ifdef CONFIG_USB_G_DBGP_SERIAL
#include "u_serial.c"
#endif

#define DRIVER_VENDOR_ID	0x0525 /* NetChip */
#define DRIVER_PRODUCT_ID	0xc0de /* undefined */

#define USB_DEBUG_MAX_PACKET_SIZE     8
#define DBGP_REQ_EP0_LEN              128
#define DBGP_REQ_LEN                  512

static struct dbgp {
	struct usb_gadget  *gadget;
	struct usb_request *req;
	struct usb_ep      *i_ep;
	struct usb_ep      *o_ep;
#ifdef CONFIG_USB_G_DBGP_SERIAL
	struct gserial     *serial;
#endif
} dbgp;

static struct usb_device_descriptor device_desc = {
	.bLength = sizeof device_desc,
	.bDescriptorType = USB_DT_DEVICE,
	.bcdUSB = __constant_cpu_to_le16(0x0200),
	.bDeviceClass =	USB_CLASS_VENDOR_SPEC,
	.idVendor = __constant_cpu_to_le16(DRIVER_VENDOR_ID),
	.idProduct = __constant_cpu_to_le16(DRIVER_PRODUCT_ID),
	.bNumConfigurations = 1,
};

static struct usb_debug_descriptor dbg_desc = {
	.bLength = sizeof dbg_desc,
	.bDescriptorType = USB_DT_DEBUG,
};

static struct usb_endpoint_descriptor i_desc = {
	.bLength = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType = USB_DT_ENDPOINT,
	.bmAttributes = USB_ENDPOINT_XFER_BULK,
	.bEndpointAddress = USB_DIR_IN,
};

static struct usb_endpoint_descriptor o_desc = {
	.bLength = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType = USB_DT_ENDPOINT,
	.bmAttributes = USB_ENDPOINT_XFER_BULK,
	.bEndpointAddress = USB_DIR_OUT,
};

#ifdef CONFIG_USB_G_DBGP_PRINTK
static int dbgp_consume(char *buf, unsigned len)
{
	char c;

	if (!len)
		return 0;

	c = buf[len-1];
	if (c != 0)
		buf[len-1] = 0;

	printk(KERN_NOTICE "%s%c", buf, c);
	return 0;
}

static void __disable_ep(struct usb_ep *ep)
{
	if (ep && ep->driver_data == dbgp.gadget) {
		usb_ep_disable(ep);
		ep->driver_data = NULL;
	}
}

static void dbgp_disable_ep(void)
{
	__disable_ep(dbgp.i_ep);
	__disable_ep(dbgp.o_ep);
}

static void dbgp_complete(struct usb_ep *ep, struct usb_request *req)
{
	int stp;
	int err = 0;
	int status = req->status;

	if (ep == dbgp.i_ep) {
		stp = 1;
		goto fail;
	}

	if (status != 0) {
		stp = 2;
		goto release_req;
	}

	dbgp_consume(req->buf, req->actual);

	req->length = DBGP_REQ_LEN;
	err = usb_ep_queue(ep, req, GFP_ATOMIC);
	if (err < 0) {
		stp = 3;
		goto release_req;
	}

	return;

release_req:
	kfree(req->buf);
	usb_ep_free_request(dbgp.o_ep, req);
	dbgp_disable_ep();
fail:
	dev_dbg(&dbgp.gadget->dev,
		"complete: failure (%d:%d) ==> %d\n", stp, err, status);
}

static int dbgp_enable_ep_req(struct usb_ep *ep)
{
	int err, stp;
	struct usb_request *req;

	req = usb_ep_alloc_request(ep, GFP_KERNEL);
	if (!req) {
		err = -ENOMEM;
		stp = 1;
		goto fail_1;
	}

	req->buf = kmalloc(DBGP_REQ_LEN, GFP_KERNEL);
	if (!req->buf) {
		err = -ENOMEM;
		stp = 2;
		goto fail_2;
	}

	req->complete = dbgp_complete;
	req->length = DBGP_REQ_LEN;
	err = usb_ep_queue(ep, req, GFP_ATOMIC);
	if (err < 0) {
		stp = 3;
		goto fail_3;
	}

	return 0;

fail_3:
	kfree(req->buf);
fail_2:
	usb_ep_free_request(dbgp.o_ep, req);
fail_1:
	dev_dbg(&dbgp.gadget->dev,
		"enable ep req: failure (%d:%d)\n", stp, err);
	return err;
}

static int __enable_ep(struct usb_ep *ep, struct usb_endpoint_descriptor *desc)
{
	int err;
	ep->desc = desc;
	err = usb_ep_enable(ep);
	ep->driver_data = dbgp.gadget;
	return err;
}

static int dbgp_enable_ep(void)
{
	int err, stp;

	err = __enable_ep(dbgp.i_ep, &i_desc);
	if (err < 0) {
		stp = 1;
		goto fail_1;
	}

	err = __enable_ep(dbgp.o_ep, &o_desc);
	if (err < 0) {
		stp = 2;
		goto fail_2;
	}

	err = dbgp_enable_ep_req(dbgp.o_ep);
	if (err < 0) {
		stp = 3;
		goto fail_3;
	}

	return 0;

fail_3:
	__disable_ep(dbgp.o_ep);
fail_2:
	__disable_ep(dbgp.i_ep);
fail_1:
	dev_dbg(&dbgp.gadget->dev, "enable ep: failure (%d:%d)\n", stp, err);
	return err;
}
#endif

static void dbgp_disconnect(struct usb_gadget *gadget)
{
#ifdef CONFIG_USB_G_DBGP_PRINTK
	dbgp_disable_ep();
#else
	gserial_disconnect(dbgp.serial);
#endif
}

static void dbgp_unbind(struct usb_gadget *gadget)
{
#ifdef CONFIG_USB_G_DBGP_SERIAL
	kfree(dbgp.serial);
#endif
	if (dbgp.req) {
		kfree(dbgp.req->buf);
		usb_ep_free_request(gadget->ep0, dbgp.req);
	}

	gadget->ep0->driver_data = NULL;
}

static int __init dbgp_configure_endpoints(struct usb_gadget *gadget)
{
	int stp;

	usb_ep_autoconfig_reset2(gadget);

	dbgp.i_ep = usb_ep_autoconfig2(gadget, &i_desc);
	if (!dbgp.i_ep) {
		stp = 1;
		goto fail_1;
	}

	dbgp.i_ep->driver_data = gadget;
	i_desc.wMaxPacketSize =
		__constant_cpu_to_le16(USB_DEBUG_MAX_PACKET_SIZE);

	dbgp.o_ep = usb_ep_autoconfig2(gadget, &o_desc);
	if (!dbgp.o_ep) {
		dbgp.i_ep->driver_data = NULL;
		stp = 2;
		goto fail_2;
	}

	dbgp.o_ep->driver_data = gadget;
	o_desc.wMaxPacketSize =
		__constant_cpu_to_le16(USB_DEBUG_MAX_PACKET_SIZE);

	dbg_desc.bDebugInEndpoint = i_desc.bEndpointAddress;
	dbg_desc.bDebugOutEndpoint = o_desc.bEndpointAddress;

#ifdef CONFIG_USB_G_DBGP_SERIAL
	dbgp.serial->in = dbgp.i_ep;
	dbgp.serial->out = dbgp.o_ep;

	dbgp.serial->in->desc = &i_desc;
	dbgp.serial->out->desc = &o_desc;

	if (gserial_setup(gadget, 1) < 0) {
		stp = 3;
		goto fail_3;
	}

	return 0;

fail_3:
	dbgp.o_ep->driver_data = NULL;
#else
	return 0;
#endif
fail_2:
	dbgp.i_ep->driver_data = NULL;
fail_1:
	dev_dbg(&dbgp.gadget->dev, "ep config: failure (%d)\n", stp);
	return -ENODEV;
}

static int __init dbgp_bind(struct usb_gadget *gadget)
{
	int err, stp;

	dbgp.gadget = gadget;

	dbgp.req = usb_ep_alloc_request(gadget->ep0, GFP_KERNEL);
	if (!dbgp.req) {
		err = -ENOMEM;
		stp = 1;
		goto fail;
	}

	dbgp.req->buf = kmalloc(DBGP_REQ_EP0_LEN, GFP_KERNEL);
	if (!dbgp.req->buf) {
		err = -ENOMEM;
		stp = 2;
		goto fail;
	}

	dbgp.req->length = DBGP_REQ_EP0_LEN;
	gadget->ep0->driver_data = gadget;

#ifdef CONFIG_USB_G_DBGP_SERIAL
	dbgp.serial = kzalloc(sizeof(struct gserial), GFP_KERNEL);
	if (!dbgp.serial) {
		stp = 3;
		err = -ENOMEM;
		goto fail;
	}
#endif
	err = dbgp_configure_endpoints(gadget);
	if (err < 0) {
		stp = 4;
		goto fail;
	}

	dev_dbg(&dbgp.gadget->dev, "bind: success\n");
	return 0;

fail:
	dev_dbg(&gadget->dev, "bind: failure (%d:%d)\n", stp, err);
	dbgp_unbind(gadget);
	return err;
}

static void dbgp_setup_complete(struct usb_ep *ep,
				struct usb_request *req)
{
	dev_dbg(&dbgp.gadget->dev, "setup complete: %d, %d/%d\n",
		req->status, req->actual, req->length);
}

static int dbgp_setup(struct usb_gadget *gadget,
		      const struct usb_ctrlrequest *ctrl)
{
	struct usb_request *req = dbgp.req;
	u8 request = ctrl->bRequest;
	u16 value = le16_to_cpu(ctrl->wValue);
	u16 length = le16_to_cpu(ctrl->wLength);
	int err = -EOPNOTSUPP;
	void *data = NULL;
	u16 len = 0;

	gadget->ep0->driver_data = gadget;

	if (request == USB_REQ_GET_DESCRIPTOR) {
		switch (value>>8) {
		case USB_DT_DEVICE:
			dev_dbg(&dbgp.gadget->dev, "setup: desc device\n");
			len = sizeof device_desc;
			data = &device_desc;
			device_desc.bMaxPacketSize0 = gadget->ep0->maxpacket;
			break;
		case USB_DT_DEBUG:
			dev_dbg(&dbgp.gadget->dev, "setup: desc debug\n");
			len = sizeof dbg_desc;
			data = &dbg_desc;
			break;
		default:
			goto fail;
		}
		err = 0;
	} else if (request == USB_REQ_SET_FEATURE &&
		   value == USB_DEVICE_DEBUG_MODE) {
		dev_dbg(&dbgp.gadget->dev, "setup: feat debug\n");
#ifdef CONFIG_USB_G_DBGP_PRINTK
		err = dbgp_enable_ep();
#else
		err = gserial_connect(dbgp.serial, 0);
#endif
		if (err < 0)
			goto fail;
	} else
		goto fail;

	req->length = min(length, len);
	req->zero = len < req->length;
	if (data && req->length)
		memcpy(req->buf, data, req->length);

	req->complete = dbgp_setup_complete;
	return usb_ep_queue(gadget->ep0, req, GFP_ATOMIC);

fail:
	dev_dbg(&dbgp.gadget->dev,
		"setup: failure req %x v %x\n", request, value);
	return err;
}

static struct usb_gadget_driver dbgp_driver = {
	.function = "dbgp",
	.speed = USB_SPEED_HIGH,
	.unbind = dbgp_unbind,
	.setup = dbgp_setup,
	.disconnect = dbgp_disconnect,
	.driver	= {
		.owner = THIS_MODULE,
		.name = "dbgp"
	},
};

static int __init dbgp_init(void)
{
	return usb_gadget_probe_driver(&dbgp_driver, dbgp_bind);
}

static void __exit dbgp_exit(void)
{
	usb_gadget_unregister_driver(&dbgp_driver);
#ifdef CONFIG_USB_G_DBGP_SERIAL
	gserial_cleanup();
#endif
}

MODULE_AUTHOR("Stephane Duverger");
MODULE_LICENSE("GPL");
module_init(dbgp_init);
module_exit(dbgp_exit);
