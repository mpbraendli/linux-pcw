/*
 * PrecisionWave FFT accelerator
 *
 * Copyright 2015 Analog Devices Inc.
 * Copyright 2024 Matthias P. Braendli
 *
 * Licensed under the GPL-2.
 */
 #include <linux/kernel.h>
 #include <linux/slab.h>
 #include <linux/module.h>
 #include <linux/string.h>
 #include <linux/dmaengine.h>

 #include <linux/iio/iio.h>
 #include <linux/iio/sysfs.h>
 #include <linux/iio/buffer.h>
 #include <linux/iio/buffer-dma.h>
 #include <linux/iio/buffer-dmaengine.h>
 #include <linux/of_device.h>
 #include <linux/of_platform.h>


#define DRIVER_NAME "fft_accelerator"

static struct iio_chan_spec_ext_info fft_accelerator_ext_info[] = {
	{ },
};

/* we need that dummy function to get a valid iio_info object */
static int fft_accelerator_read_raw_dummy(struct iio_dev *indio_dev,
			   const struct iio_chan_spec *chan,
			   int *val, int *val2, long info)
{
	return -EINVAL;
}

static const struct iio_chan_spec fft_accelerator_chan_spec[] = {
	{
		.type = IIO_VOLTAGE,
		.extend_name = "out",
		.output = 1,
		.indexed = 1,
		.channel = 0,
		.address = 0,
		.info_mask_separate = BIT(IIO_CHAN_INFO_HARDWAREGAIN),
		.scan_index = 0,
		.ext_info = fft_accelerator_ext_info,
	},
	{
		.type = IIO_VOLTAGE,
		.extend_name = "in",
		.output = 0,
		.indexed = 1,
		.channel = 1,
		.address = 1,
		.info_mask_separate = BIT(IIO_CHAN_INFO_HARDWAREGAIN),
		.scan_index = 1,
		.ext_info = fft_accelerator_ext_info,
	}
};

static int fft_accelerator_submit_block(
		struct iio_dma_buffer_queue *queue,
		struct iio_dma_buffer_block *block)
{
	struct iio_dev *indio_dev = queue->driver_data;

	if (indio_dev->direction == IIO_DEVICE_DIRECTION_IN) {
		printk("FFT Accelerator submit IN\n");
		block->block.bytes_used = block->block.size;
		iio_dmaengine_buffer_submit_block(queue, block, DMA_DEV_TO_MEM);
	} else {
		printk("FFT Accelerator submit OUT\n");
		iio_dmaengine_buffer_submit_block(queue, block, DMA_MEM_TO_DEV);
	}

	return 0;
}

static const struct iio_dma_buffer_ops fft_accelerator_dma_buffer_ops = {
	.submit = fft_accelerator_submit_block,
	.abort = iio_dmaengine_buffer_abort,
};

enum attributes {
	FA_ATTR_DUMMY,
};

struct fft_accelerator_state{
	u32 dummyattribute;
};

static ssize_t fft_accelerator_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t len)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct iio_dev_attr *this_attr = to_iio_dev_attr(attr);
	struct fft_accelerator_state *st = iio_priv(indio_dev);
	long val;
	int ret = 0;

	mutex_lock(&indio_dev->mlock);
	switch ((u32)this_attr->address){
	case FA_ATTR_DUMMY:
		st->dummyattribute = val;
		break;
	default:
		ret = -ENODEV;
	}
	mutex_unlock(&indio_dev->mlock);

	return ret ? ret : len;
}

static ssize_t fft_accelerator_show(struct device *dev,
			struct device_attribute *attr,
			char *buf)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct iio_dev_attr *this_attr = to_iio_dev_attr(attr);
	struct fft_accelerator_state *st = iio_priv(indio_dev);
	u32 val = 0;
	int ret = 0;

	mutex_lock(&indio_dev->mlock);
	switch ((u32)this_attr->address){
	case FA_ATTR_DUMMY:
		val = st->dummyattribute;
		break;
	default:
		ret = -ENODEV;
	}
	mutex_unlock(&indio_dev->mlock);

	if(ret==0){
		ret = sprintf(buf, "%d\n", val);
	}
	return ret;
}

static IIO_DEVICE_ATTR(Dummy, S_IRUGO | S_IWUSR,
			fft_accelerator_show,
			fft_accelerator_store,
			FA_ATTR_DUMMY);

static struct attribute *fft_accelerator_attributes[] = {
	&iio_dev_attr_Dummy.dev_attr.attr,
	NULL
};

static const struct attribute_group fft_accelerator_attribute_group = {
	.attrs = fft_accelerator_attributes,
};

static const struct iio_info fft_accelerator_info = {
	.read_raw = &fft_accelerator_read_raw_dummy,
	.attrs = &fft_accelerator_attribute_group,
};

static const struct of_device_id fft_accelerator_of_match[] = {
	{ .compatible = DRIVER_NAME, },
	{ },
};
MODULE_DEVICE_TABLE(of, fft_accelerator_of_match);

static int fft_accelerator_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;	// for devicetree parsing
	struct iio_dev *indio_dev;
	struct fft_accelerator_state *st;
	struct iio_buffer *buffer_out, *buffer_in;
	const struct of_device_id *id;
	int ret;

	printk("probing FFT Accelerator...\n");
	id = of_match_device(fft_accelerator_of_match, &pdev->dev);
	if (!id){
		printk("\nFFT Accelerator: -ENODEV\n");
		return -ENODEV;
	}

	indio_dev = devm_iio_device_alloc(&pdev->dev, sizeof(*st));
	if (!indio_dev) {
		printk("\nFFT Accelerator: -ENOMEM\n");
		return -ENOMEM;
	}

	st = iio_priv(indio_dev);
	indio_dev->name = np->name;
	indio_dev->modes = INDIO_DIRECT_MODE | INDIO_BUFFER_HARDWARE;
	indio_dev->info = &fft_accelerator_info;
	indio_dev->channels = fft_accelerator_chan_spec,
	indio_dev->num_channels = ARRAY_SIZE(fft_accelerator_chan_spec);

	buffer_in = devm_iio_dmaengine_buffer_alloc(&pdev->dev,
			"in", &fft_accelerator_dma_buffer_ops, NULL);
	if (IS_ERR(buffer_in)) {
		printk("FFT Accelerator buffer_in dmaengine_buffer_alloc IS ERR!\n");
		return PTR_ERR(buffer_in);
	}
	iio_device_attach_buffer(indio_dev, buffer_in);

	buffer_out = devm_iio_dmaengine_buffer_alloc(&pdev->dev,
			"out", &fft_accelerator_dma_buffer_ops, NULL);
	if (IS_ERR(buffer_out)) {
		printk("FFT Accelerator buffer_out dmaengine_buffer_alloc IS ERR!\n");
		return PTR_ERR(buffer_out);
	}
	iio_device_attach_buffer(indio_dev, buffer_out);

	ret = devm_iio_device_register(&pdev->dev, indio_dev);
	if (ret < 0) {
		dev_err(&pdev->dev, "devm_iio_device_register failed");
		return ret;
	}

	platform_set_drvdata(pdev, indio_dev);

	printk("probing FFT Accelerator DONE!\n");
	return 0;
}

static struct platform_driver fft_accelerator_driver = {
	.driver = {
		.name = DRIVER_NAME,
		.owner = THIS_MODULE,
		.of_match_table = fft_accelerator_of_match,
	},
	.probe = fft_accelerator_probe,
};
module_platform_driver(fft_accelerator_driver);

MODULE_AUTHOR("Matthias P. Braendli <matthias.braendli@mpb.li>");
MODULE_DESCRIPTION(DRIVER_NAME);
MODULE_LICENSE("GPL v2");
