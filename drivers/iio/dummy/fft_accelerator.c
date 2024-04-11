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

static const struct iio_info fft_accelerator_empty_info = { };

static const struct iio_chan_spec fft_accelerator_chan_spec[] = {
	{
		.type = IIO_VOLTAGE,
		.extend_name = "TX",
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
		.extend_name = "RX",
		.output = 0,
		.indexed = 1,
		.channel = 1,
		.address = 1,
		.info_mask_separate = BIT(IIO_CHAN_INFO_HARDWAREGAIN),
		.scan_index = 1,
		.ext_info = fft_accelerator_ext_info,
	}
};

static int fft_accelerator_tx_preenable(struct iio_dev *indio_dev)
{
	printk("FFT Accelerator tx preenable\n");
	return 0;
}

static int fft_accelerator_tx_postdisable(struct iio_dev *indio_dev)
{
	printk("FFT Accelerator tx postdisable\n");
	return 0;
}

static const struct iio_buffer_setup_ops fft_accelerator_tx_setup_ops = {
	.preenable = fft_accelerator_tx_preenable,
	.postdisable = fft_accelerator_tx_postdisable,
};

static int fft_accelerator_submit_block(
		struct iio_dma_buffer_queue *queue,
		struct iio_dma_buffer_block *block)
{
	struct iio_dev *indio_dev = queue->driver_data;

	if (indio_dev->direction == IIO_DEVICE_DIRECTION_IN) {
		block->block.bytes_used = block->block.size;
		iio_dmaengine_buffer_submit_block(queue, block, DMA_DEV_TO_MEM);
	} else {
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
	// .driver_module = THIS_MODULE,
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
	struct iio_dev *indio_dev, *indio_dev_tx, *indio_dev_rx;
	struct fft_accelerator_state *st;
	struct iio_buffer *buffer_tx, *buffer_rx;
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

	indio_dev_tx = devm_iio_device_alloc(&pdev->dev, 0);
	if (!indio_dev_tx) {
		printk("\nFFT Accelerator tx: -ENOMEM\n");
		return -ENOMEM;
	}

	indio_dev_rx = devm_iio_device_alloc(&pdev->dev, 0);
	if (!indio_dev_rx) {
		printk("\nFFT Accelerator rx: -ENOMEM\n");
		return -ENOMEM;
	}

	printk("FFT Accelerator iio_priv\n");
	st = iio_priv(indio_dev);

	indio_dev->name = np->name;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->info = &fft_accelerator_info;
	indio_dev->channels = fft_accelerator_chan_spec,
	indio_dev->num_channels = ARRAY_SIZE(fft_accelerator_chan_spec);

	printk("FFT Accelerator devm_iio_device_register\n");
	ret = devm_iio_device_register(&pdev->dev, indio_dev);
	if(ret)
		return ret;

	printk("FFT Accelerator platform_set_drvdata\n");
	platform_set_drvdata(pdev, indio_dev);

	indio_dev_tx->dev.parent = &pdev->dev;
	indio_dev_tx->name = "fft-accelerator-tx";
	indio_dev_tx->modes = INDIO_DIRECT_MODE | INDIO_BUFFER_HARDWARE;
	indio_dev_tx->info = &fft_accelerator_empty_info;
	indio_dev_tx->channels = fft_accelerator_chan_spec;
	indio_dev_tx->num_channels = ARRAY_SIZE(fft_accelerator_chan_spec);
	indio_dev_tx->direction = IIO_DEVICE_DIRECTION_OUT;
	indio_dev_tx->setup_ops = &fft_accelerator_tx_setup_ops;

	printk("FFT Accelerator dmaengine_buffer_alloc\n");
	buffer_tx = devm_iio_dmaengine_buffer_alloc(&pdev->dev, "tx", &fft_accelerator_dma_buffer_ops,
						    indio_dev_tx);
	if (IS_ERR(buffer_tx))
		return PTR_ERR(buffer_tx);

	printk("FFT Accelerator iio_device_attach_buffer\n");
	iio_device_attach_buffer(indio_dev_tx, buffer_tx);

	printk("FFT Accelerator iio_device_set_drvdata\n");
	iio_device_set_drvdata(indio_dev_tx, st);

	printk("FFT Accelerator devm_iio_device_register\n");
	ret = devm_iio_device_register(&pdev->dev, indio_dev_tx);
	if (ret)
		return ret;

	printk("FFT Accelerator RX\n");

	indio_dev_rx->dev.parent = &pdev->dev;
	indio_dev_rx->name = "fft-accelerator-rx";
	indio_dev_rx->modes = INDIO_DIRECT_MODE | INDIO_BUFFER_HARDWARE;
	indio_dev_rx->info = &fft_accelerator_empty_info;
	indio_dev_rx->channels = fft_accelerator_chan_spec;
	indio_dev_rx->num_channels = ARRAY_SIZE(fft_accelerator_chan_spec);

	buffer_rx = devm_iio_dmaengine_buffer_alloc(&pdev->dev, "rx", &fft_accelerator_dma_buffer_ops,
						    indio_dev_rx);
	if (IS_ERR(buffer_rx))
		return PTR_ERR(buffer_rx);
	iio_device_attach_buffer(indio_dev_rx, buffer_rx);

	iio_device_set_drvdata(indio_dev_rx, st);

	return devm_iio_device_register(&pdev->dev, indio_dev_rx);

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
