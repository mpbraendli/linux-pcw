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

static const struct iio_chan_spec fft_accelerator_out_chan_spec[] = {
	{
		.type = IIO_VOLTAGE,
		.extend_name = "out",
		.output = 0,
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),
		.indexed = 1,
		.channel = 0,
		.scan_index = 0,
		.scan_type = {
			.sign = 's',
			.realbits = 8,
			.storagebits = 8,
		},
	}
};

static const struct iio_chan_spec fft_accelerator_in_chan_spec[] = {
	{
		.type = IIO_VOLTAGE,
		.extend_name = "in",
		.output = 1,
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),
		.indexed = 1,
		.channel = 0,
		.scan_index = 0,
		.scan_type = {
			.sign = 's',
			.realbits = 8,
			.storagebits = 8,
		},
	}
};

static int fft_accelerator_submit_block(
		struct iio_dma_buffer_queue *queue,
		struct iio_dma_buffer_block *block)
{
	struct iio_dev *indio_dev;
	int ret;

	if (!queue) {
		printk("FFT Accelerator submit queue=NULL!\n");
		return -EINVAL;
	}

	if (!queue->driver_data) {
		printk("FFT Accelerator submit queue->driver_data=NULL!\n");
		return -EINVAL;
	}


	if (!block) {
		printk("FFT Accelerator submit block=NULL!\n");
		return -EINVAL;
	}

	indio_dev = queue->driver_data;

	if (indio_dev->direction == IIO_DEVICE_DIRECTION_IN) {
		block->block.bytes_used = block->block.size;
		ret = iio_dmaengine_buffer_submit_block(queue, block, DMA_DEV_TO_MEM);
	} else {
		ret = iio_dmaengine_buffer_submit_block(queue, block, DMA_MEM_TO_DEV);
	}

	return ret;
}

static const struct iio_dma_buffer_ops fft_accelerator_dma_buffer_ops = {
	.submit = fft_accelerator_submit_block,
	.abort = iio_dmaengine_buffer_abort,
};

struct fft_accelerator_state{
};

/* we need that dummy function to get a valid iio_info object */
static int fft_accelerator_read_raw_dummy(struct iio_dev *indio_dev,
			   const struct iio_chan_spec *chan,
			   int *val, int *val2, long info)
{
	return -EINVAL;
}

static const struct iio_info fft_accelerator_info = {
	.read_raw = &fft_accelerator_read_raw_dummy,
};

static const struct of_device_id fft_accelerator_of_match[] = {
	{ .compatible = DRIVER_NAME, },
	{ },
};
MODULE_DEVICE_TABLE(of, fft_accelerator_of_match);

static int fft_accelerator_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;	// for devicetree parsing
	struct iio_dev *indio_dev_in, *indio_dev_out;
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

	indio_dev_in = devm_iio_device_alloc(&pdev->dev, sizeof(*st));
	if (!indio_dev_in) {
		printk("\nFFT Accelerator IN: -ENOMEM\n");
		return -ENOMEM;
	}

	indio_dev_out = devm_iio_device_alloc(&pdev->dev, sizeof(*st));
	if (!indio_dev_out) {
		printk("\nFFT Accelerator OUT: -ENOMEM\n");
		return -ENOMEM;
	}

	st = iio_priv(indio_dev_in);

	indio_dev_in->dev.parent = &pdev->dev;
	indio_dev_in->name = "fft-accelerator-in";
	indio_dev_in->modes = INDIO_DIRECT_MODE | INDIO_BUFFER_HARDWARE;
	indio_dev_in->info = &fft_accelerator_info;
	indio_dev_in->channels = fft_accelerator_in_chan_spec,
	indio_dev_in->num_channels = ARRAY_SIZE(fft_accelerator_in_chan_spec);
	indio_dev_in->direction = IIO_DEVICE_DIRECTION_OUT;
	iio_device_set_drvdata(indio_dev_in, st);

	st = iio_priv(indio_dev_out);
	indio_dev_out->dev.parent = &pdev->dev;
	indio_dev_out->name = "fft-accelerator-out";
	indio_dev_out->modes = INDIO_DIRECT_MODE | INDIO_BUFFER_HARDWARE;
	indio_dev_out->info = &fft_accelerator_info;
	indio_dev_out->channels = fft_accelerator_out_chan_spec,
	indio_dev_out->num_channels = ARRAY_SIZE(fft_accelerator_out_chan_spec);
	indio_dev_out->direction = IIO_DEVICE_DIRECTION_IN;
	iio_device_set_drvdata(indio_dev_out, st);

	buffer_in = devm_iio_dmaengine_buffer_alloc(&pdev->dev, "in",
			&fft_accelerator_dma_buffer_ops, indio_dev_in);
	if (IS_ERR(buffer_in)) {
		printk("FFT Accelerator buffer_in dmaengine_buffer_alloc IS ERR!\n");
		return PTR_ERR(buffer_in);
	}
	iio_device_attach_buffer(indio_dev_in, buffer_in);

	buffer_out = devm_iio_dmaengine_buffer_alloc(&pdev->dev, "out",
			&fft_accelerator_dma_buffer_ops, indio_dev_out);
	if (IS_ERR(buffer_out)) {
		printk("FFT Accelerator buffer_out dmaengine_buffer_alloc IS ERR!\n");
		return PTR_ERR(buffer_out);
	}
	iio_device_attach_buffer(indio_dev_out, buffer_out);

	ret = devm_iio_device_register(&pdev->dev, indio_dev_in);
	if (ret < 0) {
		dev_err(&pdev->dev, "devm_iio_device_register in failed");
		return ret;
	}

	ret = devm_iio_device_register(&pdev->dev, indio_dev_out);
	if (ret < 0) {
		dev_err(&pdev->dev, "devm_iio_device_register in failed");
		return ret;
	}

	platform_set_drvdata(pdev, indio_dev_in);
	platform_set_drvdata(pdev, indio_dev_out);

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
