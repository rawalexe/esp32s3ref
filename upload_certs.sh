
python3 managed_components/espressif__esp_secure_cert_mgr/tools/configure_esp_secure_cert.py \
	-p /dev/cu.usbmodem14101 \
	--keep_ds_data_on_host \
	--ca-cert ../AWS_CERTS/AmazonRootCA1.pem \
	--device-cert ../AWS_CERTS/current-certificate.pem.crt \
	--private-key ../AWS_CERTS/current-private.pem.key \
	--target_chip esp32s3 \
	--secure_cert_type cust_flash
