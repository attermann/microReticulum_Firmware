import hashlib


ESP_IMAGE_MAGIC = 0xE9
ESP_IMAGE_HEADER_SIZE = 24
ESP_IMAGE_HASH_APPENDED_OFFSET = 23
SHA256_SIZE = 32
KISS_FEND = 0xC0
KISS_FESC = 0xDB
KISS_TFEND = 0xDC
KISS_TFESC = 0xDD
KISS_CMD_FIRMWARE_HASH = 0x58


def esp_image_sha256(image: bytes) -> bytes:
    """Return the digest produced by esp_partition_get_sha256() for an app image."""
    if len(image) < ESP_IMAGE_HEADER_SIZE:
        raise ValueError("ESP application image is shorter than its header")
    if image[0] != ESP_IMAGE_MAGIC:
        raise ValueError("firmware is not an ESP application image")

    hash_appended = image[ESP_IMAGE_HASH_APPENDED_OFFSET] == 1
    if not hash_appended:
        return hashlib.sha256(image).digest()

    if len(image) < ESP_IMAGE_HEADER_SIZE + SHA256_SIZE:
        raise ValueError("ESP application image is missing its appended SHA-256")

    content = image[:-SHA256_SIZE]
    appended_hash = image[-SHA256_SIZE:]
    calculated_hash = hashlib.sha256(content).digest()
    if calculated_hash != appended_hash:
        raise ValueError("ESP application image has an invalid appended SHA-256")

    return appended_hash


def firmware_hash_kiss_frame(firmware_hash: bytes) -> bytes:
    if len(firmware_hash) != SHA256_SIZE:
        raise ValueError("firmware hash must be exactly 32 bytes")

    escaped_hash = firmware_hash.replace(
        bytes([KISS_FESC]), bytes([KISS_FESC, KISS_TFESC])
    ).replace(
        bytes([KISS_FEND]), bytes([KISS_FESC, KISS_TFEND])
    )
    return bytes([KISS_FEND, KISS_CMD_FIRMWARE_HASH]) + escaped_hash + bytes([KISS_FEND])
