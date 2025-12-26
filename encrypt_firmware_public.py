#!/usr/bin/env python3
import sys
import os
from cryptography.hazmat.primitives.ciphers import Cipher, algorithms, modes
from cryptography.hazmat.backends import default_backend

# 動作させるための手順:
# ※Pythonを先にインストールしてください
# pip install cryptography
# 作業ディレクトリでコマンドプロンプトを開く
# python encrypt_firmware_public.py ファームウェアの名前.bin 任意の名前(暗号化済ファイル).bin

# このツールはOTAアップデートのためにバイナリファイルを暗号化するものです。
# 暗号化したバイナリファイルをOTAアップデート用のファイルとして選択してください。
# 元々この領域を他人に触らせることを想定していなかったため複雑です。
# (本来はファームウェアをブラックボックスにする予定でした)

# DD_Tuningで使用しているAESキーを以下に設定してください
AES_KEY = bytes([
    0x60, 0xc2, 0x89, 0x17, 0x67, 0x29, 0x1b, 0x3f,
    0x6a, 0x45, 0x79, 0x83, 0x54, 0x86, 0xfe, 0x5a,
    0x76, 0x5e, 0xe4, 0xb0, 0xd0, 0x90, 0x15, 0xc9,
    0x40, 0x21, 0x2f, 0x5d, 0x91, 0x39, 0x29, 0xae
])

AES_IV = bytes([
    0x12, 0x9d, 0xc6, 0x01, 0xf5, 0xd1, 0x4b, 0x15,
    0xfc, 0x05, 0xb4, 0x8f, 0xb6, 0x44, 0x8c, 0x50
])


def encrypt_firmware(input_file, output_file):
    # ファイルを読み込み
    with open(input_file, 'rb') as f:
        firmware_data = f.read()

    # PKCS7パディング
    block_size = 16
    padding_length = block_size - (len(firmware_data) % block_size)
    if padding_length == 0:
        padding_length = block_size
    
    firmware_data += bytes([padding_length] * padding_length)

    # 暗号化
    cipher = Cipher(algorithms.AES(AES_KEY), modes.CBC(AES_IV), backend=default_backend())
    encryptor = cipher.encryptor()
    encrypted_data = encryptor.update(firmware_data) + encryptor.finalize()

    # 暗号化されたファームウェアを保存
    with open(output_file, 'wb') as f:
        f.write(encrypted_data)

def main():
    if len(sys.argv) != 3:
        print("Usage: python encrypt_firmware.py <input_firmware.bin> <output_encrypted.bin>")
        sys.exit(1)

    input_file = sys.argv[1]
    output_file = sys.argv[2]

    if not os.path.exists(input_file):
        print(f"Error: Input file '{input_file}' not found.")
        sys.exit(1)

    try:
        encrypt_firmware(input_file, output_file)
        print(f"Firmware encrypted successfully: {output_file}")
    except Exception as e:
        print(f"Error encrypting firmware: {str(e)}")
        sys.exit(1)

if __name__ == "__main__":
    main()


