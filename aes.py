import random
import secrets

def generate_aes_key(length=32):
    """
    セキュアな32バイトのAESキーを生成する
    """
    # secrets.token_bytes()を使用して暗号学的に安全な乱数を生成
    return secrets.token_bytes(length)

def generate_aes_iv(length=16):
    """
    セキュアな16バイトのAES初期化ベクトルを生成する
    """
    return secrets.token_bytes(length)

def format_as_c_array(data, name, width=8):
    """
    バイトデータをC/C++の配列形式にフォーマットする
    """
    hex_values = [f"0x{b:02x}" for b in data]
    lines = []
    
    for i in range(0, len(hex_values), width):
        chunk = hex_values[i:i+width]
        lines.append("    " + ", ".join(chunk) + ",")
    
    # 最後のカンマを削除
    lines[-1] = lines[-1][:-1]
    
    result = f"const uint8_t {name}[{len(data)}] = {{\n"
    result += "\n".join(lines)
    result += "\n};"
    
    return result

def main():
    # 新しいAESキーとIVを生成
    aes_key = generate_aes_key(32)
    aes_iv = generate_aes_iv(16)
    
    # C/C++配列形式にフォーマット
    key_array = format_as_c_array(aes_key, "aesKey")
    iv_array = format_as_c_array(aes_iv, "aesIv")
    
    print(key_array)
    print()
    print(iv_array)

if __name__ == "__main__":
    main()