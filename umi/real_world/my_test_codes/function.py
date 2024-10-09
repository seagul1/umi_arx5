from typing import Union, Optional
import socket
import enum
import struct

def args_to_bytes(*args, int_bytes=1):
    buf = list()
    for arg in args:
        if isinstance(arg, float):
            # little endian 32bit float
            buf.append(struct.pack('<f', arg))
        elif isinstance(arg, int):
            buf.append(arg.to_bytes(length=int_bytes, byteorder='little'))
        elif isinstance(arg, str):
            buf.append(arg.encode('ascii'))
        else:
            raise RuntimeError(f'Unsupported type {type(arg)}')
    result = b''.join(buf)
    return result

result = args_to_bytes(123, 456.789, 'hello', int_bytes=2)
print(result)