import multiprocessing as mp
import threading as td
import numpy as np
import time
import sys
import os
from typing import Dict, List, Union, Callable
import numbers
from queue import Full, Empty
from multiprocessing.managers import SharedMemoryManager

from umi.shared_memory.shared_memory_util import SharedAtomicCounter


        
def job(a, b):
    print("good job")
    
def print_numbers(n):
    for i in range(n):
        time.sleep(1)
        print(i)
    
if __name__ == "__main__":
    from threading import Thread, Lock


    a = 0
    lock = Lock()


    def fn(n: int) -> None:
        global a
        for _ in range(n):
            # with lock:
            #     a += 1
            a+=1



    # setup
    total = 10_000_000
    # run threads to completion
    t1 = Thread(target=fn, args=(total // 2,))
    t2 = Thread(target=fn, args=(total // 2,))
    t1.start(), t2.start()
    t1.join(), t2.join()
    # print results
    print(f"a[{a}] != total[{total}]")
