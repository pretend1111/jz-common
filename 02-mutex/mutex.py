import threading

mtx = threading.Lock()

def releaseHydrogen(Hnum):
    global mtx
    while Hnum >= 4:
        with mtx:
            Hnum -= 4
            print("HHHH", end='')

def releaseCarbon(Cnum):
    global mtx
    while Cnum >= 1:
        with mtx:
            Cnum -= 1
            print("C", end='')

def main():
    H = 0
    C = 0
    CH4 = input()
    for ch in CH4:
        if ch == 'C':
            C += 1
        elif ch == 'H':
            H += 1

    a = H // 4 - C
    if a < 0:
        C += a
    else:
        H = 4 * C

    t1 = threading.Thread(target=releaseHydrogen, args=(H,))
    t2 = threading.Thread(target=releaseCarbon, args=(C,))
    t1.start()
    t2.start()
    t1.join()
    t2.join()

if __name__ == "__main__":
    main()