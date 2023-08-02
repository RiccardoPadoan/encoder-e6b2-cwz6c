from machine import Pin
import utime
import tm1637 #display
from rp2 import PIO, StateMachine, asm_pio #encoder
from time import sleep_ms


tollerance = 100
ratio = 10
maxValue = 5000

# rele
rele1 = Pin(7, Pin.OUT)
rele2 = Pin(8, Pin.OUT)
rele1.value(0)
rele2.value(0)

# keypad pins
row_list = [18,19,20,21]  
col_list = [22,23,24]

for x in range(4):
    row_list[x] = Pin(row_list[x], Pin.OUT)
    row_list[x].value(1)

for x in range(3):
    col_list[x] = Pin(col_list[x], Pin.IN, Pin.PULL_UP)

key_list = [
    ["1", "2", "3"],
    ["4", "5", "6"],
    ["7", "8", "9"],
    ["*", "0", "#"]]

def keypad(col, row):
  for r in row:
    r.value(0)
    result = [col[0].value(), col[1].value(), col[2].value()]
    if min(result) == 0:
      key = key_list[int(row.index(r))][int(result.index(0))]
      r.value(1)
      return (key)
    r.value(1)


# display
tm = tm1637.TM1637(clk=Pin(2), dio=Pin(0))

def swap(segs):
    length = len(segs)
    segs = segs[::-1]
    segs[0], segs[2] = segs[2], segs[0]
    if length >= 4:
        segs[3], segs[5] = segs[5], segs[3]
    return segs

def swap_str(segs):
    length = len(segs)
    if length == 4 or length == 5:
        segs.extend(bytearray([0] * (6 - length)))
    segs[0], segs[2] = segs[2], segs[0]
    if length >= 4:
        segs[3], segs[5] = segs[5], segs[3]
    return segs

tm.brightness(0)
zero = [0,0,0,0,0,0]
tm.write(zero)
measure = zero
encKey = 0

# read last positions.txt write on display
try:
    f = open('positions.txt', 'r')
except:
    f = open('positions.txt', 'w')
positions = f.read()
f.close()
l = list(positions.split(' '))
length = int(len(l))
lastPos = int(l[length-1])
lastMisure = int(lastPos/ratio)
position = zero
for x in str(lastMisure):
    encX = tm.encode_char(x)
    position = [encX] + position
    position = position[:6]
tm.write(swap(position))

# encoder
class PIO_QENC:
    def __init__(self, sm_id, pins, freq=10_000_000):
        if not isinstance(pins, (tuple, list)) or len(pins) != 2:
            raise ValueError('2 successive pins required')
        pinA = int(str(pins[0]).split(')')[0].split('(')[1].split(',')[0])
        pinB = int(str(pins[1]).split(')')[0].split('(')[1].split(',')[0])
        if abs(pinA-pinB) != 1:
            raise ValueError('2 successive pins required')
        in_base = pins[0] if pinA < pinB else pins[1]
        self.sm_qenc = StateMachine(sm_id, self.sm_qenc, freq=freq, in_base=in_base, out_base=in_base)
        self.sm_qenc.exec("set(x, 1)")  # we once decrement at the start
        self.sm_qenc.exec("in_(pins, 2)")
        self.sm_qenc.active(1)
    
    @staticmethod
    @rp2.asm_pio(in_shiftdir=PIO.SHIFT_LEFT, out_shiftdir=PIO.SHIFT_RIGHT)
    def sm_qenc():
        jmp("read")        # 0000 : from 00 to 00 = no change
        jmp("decr")        # 0001 : from 00 to 01 = backward
        jmp("incr")        # 0010 : from 00 to 10 = orward
        jmp("read")        # 0011 : from 00 to 11 = error
        jmp("incr")        # 0100 : from 01 to 00 = forward
        jmp("read")        # 0101 : from 01 to 01 = no change
        jmp("read")        # 0110 : from 01 to 10 = error
        jmp("decr")        # 0111 : from 01 to 11 = backward
        jmp("decr")        # 1000 : from 10 to 00 = backward
        jmp("read")        # 1001 : from 10 to 01 = error
        jmp("read")        # 1010 : from 10 to 10 = no change
        jmp("incr")        # 1011 : from 10 to 11 = forward
        jmp("read")        # 1100 : from 11 to 00 = error
        jmp("incr")        # 1101 : from 11 to 01 = forward

        label("decr")
        jmp(x_dec, "read") # 1110 : from 11 to 10 = backward

        label("read")      # 1111 : from 11 to 11 = no change
        mov(osr, isr)      # save last pin input in OSR
        mov(isr, x)
        push(noblock)
        out(isr, 2)        # 2 right bits of OSR into ISR, all other 0
        in_(pins, 2)       # combined with current reading of input pins
        mov(pc, isr)       # jump into jump-table at addr 0

        label("incr")      # increment x by inverting, decrementing and inverting
        mov(x, invert(x))
        jmp(x_dec, "here")
        label("here")
        mov(x, invert(x))
        jmp("read")
        
        nop()
        nop()
        nop()
        nop()
        nop()
        nop()
        nop()

    def read(self):
        for _ in range(self.sm_qenc.rx_fifo()):
            self.sm_qenc.get()
        n = self.sm_qenc.get()
        return n if n < (1<<31) else n - (1<<32)

pinA = Pin(14, Pin.IN, Pin.PULL_UP)
pinB = Pin(15, Pin.IN, Pin.PULL_UP)
qenc = PIO_QENC(0, (pinA, pinB))
move = False
value = 0

while True:
    key =  keypad(col_list, row_list)
    if key:
        if key == '#' or key == '*':
            
            # activation relè
            move = True
            qenc.sm_qenc.active(1)
            while move:
                encVal = qenc.read() + lastPos
                encMin = (encVal - tollerance)/ratio
                encMax = (encVal + tollerance)/ratio               
                if encMin >= value:
                    rele2.value(0)
                    rele1.value(1)
                elif encMax <= value:
                    rele1.value(0)
                    rele2.value(1)
                else:
                    rele1.value(0)                   
                    rele2.value(0)
                    utime.sleep(0.3)
                    f = open('positions.txt', 'a')
                    f.write(' ' + str(qenc.read() + lastPos))
                    f.close()
                    move = False
                    value = 0
                    measure = zero
            qenc.sm_qenc.active(0)
                    
        else:
            if value == 0 and key == '0':
                pass
            else:
                value = int(value*10)+int(key)
                if value >= maxValue:
                    tm.write(swap_str(tm.encode_string('troppo')))
                    utime.sleep(2)
                    measure = zero
                    value = 0
                else:
                    encKey = tm.encode_char(key)
                    measure = [encKey] + measure
                    measure = measure[:6]
            tm.write(swap(measure))
    utime.sleep(0.3)
    