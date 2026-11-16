"""Reproducible offline AVR build using installed Arduino core + avr-gcc.
python tools/build_avr.py --core PATH_TO_ArduinoCore-avr --toolchain PATH_TO_avr
No uploads. Equivalent ATmega2560 flags to platformio.ini.
"""
import argparse
from pathlib import Path
import subprocess
p=argparse.ArgumentParser(); p.add_argument('--core',type=Path,required=True); p.add_argument('--toolchain',type=Path,required=True); p.add_argument('--configured',action='store_true'); a=p.parse_args()
root=Path(__file__).resolve().parents[1]; fw=root/'firmware'; out=fw/('build_enabled' if a.configured else 'build'); out.mkdir(exist_ok=True)
core=a.core.resolve(); tc=a.toolchain.resolve()/'bin'
libs=fw/'lib'; src=fw/'AntennaTracker'
includes=[core/'cores/arduino',core/'variants/mega',src]+[x/'src' for x in libs.iterdir() if (x/'src').is_dir()]
flags=['-mmcu=atmega2560','-DF_CPU=16000000L','-DARDUINO=10819','-DARDUINO_AVR_MEGA2560','-DARDUINO_ARCH_AVR','-DSERIAL_RX_BUFFER_SIZE=256','-DSERIAL_TX_BUFFER_SIZE=128','-Os','-ffunction-sections','-fdata-sections','-flto','-g','-Wall']
if a.configured: flags+=['-DANTENNA_MECHANICS_CONFIGURED=1']
for i in includes: flags+=['-I',str(i)]
files=list((core/'cores/arduino').glob('*.c'))+list((core/'cores/arduino').glob('*.cpp'))+list((core/'cores/arduino').glob('*.S'))
files+=list((libs/'Arduino_FreeRTOS_Library-master/src').glob('*.c'))+list((libs/'Arduino_FreeRTOS_Library-master/src').glob('*.cpp'))
files+=list((libs/'Servo-master/src/avr').glob('*.cpp'))+list(src.glob('*.cpp'))+list(src.glob('*.ino'))
objects=[]
for i,f in enumerate(files):
    obj=out/f'{i}_{f.stem}.o'; cpp=f.suffix in ('.cpp','.ino')
    compiler=tc/('avr-g++' if cpp else 'avr-gcc')
    extra=['-std=gnu++11','-fpermissive','-fno-exceptions','-fno-threadsafe-statics','-x','c++'] if cpp else (['-x','assembler-with-cpp'] if f.suffix=='.S' else ['-std=gnu11'])
    subprocess.run([str(compiler),*flags,*extra,'-c',str(f),'-o',str(obj)],check=True)
    objects.append(str(obj))
elf=out/'AntennaTracker.elf'; hexfile=out/'AntennaTracker.hex'
subprocess.run([str(tc/'avr-g++'),'-mmcu=atmega2560','-Os','-flto','-Wl,--gc-sections','-o',str(elf),*objects,'-lm'],check=True)
subprocess.run([str(tc/'avr-objcopy'),'-O','ihex','-R','.eeprom',str(elf),str(hexfile)],check=True)
subprocess.run([str(tc/'avr-size'),'-C','--mcu=atmega2560',str(elf)],check=True)
print(hexfile)
