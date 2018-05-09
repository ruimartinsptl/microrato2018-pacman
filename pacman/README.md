+ src - Codigo principal do micro rato
+ exp - Experiencias, tem pequenos pedacos de codigo que podem ser colados no main para testes

# Se usar conda:
```bash
conda create --name microrato python=2.7
conda activate microrato
```
```bash
pip install -U platformio

pio lib install "Servo"
pio lib install "AccelStepper"
pio lib install "IRremote"
```

Iniciar o projecto pela primeira vez para criar ficheiros do cmake:
```bash
pio init -b <nanoatmega328|megaADK|dueUSB> --ide clion
```

Upload para uma board especifica quando houver varios arduinos ligados por USB:
```bash
pio run -t upload -e <nano|due|mega> [ --upload-port /dev/cu.usbmodem1411 ]
pio run -t upload -e nano
pio run -t upload -e due  # [env:due] é um alias para [env:dueUSB]
pio run -t upload -e mega
```

CLEAN:
```bash
pio run -t clean
```
```bash
miniterm.py --exit-char 24 --raw /dev/cu.usbmodem1411 115200 # [Exit char é para CTRL X]
```


Abreviado:
```bash
pio run -t upload -e due && miniterm.py --exit-char 24 --raw /dev/cu.usbmodem1411 115200
pio run -t upload -e mega && miniterm.py --exit-char 24 --raw /dev/cu.usbmodem1411 115200
```

https://matplotlib.org/1.5.3/faq/virtualenv_faq.html

# Upload via ESP:
pio run && ./megaflash.sh 192.168.1.177 .pioenvs/mega/firmware.hex

# Ler serial da rede com netcat:
nc 192.168.1.177 23

# Usar a aplicação de gráficos em tempo real
##Via USB Serial: 
```bash
python serial_read_v2.py -p <serial_port>
```

##Via ESP log:
1. Criar uma interface serial virtual com para pordutor e consumidor: 
```bash
socat -d -d PTY,link=./ttydevice,raw,echo=0 PTY,link=./ttyclient,raw,echo=0
```
> NOTA: Se necessário instalar socat com (em macOS):
> ```bash brew install socat```

2. Cricar o produtor fazendo redirect da ligação Telnet para Serial
```bash
python tcp_serial_redirect.py -c 192.168.1.177:23 ttyclient 115200
```

3. Iniciar a aplicação 
```bash
python serial_read_v2.py -p ttydevice
```


