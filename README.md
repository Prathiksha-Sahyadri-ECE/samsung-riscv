# RISC-V Talent Development Program Powered by SAMSUNG and VSD
This is a RISC-V Internship using VSDSquadron Mini based  on RISC-V architecture and uses open-source tools to teach students about VLSI SoC design and RISC-V. The instructor and guide for this internship program is Mr. Kunal Ghosh, Co-Founder of VSD.

# ABOUT ME🚀
Name: Prathiksha Shetty
-
College: Sahyadri College of Engineering and Management, Adyar, Mangaluru.
-
Email ID: prathikshashetty.ec22@sahyadri.edu.in or shettyprathiksha278@gmail.com
-
LinkedIn: [Prathiksha Shetty](https://www.linkedin.com/in/prathiksha-shetty-00270725a?utm_source=share&utm_campaign=share_via&utm_content=profile&utm_medium=android_app)
-
<details>
<summary>TASK1:Development of C Based LAB</summary>
<img 
src="https://github.com/Prathiksha-Sahyadri-ECE/samsung-riscv/blob/main/Task1/C_based_lab1.png"
  <img
src="https://github.com/Prathiksha-Sahyadri-ECE/samsung-riscv/blob/main/Task1/RISC-V_based_lab1.png"
<img
src="https://github.com/Prathiksha-Sahyadri-ECE/samsung-riscv/blob/main/Task1/RISC-V_based_lab2.png"
<img
src="https://github.com/Prathiksha-Sahyadri-ECE/samsung-riscv/blob/main/Task1/RISC-V_based_lab3.png"
</details>
<details>
<summary>TASK2:Simulation with Spike</summary>
<img 
src="https://github.com/Prathiksha-Sahyadri-ECE/samsung-riscv/blob/main/Task%202/riscv_objdump_-01_3.png"
  <img
src="https://github.com/Prathiksha-Sahyadri-ECE/samsung-riscv/blob/main/Task%202/riscv_objdump_-Ofast_3.png.png"
<img
src="https://github.com/Prathiksha-Sahyadri-ECE/samsung-riscv/blob/main/Task%202/list_objects.png"
<img
src="https://github.com/Prathiksha-Sahyadri-ECE/samsung-riscv/blob/main/Task%202/13.png"
</details>

<details>
<summary>TASK3:Identification of RISCV instructions</summary>
  <img
src="https://github.com/Prathiksha-Sahyadri-ECE/samsung-riscv/blob/main/TASK3/32-bit%20instruction%20code.md"
<summary>1. lui a0, 0x21 
Opcode(LUI): 0110111  
Immediate (0x21<<12): 0000000000101011 
Register (rd): a0 = 01010  

| imm[31:12]      | rd    | opcode  |
|------------------|-------|---------|
| 0000000000101011 | 01010 | 0110111 |

---

###2. addi sp, sp, -16  
Opcode(ADDI): 0010011  
Immediate: -16 = 111111110000 (12 bits)  
Registers: sp(rd) = 00010, sp(rs1) = 00010  

| imm[11:0]      | rs1   | funct3 | rd    | opcode  |
|-----------------|-------|--------|-------|---------|
| 111111110000    | 00010 | 000    | 00010 | 0010011 |

---

### 3. li a2, 5  
Opcode(ADDI): 0010011  
Immediate: 5 = 000000000101  
Registers: a2(rd) = 00101, x0(rs1) = 00000  

| imm[11:0]      | rs1   | funct3 | rd    | opcode  |
|-----------------|-------|--------|-------|---------|
| 000000000101    | 00000 | 000    | 00101 | 0010011 |

---

### 4. li a1, 10  
Opcode(ADDI): 0010011  
Immediate: 10 = 000000001010  
Registers: a1(rd) = 01011, x0(rs1) = 00000  

| imm[11:0]      | rs1   | funct3 | rd    | opcode  |
|-----------------|-------|--------|-------|---------|
| 000000001010    | 00000 | 000    | 01011 | 0010011 |

---

### 5. addi a0, a0, 384  
Opcode(ADDI): 0010011  
Immediate: 384 = 000011000000  
Registers: a0(rd) = 01010, a0(rs1) = 01010  

| imm[11:0]      | rs1   | funct3 | rd    | opcode  |
|-----------------|-------|--------|-------|---------|
| 000011000000    | 01010 | 000    | 01010 | 0010011 |

---

### 6. sd ra, 8(sp)  
Opcode(SD): 0100111  
Immediate: 8 = 0000000001000 (split into imm[11:5] and imm[4:0])  
Registers: rs1(sp) = 00010, rs2(ra) = 00001  

| imm[11:5] | rs2   | rs1   | funct3 | imm[4:0] | opcode  |
|-----------|-------|-------|--------|----------|---------|
| 0000000   | 00001 | 00010 | 011    | 01000    | 0100111 |

---

### 7. jal ra, 10408  
Opcode(JAL): 1101111  
Immediate: 10408 = 0x28A8 (split into J-type format)  
Registers: ra(rd) = 00001  

| imm[20] | imm[10:1] | imm[11] | imm[19:12] | rd    | opcode  |
|---------|-----------|---------|------------|-------|---------|
|   0     | 1010101000|    1    |  01010000  | 00001 | 1101111 |

---

### 8. ld ra, 8(sp)  
Opcode(LD): 0000011  
Immediate: 8 = 0000000001000  
Registers: ra(rd) = 00001, sp(rs1) = 00010  

| imm[11:0]      | rs1   | funct3 | rd    | opcode  |
|-----------------|-------|--------|-------|---------|
| 000000001000    | 00010 | 011    | 00001 | 0000011 |

---

### 9. li a0, 0  
Opcode(ADDI): 0010011  
Immediate: 0 = 000000000000  
Registers: a0(rd) = 01010, x0(rs1) = 00000  

| imm[11:0]      | rs1   | funct3 | rd    | opcode  |
|-----------------|-------|--------|-------|---------|
| 000000000000    | 00000 | 000    | 01010 | 0010011 |

---

### 10. addi sp, sp, 16  
Opcode(ADDI): 0010011  
Immediate: 16 = 000000010000  
Registers: sp(rd) = 00010, sp(rs1) = 00010  

| imm[11:0]      | rs1   | funct3 | rd    | opcode  |
|-----------------|-------|--------|-------|---------|
| 000000010000    | 00010 | 000    | 00010 | 0010011 |

---
</details>
<details>
<summary>TASK4:Functional Simulation of RISC-V Core</summary>
</summary>
<br>
Steps to perform functional simulation of RISCV

1. Download Files:
Download the code from the reference github repo.

2. Set Up Simulation Environment:
Install iverlog using commands:

        sudo apt install iverilog
        sudo apt install gtkwave

3. To run and simulate the verilog code, enter the following command:

        iverilog -o iiitb_rv32i iiitb_rv32i.v iiitb_rv32i_tb.v
        ./iiitb_rv32i

4. To see the simulation waveform in GTKWave, enter the following command:

        gtkwave iiitb_rv32i.vcd

32-bits instruction used in the code:

![Instructions](<TASK 4/installing gtkwave iverilog.png>)

Analysing the Output Waveform of various instructions that we have covered in this task.

1. ADD R6,R1,R2

![ADD R6,R1,R2](<TASK 4/add.png>)

  32 bit instruction:32'h02208300

2. SUB R7,R1,R2

![SUB R7,R1,R2](<TASK 4/sub1.png>)

32 bit instruction:32'h02209380

3. And R8,R1,R3

![And R8,R1,R3](<TASK 4/sub1.png>)

32 bit instruction:32'h0230a400

4. OR R9,R2,R5

![OR R9,R2,R5](<TASK 4/or.png>)

32 bit instruction:32'h02513480

5. XOR R10,R1,R4

![XOR R10,R1,R4](<TASK 4/xor.png>)

32 bit instruction:32'h0240c500

6. SLT R11,R2,R4

![SLT R11,R2,R4](<TASK4/slt.png>)

32 bit instruction:32'h02415580

7. ADDI R12,R4,5

![ADDI R12,R4,5](<TASK 4/addi.png>)

32 bit instruction:32'h00520600

8. BEQ R0,R0,15

![BEQ R0,R0,15](<TASK 4/beq.png>)

32 bit instruction:32'h00f00002

</details>
<details>
<summary>TASK5:Project overview-circuit diagram</summary>
</summary>
1.Pinout Diagram of Obstacle-Detection
<img 
src="https://github.com/Prathiksha-Sahyadri-ECE/samsung-riscv/blob/main/TASK%205/OBSTRACL-DETECTION%20CIRCUIT%20DIAGRAM.pdf"
<img

2.Components Required:
1.VSD Squadronmini CH32V00x RISC V processor

2.Ultrasonic sensor

3.Buzzer

4.Jumper wires

5.Bread board

## Pin Connections:

### Ultrasonic Sensor
| Pin | CH32V00x |
|-----|----------|
| TRIG| PC4      |
| ECO | PC5      |
| VCC | 3.3      |
| GND | GND      |

### BUZZER
| Pin  | CH32V00x |
|------|----------|
| +    | PC7      |
| -    | GND      |

---


![PIN CONNECTION DETAILS](https://github.com/Prathiksha-Sahyadri-ECE/samsung-riscv/blob/main/TASK%205/PIN%20CONNECTION.pdf).

3. Test code simulation.

https://github.com/Prathiksha-Sahyadri-ECE/samsung-riscv/blob/main/TASK%205/OBSTRACLE_DETECTION_VIDEO.mp4
</details>

<details>
<summary>TASK6:Project Application</summary>
</summary>
1.Obstacle-Detection video.
https://github.com/Prathiksha-Sahyadri-ECE/samsung-riscv/blob/4475a15043e46838028152b822a71a5599043498/TASK%206/OBSTRACLE_DETECTION_VIDEO.mp4


2.Obstacle-Detection code.
```
#include "ch32v00x.h"
#include "debug.h"

// Define Ultrasonic Sensor Pins
#define TRIG_PIN    PC4
#define ECHO_PIN    PC5

// Define Buzzer Pin
#define BUZZER_PIN  PC7

void delay_us(uint32_t us) {
    for (uint32_t i = 0; i < (SystemCoreClock / 8000000) * us; i++) {
        __NOP();  // No operation (wastes time for delay)
    }
}

void setup() {
    // Enable GPIOC clock
    RCC->APB2PCENR |= RCC_APB2Periph_GPIOC;

    // Configure Trig pin as Output
    GPIOC->CFGLR &= ~(0xF << (4 * (TRIG_PIN % 8)));  
    GPIOC->CFGLR |= (0x3 << (4 * (TRIG_PIN % 8)));  

    // Configure Echo pin as Input
    GPIOC->CFGLR &= ~(0xF << (4 * (ECHO_PIN % 8)));  
    GPIOC->CFGLR |= (0x4 << (4 * (ECHO_PIN % 8)));  

    // Configure Buzzer pin as Output
    GPIOC->CFGLR &= ~(0xF << (4 * (BUZZER_PIN % 8)));  
    GPIOC->CFGLR |= (0x3 << (4 * (BUZZER_PIN % 8)));  

    // Ensure Trig is low initially
    GPIOC->OUTDR &= ~(1 << TRIG_PIN);

    // Turn off Buzzer initially
    GPIOC->OUTDR &= ~(1 << BUZZER_PIN);
}

uint32_t getDistance() {
    uint32_t start_time, stop_time, duration;

    // Send Trigger Pulse (10us)
    GPIOC->OUTDR |= (1 << TRIG_PIN);
    delay_us(10);
    GPIOC->OUTDR &= ~(1 << TRIG_PIN);

    // Wait for Echo to go HIGH
    while (!(GPIOC->INDR & (1 << ECHO_PIN)));  
    start_time = SysTick->CNT;

    // Wait for Echo to go LOW
    while (GPIOC->INDR & (1 << ECHO_PIN));  
    stop_time = SysTick->CNT;

    // Calculate duration
    duration = stop_time - start_time;

    // Convert to distance (in cm) using speed of sound (343m/s)
    return (duration * 0.0343) / 2;
}

void loop() {
    uint32_t distance = getDistance();

    if (distance > 0 && distance <= 10) {
        GPIOC->OUTDR |= (1 << BUZZER_PIN);  // Turn ON Buzzer if object is close
    } else {
        GPIOC->OUTDR &= ~(1 << BUZZER_PIN); // Turn OFF Buzzer
    }
}

int main() {
    setup();
    while (1) {
        loop();
    }
}
