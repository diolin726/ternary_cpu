start:addi x8, x0 ,10
      addi x6 ,x0 , 1
      jal x27 , loop
loop: beq x8 , x5 , stop
      add x5 , x5 , x6
      jalr x28 , x27 , 0
stop: sw x28 , 0(x0)
      lw x9 , 0(x0)
