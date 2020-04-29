start:
  ldi r2, 0    ; load r2 with 0
  ldi r1, 1    ; load r1 with 1
  stf r1, 0xc8 ; stores r1 to FU PDA (Peripheral Destination Address) - clear the EPIT output compare interrupt flag
  done 4       ; done, until triggered - wait for the next EPIT event

update:
  stf r1, 0xc8 ; store r1 to FU PDA (Peripheral Destination Address) again - clear the EPIT output compare interrupt flag
  stf r2, 0x2b ; store r2 to FU MD (Memory Data) word - prefetch - this must be writing to the PWM duty cycle register, so the first time it will write zero - why does this get written twice???
  ldf r3, 0x00 ; load FU MSA (Memory Source Address) to r3 (read source address)
  cmphs r3, r7 ; compare >= (r7 is end address)
  bt alldone   ; branch if true to alldone
  ldf r2, 0x0a ; load FU MD (Memory Data) halfword to r2
  stf r2, 0x2b ; store r2 to FU MD (Memory Data) word - prefetch - write to PWM duty cycle

  done 4       ; done, until triggered - wait for the next EPIT event
  btsti r1, 0  ; this is always true...
  bt update    ; branch to update

alldone:
  done 3       ; done and should generate interrupt
  done 4       ; done, until triggered - wait for the next EPIT event
  btsti r1, 0  ; this is always true
  bt update    ; branch to update
