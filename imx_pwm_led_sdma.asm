start:
  ldi r2, 0    ; load r2 with 0 (duty cycle register)
  ldi r1, 1    ; load r1 with 1 - used to clear the EPIT output compare interrupt flag and perform unconditional branches
  stf r1, 0xc8 ; store r1 to FU PDA (Peripheral Destination Address) - clear the EPIT output compare interrupt flag

update:
  done 4       ; done - wait for the next EPIT event
  stf r1, 0xc8 ; store r1 to FU PDA (Peripheral Destination Address) - clear the EPIT output compare interrupt flag
  ldf r3, 0x00 ; load FU MSA (Memory Source Address) to r3 (read address register)
  cmphs r3, r7 ; compare if r3 (read address register) >= r7 (end address register)
  bt alldone   ; branch if true to alldone
  ldf r2, 0x0a ; load FU MD (Memory Data) halfword to r2 (duty cycle register)
  stf r2, 0x2b ; store r2 to FU MD (Memory Data) word - write to PWM duty cycle register
  btsti r1, 0  ; always true (for unconditional relative branch)
  bt update    ; branch to update (there is no relative jmp instruction, so relative branch is used instead)

alldone:
  done 3       ; generate SDMA interrupt
  btsti r1, 0  ; always true
  bt update    ; branch to update
