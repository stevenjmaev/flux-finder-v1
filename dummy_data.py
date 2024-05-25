# %%

end = 65535

current = 0

zero = [15, 35]
one = [35, 15]

data = 0xFF0000_00FF00_0000FF

arr = []
i = True

current_bit = 72
while current_bit > 0:
    if data & (1 << (current_bit - 1)): # it's a '1'
        width = 35 if i else 15
    else: # it's a '0'
        width = 15 if i else 35

    current += width
    if not i:
        current_bit -= 1
    i = not i
    arr.append(current)
# %%
print("{")

for i, a in enumerate(arr):
    print(f"{a}, ", end="")
    if i % 15 == 0 and i != 0:
        print("")

# %%
