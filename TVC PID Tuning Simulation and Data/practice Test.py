"""choice1 = [1, 2, 3]
choice2a = ["a", "b"]
choice2b = ["c", "d"]
choice2c = ["e", "f"]
while True:
    choice = input(f"Which one? {choice1}")
    if choice == choice[0]:
        letterChoice = input(f"Which one? {choice2a}?").lower()
        if letterChoice == choice2a[0]:
            print(f"{choice2a[0]} has been chosen, lmao")
        if letterChoice == choice2a[1]:
            print(f"{choice2a[1]} has been chosen, lmao")
    elif choice == choice[0]:
        letterChoice = input("C or D?").lower()
        if letterChoice == choice2b[0]:
            print(f"{choice2b[0]} has been chosen, lmao")
        if letterChoice == choice2b[1]:
            print(f"{choice2b[1]} has been chosen, lmao")
    elif choice == choice[0]:
        letterChoice = input("E or F?").lower()
        if letterChoice == choice2c[0]:
            print(f"{choice2c[0]} has been chosen, time to die")
        if letterChoice == choice2c[1]:
            print(f"{choice2c[1]} has been chosen, lmao")"""
dict = {
"1":["a", "b"],
"2":["c", "d"],
"3":["e", "f"],
}
while True:
    print("Which one?")
    for key in dict:
        print(key)
    choice1 = input()
    print("CHOOSE WISELY")
    for value in dict[choice1]:
        print(value)
    choice2 = input()
    print(f"You have chosen {choice2}")